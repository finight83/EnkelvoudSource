#ifndef EVDBTBLUETOOTH_H
#define EVDBTBLUETOOTH_H
// ============================================================================
//  EVDBTBluetooth.h - A2DP sink, device mgmt, AVRCP passthrough, callbacks
//
//  NOTE: A2DP callbacks run on the Bluetooth stack task. They must NOT do NVS
//  access, blocking UART writes, or Serial.printf - all of that stalls the BT
//  controller and drops the link. Callbacks only set flags; loop() drains them.
//
//  For richer diagnostics, callbacks push short records into a small lock-free
//  ring buffer (btLogPush) which loop() drains via drainBtLogQueue(). The ring
//  uses only POD types and static string literals - no String, no heap, no
//  Serial - so it is safe from the BT task.
// ============================================================================
#include "EVDBTHelpers.h"
#include <esp_bt_main.h>
#include <esp_gap_bt_api.h>
#include <esp_avrc_api.h>

#include "esp_gap_bt_api.h"


// ---- Deferred BT event log (BT task -> loop()) -----------------------------
#define BT_LOG_QUEUE_LEN 32
struct BtLogEntry {
  uint32_t    ms;
  const char *tag;
  int         value;
};
BtLogEntry       btLogQueue[BT_LOG_QUEUE_LEN];
volatile uint8_t btLogHead = 0;   // producer index
volatile uint8_t btLogTail = 0;   // consumer index

// Called from the BT stack task. Overwrites nothing: if the queue is full the
// entry is silently dropped (diagnostics only, never load-bearing).
inline void btLogPush(const char *tag, int value) {
  uint8_t h    = btLogHead;
  uint8_t next = (uint8_t)((h + 1) % BT_LOG_QUEUE_LEN);
  if (next == btLogTail) return;
  btLogQueue[h].ms    = millis();
  btLogQueue[h].tag   = tag;
  btLogQueue[h].value = value;
  __sync_synchronize();   // slot contents before publishing head
  btLogHead = next;
}

inline void drainBtLogQueue() {
  while (btLogTail != btLogHead) {
    // [ring-acquire] Match the release barrier in btLogPush(): the
    // volatile head load must be observed before the non-volatile
    // slot payload is read. Without this, a compiler is free to
    // hoist e.ms / e.tag / e.value above the head compare on dual-
    // core ESP32 (BT task on core 0, loop() on core 1).
    __sync_synchronize();
    BtLogEntry &e = btLogQueue[btLogTail];
    if (gSerialLogLevel >= 3) {
      char lb[96];
      int  n = snprintf(lb, sizeof(lb), "[%10lu] [BT] %s (%d)\n",
                        (unsigned long)e.ms, e.tag, e.value);
      if (n > 0 && n < (int)sizeof(lb)) Serial.write((const uint8_t*)lb, n);
    }
    // [ring-release] Slot consumed before tail advances.
    __sync_synchronize();
    btLogTail = (uint8_t)((btLogTail + 1) % BT_LOG_QUEUE_LEN);
  }
}

// ---- Connect / disconnect sounds (DISABLED) --------------------------------
// [tone-dead-code-guarded]
// These functions write directly to dac1I2s while the A2DP writer is pushing
// frames into the same I2SStream through multiOutput. AudioTools::I2SStream
// is not thread-safe; a concurrent write corrupts the DMA ring and the peer
// drops the link within a few seconds. This was the reason the calls in
// handleDeviceConnected()/handleDeviceDisconnected() were removed. Keep the
// implementation behind #if 0 so nobody can re-enable the call sites by
// accident — un-#if 0 only if the tone is being routed to a separate
// I2SStream on a spare peripheral.
#if 0
inline void playTone(float freq, uint32_t durationMs) {
  if (!dac1I2s) return;
  const int channels = CHANNELS;
  const int bufferSize = 256;
  const size_t chunkBytes = (size_t)bufferSize * channels * sizeof(int16_t);
  if (dac1I2s->availableForWrite() < (int)chunkBytes) return;

  const int sampleRate = SAMPLE_RATE;
  const float amplitude = 0.008f;
  const int totalSamples = (sampleRate * (int)durationMs) / 1000;
  int16_t buffer[bufferSize * channels];

  for (int i = 0; i < totalSamples; i += bufferSize) {
    if (dac1I2s->availableForWrite() < (int)chunkBytes) return;
    int count = min(bufferSize, totalSamples - i);
    for (int j = 0; j < count; j++) {
      float t = (float)(i + j) / sampleRate;
      int16_t val = (int16_t)(sin(2.0 * PI * freq * t) * 32767.0f * amplitude);
      for (int c = 0; c < channels; c++) {
        buffer[j * channels + c] = val;
      }
    }
    dac1I2s->write((uint8_t*)buffer, (size_t)count * channels * sizeof(int16_t));
  }
}

inline void playConnectSound() {
  playTone(1000.0, 200);
}

inline void playDisconnectSound() {
  playTone(500.0, 200);
}
#endif  // [tone-dead-code-guarded]

// ---- Device list report ----------------------------------------------------
// Entirely heap-free: reads the NVS list into one stack buffer and formats
// the JSON in place with snprintf. One preferences.begin() per call (the
// old code opened it twice). No String anywhere, no fragmentation.
// Copies a MAC-like token keeping only [0-9A-Za-z:-] so it can never break
// out of a JSON string.
inline void copyJsonSafeMac(char *dst, size_t cap, const char *src, size_t len) {
  size_t o = 0;
  for (size_t i = 0; i < len && o + 1 < cap; i++) {
    char c = src[i];
    if (isalnum((unsigned char)c) || c == ':' || c == '-') dst[o++] = c;
  }
  dst[o] = '\0';
}

inline void sendBtDeviceList() {
  char safeCur[24];
  copyJsonSafeMac(safeCur, sizeof(safeCur), currentConnectedMac.c_str(),
                  currentConnectedMac.length());
  // [bt-devlist-buf-512] 256 B held only 14 MACs (14*18=252 + NUL).
  // 512 B = 28 MACs, well above any realistic pair history.
  static char devListBuf[512];
  size_t n = 0;
  preferences.begin("bt_devs", true);
  n = preferences.getBytes("dev_list", devListBuf, sizeof(devListBuf) - 1);
  preferences.end();
  devListBuf[n] = '\0';

  // [bt-list-buf-1024] 512 overflowed at 9 paired devices: header (~48 B)
  // + 9 * ~55 B = 543 B. When the snprintf below returned w >= remaining,
  // the loop broke and sent truncated JSON, which the S3 parser dropped.
  //
  // [bt-list-buf-cross-repo] THIS BUFFER IS A CROSS-REPO CONTRACT with
  // EnkelvoudServer/EVDPlayers.h's rxLine[]. The S3 reads the whole line
  // "BT_JSON|" + out into a fixed buffer and DROPS the line if it does not
  // fit. The server's buffer is BT_LINE_MAX = 1032 (8-byte prefix + this
  // 1024-byte payload). If you grow `out` here, grow BT_LINE_MAX there in
  // the same change, or the device list silently freezes on the dashboard
  // once enough devices are paired.
  static char out[1024];
  int len = snprintf(out, sizeof(out),
                     "BT_JSON|{\"activeMac\":\"%s\",\"devices\":[",
                     safeCur);
  if (len < 0) len = 0;
  if (len >= (int)sizeof(out)) len = (int)sizeof(out) - 1;

  // Snapshot once; activeBtVolume is volatile and could change mid-loop.
  const unsigned curVol = (unsigned)activeBtVolume;

  bool first = true;
  int  start = 0;
  while (start < (int)n) {
    int comma = start;
    while (comma < (int)n && devListBuf[comma] != ',') comma++;
    int macEnd = comma;
    // Trim spaces, tabs, AND trailing NULs left by legacy putString() writes.
    while (macEnd > start && (devListBuf[macEnd - 1] == ' ' ||
                              devListBuf[macEnd - 1] == '\t' ||
                              devListBuf[macEnd - 1] == '\0')) macEnd--;
    int macStart = start;
    while (macStart < macEnd && (devListBuf[macStart] == ' ' ||
                                 devListBuf[macStart] == '\t')) macStart++;
    int macLen = macEnd - macStart;
    if (macLen > 0) {
      bool isConn = (macLen == (int)currentConnectedMac.length()) &&
                    (strncasecmp(devListBuf + macStart,
                                 currentConnectedMac.c_str(), macLen) == 0);
      // [devlist-reserve-tail] Reserve 3 bytes ("]}" + NUL) so the
      // trailer can always be written and the JSON is never truncated
      // mid-object. The old check allowed snprintf() to partially write
      // and then bailed without appending "]}" — sending garbage to the
      // S3 which the parser discarded entirely, so the whole device
      // list vanished on overflow.
      char safeMac[24];
      copyJsonSafeMac(safeMac, sizeof(safeMac), devListBuf + macStart, (size_t)macLen);
      int w = snprintf(out + len, sizeof(out) - len - 3,
                       "%s{\"mac\":\"%s\",\"vol\":%u,\"conn\":%s}",
                       first ? "" : ",", safeMac,
                       curVol, isConn ? "true" : "false");
      if (w < 0 || len + w >= (int)sizeof(out) - 3) break;
      len += w;
      first = false;
    }
    if (comma >= (int)n) break;
    start = comma + 1;
  }
  if (len < (int)sizeof(out) - 2) {
    out[len++] = ']';
    out[len++] = '}';
    out[len]   = '\0';
  }
  cmdUart.println(out);
  // [devlist-log-verbose-only] Was unconditional. S3 sends
  // CMD:GET_BT_DEVICES every 5 s; the response is itself logged by
  // UART RX (now verbose-gated) and by this line. Together they put
  // ~24 log entries per minute into the S3's 16-slot ring, so the
  // ring cycles every 40 s and real warnings are evicted within that
  // window. Gate behind verbose; the byte count is only useful during
  // boot-time debugging anyway.
  if (gSerialLogLevel >= 3) {
    logLinef("BT device list sent (%d bytes)", len);
  }
  // [passthrough-arch] Report the DETECTED peer rate and the LIVE I2S
  // clock. getOutputRate() returns the constructor-time _dstRate (always
  // 48000) and is meaningless after the passthrough refactor — using it
  // here made the heartbeat report out=48000 while RATEDIAG reported
  // out=44100 one line earlier, contradicting itself.
  // [rateinfo-gated] RATEINFO has no consumer on the S3 side — it is
  // printed only for the local serial trace. Gate on verbose so it does
  // not fill the S3 log ring with 12 lines/minute of unrecognised text.
  if (a2dpResampler && gSerialLogLevel >= 3) {
    logLinef("RATEINFO|in=%d out=%u mode=%s",
             a2dpResampler->getDetectedInputRate(),
             (unsigned)g_currentI2sRateHz,
             a2dpResampler->getMode());
  }

  // [srcstat-extended] SRCSTAT carries the full loop-side picture so the S3
  // dashboard can tell "source board wedged" from "source board just idle".
  // All new keys are appended AFTER "src" so existing parsers that only look
  // for mac/vol/connected/avrc/a2dpC/a2dpD/volEv/uartL/src keep working
  // unchanged. hbAgo = -1 means "never seen a GET_BT_DEVICES since boot".
  char stats[640];
  long hbAgo = (lastHeartbeatMs == 0)
               ? -1L
               : (long)(millis() - lastHeartbeatMs);
  // [srcstat-fw-field] "fw" added so the server's currentSourceFwVersion is
  // populated instead of blank. New field is APPENDED after "hbAgo" so any
  // pre-existing parser that keys only on the earlier fields continues to
  // work unchanged. EVD_FW_VERSION is the macro from EnkelvoudSource.ino.
  snprintf(stats, sizeof(stats),
    "SRCSTAT|{\"mac\":\"%s\",\"vol\":%u,\"connected\":%d,\"avrc\":%d,"
    "\"a2dpC\":%lu,\"a2dpD\":%lu,\"volEv\":%lu,\"uartL\":%lu,\"src\":\"%s\","
    "\"a2dpCg\":%lu,\"a2dpDg\":%lu,"
    "\"volAp\":%lu,"
    "\"uartB\":%lu,\"uartOvf\":%lu,"
    "\"up\":%lu,\"heap\":%lu,\"minHeap\":%lu,\"maxBlock\":%lu,"
    "\"ticks\":%lu,\"srcCh\":%lu,\"hbAgo\":%ld,\"fw\":\"%s\"}",
    safeCur, (unsigned)activeBtVolume,
    a2dpSink && a2dpSink->is_connected() ? 1 : 0, avrcConnected ? 1 : 0,
    (unsigned long)diagA2dpConnected,
    (unsigned long)diagA2dpDisconnected,
    (unsigned long)diagVolumeEvents,
    (unsigned long)diagUartRxLines,
    activeSource.c_str(),
    (unsigned long)diagA2dpConnecting,
    (unsigned long)diagA2dpDisconnecting,
    (unsigned long)diagVolumeApplied,
    (unsigned long)diagUartRxBytes,
    (unsigned long)diagUartRxOverflow,
    (unsigned long)(millis() / 1000),
    (unsigned long)ESP.getFreeHeap(),
    (unsigned long)ESP.getMinFreeHeap(),
    (unsigned long)ESP.getMaxAllocHeap(),
    (unsigned long)diagLoopTicks,
    (unsigned long)diagSourceChanges,
    hbAgo,
    EVD_FW_VERSION);
  cmdUart.println(stats);
}

// ---- Connection / disconnection (run on loop(), not BT task) ---------------
// Heap-free NVS list handling. Matches the fixed-buffer style used in
// sendBtDeviceList(): read into a stack/static buffer, scan with pointer
// arithmetic, append with memcpy, write with putBytes. No String churn on
// every connect.
inline void handleDeviceConnected() {
  if (pendingPeerAddrValid) {
    currentConnectedMac = macToString(pendingPeerAddr);
  } else {
    esp_bd_addr_t *bda = a2dpSink->get_current_peer_address();
    currentConnectedMac = (bda != nullptr) ? macToString(*bda) : "CONNECTED";
  }
  logLine("BT Connected: " + currentConnectedMac);
  logLinef("  peer classification: %s",
           peerIsComputer ? "COMPUTER (10% hold, no fallback)"
                          : "OTHER (fallback to 127/127 armed)");

  reassertAtMs = lastConnectMs + 2500;
  // lastConnectMs / volEventReceived / volFallbackAtMs are set in
  // connection_state_changed() so they cover pre-loop volume events.

  // [bt-connect-volume-passthrough]
  // The connect volume is a TV/phone workaround, not a general rule. Those
  // peers ignore the sink's volume until their own remote is touched, so a
  // low connect volume was used to make "nothing happened" obvious and
  // audible. A PC is the opposite case: Windows sends an absolute-volume
  // event on connect and again on every slider move, and starting at 13/127
  // makes the first thing the user hears a level jump that reads as a
  // fault — and, when the event races the connect, as a dropout.
  //
  // So: pass through at unity for a recognised computer, keep the 10%
  // + fallback dance for everything else. The fallback in
  // serviceVolumeFallback() already refuses to touch a COMPUTER peer, so
  // the two paths agree.
  activeBtVolume = peerIsComputer ? 127 : CONNECT_FORCED_VOLUME;
  a2dpSink->set_volume(activeBtVolume);
  logLinef("BT volume set to %u/127 (%s)", (unsigned)activeBtVolume,
           peerIsComputer ? "COMPUTER passthrough"
                          : "non-computer, 10% then fallback");

  // playConnectSound() intentionally NOT called. It would write to dac1I2s
  // while the A2DP writer is pushing frames into the same I2SStream through
  // multiOutput. AudioTools::I2SStream is not thread-safe; a concurrent
  // write corrupts the DMA ring and the peer drops the link within a few
  // seconds. Re-enable only if the tone is routed to a separate I2SStream
  // on a spare peripheral.

  // [devlist-storage-type-consistent]
  // All THREE functions that touch "dev_list" MUST use the SAME NVS
  // storage primitive. sendBtDeviceList() and removeBtDevice() read via
  // getBytes(); handleDeviceConnected() previously wrote via putString().
  // NVS enforces strict type matching (nvs_get_blob returns
  // ESP_ERR_NVS_TYPE_MISMATCH on a STRING-type key), so getBytes() in
  // sendBtDeviceList() ALWAYS returned 0 → BT_JSON "devices" array was
  // always empty. removeBtDevice() failed at the same getBytes() call,
  // logging "NVS read failed" on every invocation, so putBytes() was
  // unreachable and the NVS list could never be pruned after the first
  // write. Rewritten here to read/write BYTES so all three agree.
  //
  // Migration: a device that boots with a legacy STRING-type entry will
  // get getBytes() == 0 (treat as empty), append currentConnectedMac,
  // and putBytes() replaces the STRING entry with a BLOB. Any devices
  // that were already in the STRING list are lost from the DISPLAY
  // list — but they were already invisible to BT_JSON, so no regression.
  // The BT stack's bond list is separate and untouched.
  if (currentConnectedMac != "CONNECTED") {
    if (!preferences.begin("bt_devs", false)) {
      logLine("handleDeviceConnected: NVS begin failed; list unchanged");
    } else {
      char listBuf[256];
      size_t n = preferences.getBytes("dev_list", listBuf, sizeof(listBuf) - 1);
      if (n >= sizeof(listBuf)) n = sizeof(listBuf) - 1;   // defensive
      listBuf[n] = '\0';

      bool isKnown = false;
      size_t p = 0;
      while (p < n) {
        size_t s = p;
        while (p < n && listBuf[p] != ',') p++;
        size_t e = p;
        while (e > s && (listBuf[e - 1] == ' ' || listBuf[e - 1] == '\t' ||
                         listBuf[e - 1] == '\0')) e--;
        while (s < e && (listBuf[s] == ' ' || listBuf[s] == '\t')) s++;
        size_t len = e - s;
        if (len == currentConnectedMac.length() &&
            strncasecmp(listBuf + s, currentConnectedMac.c_str(), len) == 0) {
          isKnown = true;
          break;
        }
        if (p < n) p++;
      }

      if (!isKnown) {
        size_t needed = currentConnectedMac.length() + (n > 0 ? 1 : 0);
        if (n + needed >= sizeof(listBuf)) {
          logLine("handleDeviceConnected: list full; MAC not added");
        } else {
          if (n > 0) listBuf[n++] = ',';
          memcpy(listBuf + n, currentConnectedMac.c_str(),
                 currentConnectedMac.length());
          n += currentConnectedMac.length();
          // putBytes() overwrites the legacy STRING-type entry in place.
          // No preferences.remove() first — NVS handles the type swap
          // during the write.
          size_t written = preferences.putBytes("dev_list", listBuf, n);
          if (written == n) {
            logLine("New BT device added to list");
          } else {
            logLine("handleDeviceConnected: NVS write failed");
          }
        }
      }
      preferences.end();
    }
  }

  sendBtDeviceList();
}

inline void handleDeviceDisconnected() {
  // set_auto_reconnect(true) fires DISCONNECTED on every failed retry while
  // idle. Only react to a real connected -> disconnected transition, i.e. we
  // still have a MAC recorded from the last successful connect.
  if (currentConnectedMac.length() == 0) {
    return;
  }
  logLine("BT Disconnected.");
  // playDisconnectSound() intentionally NOT called (same I2S thread-safety
  // reason as the connect tone; even more likely to race because the sink
  // is often still draining the last frames into dac1I2s at this moment).
  currentConnectedMac = "";
  sendBtDeviceList();
}

// ---- Device removal --------------------------------------------------------
// Heap-free removal: single buffer read, in-place compaction, single write.
// Signature moved to const char* so the caller does not need a String.
inline void removeBtDevice(const char *macToRemove) {
  if (macToRemove == nullptr || *macToRemove == '\0') return;

  static char listBuf[256];
  static char keepBuf[256];

  if (!preferences.begin("bt_devs", false)) {
    logLine("NVS open failed on remove - device list unchanged");
  } else {
    // [remove-error-vs-empty]
    // getBytes() returns 0 for BOTH "key absent" (empty list) and
    // "NVS read error". Previously both cases fell through to
    // preferences.remove("dev_list"), so a transient NVS error
    // silently erased every paired device. Only treat it as empty
    // if isKey() confirms the key really is absent.
    if (!preferences.isKey("dev_list")) {
      preferences.end();
      logLine("removeBtDevice: list already empty");
      return;
    }
    size_t n = preferences.getBytes("dev_list", listBuf, sizeof(listBuf) - 1);
    if (n == 0) {
      preferences.end();
      logLine("removeBtDevice: NVS read failed - device list unchanged");
      return;
    }
    listBuf[n] = '\0';

    size_t keepN = 0;
    size_t p = 0;
    while (p < n) {
      size_t start = p;
      while (p < n && listBuf[p] != ',') p++;
      size_t end = p;
      while (end > start && (listBuf[end - 1] == ' ' ||
                             listBuf[end - 1] == '\t' ||
                             listBuf[end - 1] == '\0')) end--;
      while (start < end && (listBuf[start] == ' ' || listBuf[start] == '\t')) start++;
      size_t len = end - start;
      if (len > 0 && !(len == strlen(macToRemove) &&
                       strncasecmp(listBuf + start, macToRemove, len) == 0)) {
        size_t needed = len + (keepN > 0 ? 1 : 0);
        if (keepN + needed < sizeof(keepBuf)) {
          if (keepN > 0) keepBuf[keepN++] = ',';
          memcpy(keepBuf + keepN, listBuf + start, len);
          keepN += len;
        }
      }
      if (p < n) p++;
    }
    // Zero-length blob handling is version-dependent in ESP-IDF: nvs_set_blob
    // with length 0 either erases the key (OK) or returns an error (removal
    // silently dropped). Handle the empty case explicitly via remove().
    if (keepN == 0) {
      if (!preferences.remove("dev_list")) {
        logLine("NVS remove failed - empty list not persisted");
      }
    } else if (preferences.putBytes("dev_list", keepBuf, keepN) != keepN) {
      logLine("NVS write failed - removal not persisted");
    }
    preferences.end();
  }

  esp_bd_addr_t bda;
  if (stringToMac(String(macToRemove), bda)) {
    esp_err_t e = esp_bt_gap_remove_bond_device(bda);
    if (e != ESP_OK && e != ESP_ERR_NOT_FOUND) {
      logLinef("bond remove failed: %d", (int)e);
    }
  } else {
    logLinef("bad MAC for removal: %s", macToRemove);
  }
  if (strcasecmp(currentConnectedMac.c_str(), macToRemove) == 0) a2dpSink->disconnect();

  logLinef("Removed NVS device: %s", macToRemove);
  sendBtDeviceList();
}

// ---- A2DP callbacks (BT task context - flag only) --------------------------
// Connect policy:
//   * COMPUTER peer — pass through. Connect at unity (127) and apply every
//     AVRCP volume the host sends. [bt-connect-volume-passthrough]
//   * everything else (phone, TV) — force 10% (13/127) once on connect, then
//     honour the device's own volume changes, with the unity fallback in
//     serviceVolumeFallback() if no real event ever arrives.
//
// The suppression machinery below is only needed for the non-computer path
// now: a PC sends SetAbsoluteVolume on connect and again on every slider
// move, and forcing it back to 13 mid-stream is heard as a dropout, so a
// COMPUTER peer is deliberately exempt from rule (1) and (2).
static volatile bool inVolumeApply = false;

inline void volume_changed_callback(int vol) {
  if (inVolumeApply) return;   // reentrancy: our own set_volume() echo
  diagVolumeEvents++;
  if (vol < 0)   vol = 0;
  if (vol > 127) vol = 127;

  const uint32_t sinceConnect = (uint32_t)(millis() - lastConnectMs);

  // 1) Echo of our own forced 13 — swallow silently.
  if (sinceConnect < CONNECT_VOL_GUARD_MS && vol == CONNECT_FORCED_VOLUME) {
    btLogPush("AVRCP vol SUPPRESSED (connect echo 13)", vol);
    return;
  }

  // 2) Non-computer peers only: a phone that reconnects with its own volume
  //    already at max also sends 127, and a phone sends AVRCP volume only
  //    ONCE per connection. Suppressing that single event would leave the
  //    phone at 10% for the whole session (the "sounds quiet" report), so it
  //    is applied normally — this branch exists for the TV/other cases where
  //    127 arrives as an echo of the connect volume.
  //
  //    [bt-connect-volume-passthrough] A COMPUTER peer is exempt: its 127 is
  //    the real host volume, and re-forcing 13 in the middle of a stream is
  //    a level jump the user hears as a fault.
  if (!peerIsComputer && sinceConnect < PC_VOL_HOLD_MS && vol == 127) {
    inVolumeApply = true;
    a2dpSink->set_volume(CONNECT_FORCED_VOLUME);
    inVolumeApply = false;
    activeBtVolume   = CONNECT_FORCED_VOLUME;
    volEventReceived = true;
    volFallbackAtMs  = 0;
    btLogPush("AVRCP vol IGNORED (max echo, held at 13)", vol);
    return;
  }

  // 3) Real user volume (phone remembered value, user turning PC/TV up
  //    after the hold window, etc.). Apply normally.
  inVolumeApply = true;
  a2dpSink->set_volume(vol);
  inVolumeApply = false;
  activeBtVolume   = (uint8_t)vol;
  volEventReceived = true;
  volFallbackAtMs  = 0;
  diagVolumeApplied++;
  btLogPush("AVRCP vol applied", vol);
}

// ---- A2DP callbacks (BT task context - flag only) --------------------------
inline void sample_rate_changed(uint16_t rate) {
  if (a2dpResampler) a2dpResampler->setPeerRate(rate);
}

inline void connection_state_changed(esp_a2d_connection_state_t state, void*) {
  switch (state) {
    case ESP_A2D_CONNECTION_STATE_DISCONNECTED:
      if (a2dpResampler) a2dpResampler->reset();
      diagA2dpDisconnected++;
      avrcConnected = false;
      lastConnectMs    = 0;
      volEventReceived = false;
      volFallbackAtMs  = 0;
      btLogPush("A2DP state: DISCONNECTED", diagA2dpDisconnected);
      pendingConnectEvent    = false;
      pendingDisconnectEvent = true;
      break;

    case ESP_A2D_CONNECTION_STATE_CONNECTING:
      diagA2dpConnecting++;
      btLogPush("A2DP state: CONNECTING", diagA2dpConnecting);
      break;

    case ESP_A2D_CONNECTION_STATE_CONNECTED: {
      diagA2dpConnected++;
      avrcConnected = true;

      const bool freshConnect =
          (lastConnectMs == 0) ||
          ((uint32_t)(millis() - lastConnectMs) >= 5000);
      if (freshConnect) {
        lastConnectMs    = millis();
        volEventReceived = false;
        volFallbackAtMs  = millis() + CONNECT_VOL_FALLBACK_MS;
      }

      esp_bd_addr_t *bda = a2dpSink->get_current_peer_address();
      if (bda != nullptr) {
        memcpy(pendingPeerAddr, *bda, sizeof(esp_bd_addr_t));
        peerIsComputer = isComputerMac(*bda);
        __sync_synchronize();
        pendingPeerAddrValid = true;
      } else {
        pendingPeerAddrValid = false;
        peerIsComputer      = false;
      }
      btLogPush(peerIsComputer ? "A2DP state: CONNECTED (COMPUTER)"
                               : "A2DP state: CONNECTED (OTHER)",
                diagA2dpConnected);
      __sync_synchronize();
      pendingDisconnectEvent = false;

      // [spurious-connected-guard] Only schedule handleDeviceConnected()
      // for a genuine fresh connect. Re-fired spurious CONNECTED inside
      // the 5 s window previously forced activeBtVolume back to 13 —
      // the "Windows jumps to 10 %" report.
      if (freshConnect) {
        pendingConnectEvent = true;
      }
      break;
    }

    case ESP_A2D_CONNECTION_STATE_DISCONNECTING:
      diagA2dpDisconnecting++;
      avrcConnected = false;
      btLogPush("A2DP state: DISCONNECTING", diagA2dpDisconnecting);
      break;

    default:
      btLogPush("A2DP state: UNKNOWN", (int)state);
      break;
  }
}

// ---- AVRCP Controller (ESP-IDF v5.x) ---------------------------------------
inline void avrcCtCallback(esp_avrc_ct_cb_event_t event,
                           esp_avrc_ct_cb_param_t *param) {
  if (event == ESP_AVRC_CT_CONNECTION_STATE_EVT) {
    avrcConnected = param->conn_stat.connected;
    btLogPush(avrcConnected ? "AVRCP CT connected" : "AVRCP CT disconnected", 0);
  }
}

inline void initAvrcController() {
  esp_err_t err = esp_avrc_ct_init();
  if (err == ESP_ERR_INVALID_STATE) {
    // [avrc-owned-by-library]
    // BluetoothA2DPSink::start() already initialised AVRCP CT and
    // registered its own callback. Re-registering ours would REPLACE
    // the library's slot (ESP-IDF keeps one callback per profile),
    // killing metadata notifications and absolute-volume handling.
    // The library's callback handles the passthrough responses, so
    // esp_avrc_ct_send_passthrough_cmd() still works — avrcConnected
    // is therefore a proxy ("A2DP link is up"), which is exactly how
    // requestAvrcKey()/serviceAvrcKey() use it. Log once at warn.
    LOGW("AVRCP CT already owned by BluetoothA2DPSink — using A2DP link "
         "state as AVRCP-connected proxy");
    return;
  }
  if (err != ESP_OK) {
    LOGE("AVRCP CT init failed: %d", (int)err);
    return;
  }
  esp_avrc_ct_register_callback(avrcCtCallback);
  logLine("AVRCP Controller initialised.");
}

// Key codes: 0x4B = next, 0x4C = previous. Key states: 0 = pressed, 1 = released.
inline void requestAvrcKey(uint8_t keyCode) {
  if (!avrcConnected) {
    logLinef("AVRCP not connected - couldn't send key 0x%02X", (unsigned)keyCode);
    return;
  }
  avrcPendingKey   = keyCode;
  avrcPendingState = 1;
  avrcPressAtMs    = millis();
  logLinef("AVRCP key queued: 0x%02X", (unsigned)keyCode);
}

// AVRCP passthrough requires BOTH pressed (0) and released (1) events. The
// previous code sent only RELEASED, which some phones treat as malformed.
// Both messages of one tap must share the same transaction label (avrcTl).
//
// State machine:
//   0 : idle
//   1 : key queued     -> send PRESSED, record press time, advance to 2
//   2 : press sent     -> after PRESS_HOLD_MS send RELEASED, advance to 3
//   3 : release sent   -> clear, advance to 0
inline void serviceAvrcKey() {
  static const uint32_t PRESS_HOLD_MS = 50;

  if (avrcPendingState == 1) {
    esp_avrc_ct_send_passthrough_cmd(avrcTl, avrcPendingKey, 0);  // PRESSED
    avrcPressAtMs    = millis();
    avrcPendingState = 2;
    logLinef("AVRCP press sent (tl=%u, key=0x%02X)",
             (unsigned)avrcTl, (unsigned)avrcPendingKey);
  } else if (avrcPendingState == 2 &&
             (uint32_t)(millis() - avrcPressAtMs) >= PRESS_HOLD_MS) {
    esp_avrc_ct_send_passthrough_cmd(avrcTl, avrcPendingKey, 1);  // RELEASED
    uint8_t usedTl = avrcTl;
    avrcTl = (uint8_t)((avrcTl + 1) & 0x0F);
    avrcPendingState = 3;
    logLinef("AVRCP release sent (tl=%u, key=0x%02X)",
             (unsigned)usedTl, (unsigned)avrcPendingKey);
  } else if (avrcPendingState == 3) {
    avrcPendingState = 0;
    avrcPendingKey   = 0;
  }
}

// ---- Bring-up --------------------------------------------------------------
inline void startBluetooth() {
  auto config1 = dac1I2s->defaultConfig(TX_MODE);
  config1.port_no         = 0;
  // [restore-resampling] Fixed output clock. The resampler converts peer
  // rate -> _dstRate (SAMPLE_RATE = 48000) so the I2S DMA always drains at
  // exactly the rate the S3 expects (SOURCE_SAMPLE_RATE = 48000). This is
  // what eliminates the ±0.06 % peer wobble that was reaching the S3 and
  // jittering the UDP pacer. Do NOT restore 44100 here: the S3 then pays
  // the cubic resample path on every chunk and its ratio chases the peer
  // crystal.
  config1.sample_rate     = SAMPLE_RATE;   // 48000 from EVDBTConfig.h
  config1.channels        = CHANNELS;
  config1.bits_per_sample = BITS_PER_SAMPLE;
  config1.pin_bck         = PIN_OUT1_BCLK;
  config1.pin_ws          = PIN_OUT1_LRC;
  config1.pin_data        = PIN_OUT1_DOUT;
  config1.is_master       = true;
  // AudioTools' ESP32 I2S drivers interpret buffer_size as frames, not
  // bytes. 32 x 256 stereo 16-bit frames is 32 KiB, about 171 ms at 48 kHz.
  config1.buffer_count    = 32;
  config1.buffer_size     = 256;
  bool ok = dac1I2s->begin(config1);
  if (!ok) {
    LOGE("I2S begin FAILED at buffer_count=32 - retrying 16");
    config1.buffer_count = 16;
    ok = dac1I2s->begin(config1);
  }
  if (!ok) {
    LOGE("I2S begin FAILED - audio pipeline dead");
    return;
  }

  multiOutput->clear();
  multiOutput->add(*dac1I2s);

  a2dpSink->start("Enkelvoud");
  a2dpRunning = true;

  // [bt-scan-mode] Explicitly enable BT discoverability + connectability.
  // Some BluetoothA2DPSink builds leave the controller in a state where it
  // accepts previously-bonded reconnects but disappears from new pairings.
  // The captured diagnostic showed 100+ s of uptime with zero A2DP events
  // and an empty paired-device list — consistent with the sink being
  // invisible to the peer. This call is idempotent and safe after start().
  esp_err_t scanErr = esp_bt_gap_set_scan_mode(
      ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
  if (scanErr == ESP_OK) {
    logLine("BT discoverable + connectable (scan mode set)");
  } else {
    logLinef("BT scan mode set FAILED: %d", (int)scanErr);
  }

  // Rate adapter is upstream of this I2S stream; report its current state
  // so the source-board log shows what path audio is on.
  // [boot-log-live-rate] Report the actual I2S clock, not SAMPLE_RATE —
  // startBluetooth() configures 44100 here, and SAMPLE_RATE (48000) is
  // a dead macro in the passthrough architecture. Previously the boot
  // log claimed 48000 while the DMA ran at 44100, contradicting the
  // very first RATEDIAG line emitted 60 s later.
  if (a2dpResampler) {
    logLinef("Bluetooth Active. I2S @ %u Hz; resampler: %s.",
             (unsigned)g_currentI2sRateHz, a2dpResampler->getMode());
  } else {
    logLinef("Bluetooth Active. I2S @ %u Hz (resampler MISSING).",
             (unsigned)g_currentI2sRateHz);
  }
}

#endif // EVDBTBLUETOOTH_H