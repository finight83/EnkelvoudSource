#ifndef EVDBTMAIN_H
#define EVDBTMAIN_H
// ============================================================================
//  EVDBTMain.h - setup(), loop(), deferred-event plumbing, periodic diagnostics
//
//  NOTE: setup() and loop() MUST NOT be 'inline' - the Arduino core's main.cpp
//  references them as external symbols.
// ============================================================================
#include "EVDBTHelpers.h"
#include "EVDBTResampler.h"
#include "EVDBTOta.h"       // [ota-self-containment] otaServiceWatchdog()
#include "EVDBTBluetooth.h"
#include "EVDBTInputs.h"
#include "EVDBTUart.h"

// ---- Deferred BT event drain -----------------------------------------------
inline void handleDeferredBtEvents() {
  if (pendingConnectEvent) {
    pendingConnectEvent = false;
    handleDeviceConnected();
  }
  if (pendingDisconnectEvent) {
    pendingDisconnectEvent = false;
    handleDeviceDisconnected();
  }
}

// ---- Volume re-assert after connect ----------------------------------------
inline void serviceVolumeReassert() {
  // Signed difference is wrap-safe across millis() rollover. The plain
  // `millis() >= reassertAtMs` comparison would mis-fire once every ~49.7 d.
  if (reassertAtMs != 0 && (int32_t)(millis() - reassertAtMs) >= 0) {
    reassertAtMs = 0;
    if (a2dpRunning && a2dpSink->is_connected()) {
      a2dpSink->set_volume(activeBtVolume);
      logLinef("Volume re-asserted after connect: %u", (unsigned)activeBtVolume);
    }
  }
}

// ---- Reconnect watchdog ----------------------------------------------------
// No ESP.restart() here. set_auto_reconnect(true) already handles reconnects;
// an unconditional restart reboot-loops the device every ~80 s whenever the
// phone is off or out of range. The watchdog now only logs.
inline void serviceReconnectWatchdog() {
  static unsigned long lastAttemptMs = 0;
  static uint8_t       fails         = 0;

  if (a2dpRunning && !a2dpSink->is_connected()) {
    if (millis() - lastAttemptMs > RECONNECT_IDLE_MS) {
      lastAttemptMs = millis();
      if (fails < 255) fails++;

      // [reconnect-log-milestones]
      // Was one line every 8 s. A 30-minute unplugged period produced
      // 225 entries, filling the S3's 16-slot log ring in ~2 minutes
      // and evicting every other diagnostic. Log at milestones (1, 5,
      // 10, 25, 50, 100, 255) and once per 60 s in between so the
      // operator can still see "still trying" without the flood.
      const bool milestone =
          (fails == 1 || fails == 5 || fails == 10 || fails == 25 ||
           fails == 50 || fails == 100 || fails == 255);
      static uint32_t lastDetailLogMs = 0;
      const bool slowTick =
          (lastDetailLogMs == 0) ||
          ((uint32_t)(millis() - lastDetailLogMs) > 60000UL);

      if (milestone || slowTick) {
        lastDetailLogMs = millis();
        logLinef("BT idle - reconnect pending (fail #%u)", (unsigned)fails);
      }
    }
  } else {
    fails              = 0;
    lastAttemptMs      = millis();   // reset so next disconnect starts fresh
  }
}

// ---- BT stall watchdog -----------------------------------------------------
// [bt-stall-watchdog]
// Distinguishes "connected but silent" from "not connected". is_connected()
// stays true through a BT stack stall, a peer that stopped sending, or a
// deadlocked write() in the resampler's backpressure path. All three produce
// the same downstream symptom: silence at the sink, which the S3 server
// interprets as a lower source rate (and then miscommits because its window
// counter cannot tell "fewer frames" from "silent frames").
//
// Recovery ladder (each step waits for the previous to have a chance):
//   >3 s silent:   log once. Do NOT act; transient stalls recover on their own.
//   >10 s silent:  a2dpSink->disconnect(). Forces reconnect, which re-runs
//                  A2DP handshake and clears BT stack deadlocks.
//   >30 s silent:  log CRITICAL after 2 failed recoveries. No safe soft
//                  recovery exists at this point; operator must power-cycle.
//
// Rate-limited to one log per level per 5 s so a permanently-stalled board
// cannot flood its own 50-slot ring buffer into uselessness.
// ---- BT stall watchdog -----------------------------------------------------
// [idle-does-not-mean-stalled]
// This function is DIAGNOSTIC-ONLY. It must never call a2dpSink->disconnect().
//
// Historical bug: at 10 s of "no write() calls" it disconnected the peer to
// "force reconnect". That signal cannot distinguish two very different states:
//
//   * Peer paused by the user (phone/PC playback paused, TV connected but
//     not streaming yet). Link is healthy, is_connected() == true, AVRCP
//     events still arrive. Disconnecting here kicks a perfectly good session
//     for no reason — every pause longer than 10 s triggered a reconnect
//     handshake, and some peers drop the pairing entirely as a result.
//
//   * BT stack genuinely deadlocked. is_connected() returns true but no
//     frames arrive. Sending disconnect() into the stack usually gets
//     absorbed by the same deadlock it's supposed to break; only a reboot
//     reliably clears it. So the auto-disconnect rarely helped even in the
//     case it was designed for.
//
// Real disconnects are handled by the link layer (ESP_A2D_CONNECTION_
// STATE_DISCONNECTED) and by set_auto_reconnect(true), which has been armed
// on the sink since startBluetooth(). The watchdog's remaining value is
// purely visibility: the two log lines below make "peer idle" distinguishable
// from "peer spamming" from "audio silently broken" in the source board log
// and the S3 dashboard's Source tab.
inline void serviceBtStallWatchdog() {
  if (!a2dpRunning || !a2dpSink || !a2dpSink->is_connected()) return;
  if (!a2dpResampler) return;

  static uint32_t lastLog3sMs  = 0;

  uint32_t now    = millis();
  uint32_t lastWr = a2dpResampler->getLastWriteMs();
  if (lastWr == 0) return;               // no write since connect — nothing to age

  uint32_t silentMs = now - lastWr;

  // [idle-log-once-per-minute] Was one line per 5 s of idle, i.e. 360
  // lines during a 30-minute pause. That noise dominated the S3 log
  // ring. One line per idle-transition + one per 60 s thereafter is
  // still enough to prove the link stayed up without flooding.
  static bool idleLogged = false;
  if (silentMs < 3000) {
    idleLogged = false;
  } else if (!idleLogged || (now - lastLog3sMs) > 60000) {
    lastLog3sMs = now;
    idleLogged  = true;
    logLinef("BT idle: %lu ms since last write (mode=%s) — link kept up",
             (unsigned long)silentMs, a2dpResampler->getMode());
  }

  // [stall-watchdog-no-redundant-branch]
  // Was: a second log every 15 s starting at 30 s of idle. That doubled
  // up with the once-per-minute branch above on every 60 s boundary,
  // producing two log entries with different wording for the same
  // event. Removed; the once-per-minute branch is sufficient to prove
  // the link is up during a long pause, and any operator investigating
  // a silence period will look at RATEDIAG (now 60 s at level 1)
  // alongside BT idle anyway.
}

// [rate-wd-decls-moved] File-scope state shared between
// applyDetectedSourceRate() (below) and serviceRateChangeWatchdog() (further
// below). Must be declared BEFORE applyDetectedSourceRate() — that function
// writes both, and the compiler does not look ahead in a translation unit.
// Kept as plain file-scope statics (not in EVDBTState.h) because they are
// specific to this rate-change path and have no cross-TU consumers.
static uint32_t g_lastRateChangeMs = 0;
static uint32_t g_prevI2sRateHz    = 44100;

// ---- Apply detected rate to I2S -------------------------------------------
// [passthrough-arch]
// The BT source no longer resamples. When the rate detector locks onto a
// peer rate that differs from the current I2S clock, tear down and restart
// the I2S stream at the new rate. Called from loop() only — never from
// the BT task.
//
// Safety:
//   - a2dpResampler->setSuspended(true) first, so any in-flight write()
//     returns immediately and stops touching the I2S DMA ring
//   - delay(20) lets any BT-task write() that already passed the suspend
//     check finish; one A2DP chunk is 5-20 ms, so 20 is enough
//   - multiOutput->clear() before end() so the MultiOutput sink list is
//     empty during the tear-down
//   - on begin() failure, revert to 48000 so the node is never silent
inline void applyDetectedSourceRate() {
  if (g_i2sRateChangeInProgress) return;
  if (!a2dpResampler || !dac1I2s || !multiOutput) return;
  if (!a2dpResampler->isLocked()) return;

  int detected = a2dpResampler->getDetectedInputRate();
  if (detected <= 0) return;
  if ((uint32_t)detected == g_currentI2sRateHz) return;

  // [pc-rate-fix] Reject PC rates outside {44100,48000}.
  if (peerIsComputer && detected != 44100 && detected != 48000) {
    logLinef("[RATE-REJECT] PC peer: implausible %d Hz — forcing 44100",
             detected);
    a2dpResampler->setPeerRate(44100);
    return;
  }

  if (g_currentI2sRateHz == 44100 || g_currentI2sRateHz == 48000) {
    g_prevI2sRateHz = g_currentI2sRateHz;
  }
  g_lastRateChangeMs = millis();

  g_i2sRateChangeInProgress = true;
  logLinef("I2S rate change: %u -> %d Hz",
           (unsigned)g_currentI2sRateHz, detected);

  a2dpResampler->setSuspended(true);
  delay(20);

  multiOutput->clear();
  dac1I2s->end();

  auto config = dac1I2s->defaultConfig(TX_MODE);
  config.port_no         = 0;
  config.sample_rate     = detected;
  config.channels        = CHANNELS;
  config.bits_per_sample = BITS_PER_SAMPLE;
  config.pin_bck         = PIN_OUT1_BCLK;
  config.pin_ws          = PIN_OUT1_LRC;
  config.pin_data        = PIN_OUT1_DOUT;
  config.is_master       = true;
  config.buffer_count    = 32;
  config.buffer_size     = 1024;

  bool ok = dac1I2s->begin(config);
  if (!ok) {
    logLinef("I2S begin at %d Hz FAILED - retrying buffer_count=16", detected);
    config.buffer_count = 16;
    ok = dac1I2s->begin(config);
  }
  if (!ok) {
    logLinef("I2S begin at %d Hz FAILED - reverting to 44100", detected);
    config.sample_rate  = 44100;
    config.buffer_count = 32;
    ok = dac1I2s->begin(config);
    if (!ok) {
      // [i2s-recovery-failed-no-retry-storm]
      // Pretend the clock *is* at 'detected' so the entry check
      // ((uint32_t)detected == g_currentI2sRateHz) short-circuits on
      // the next loop() pass. Without this, the function re-fires every
      // iteration, tearing down an already-dead DMA ring and spamming
      // the 115200 UART (~5 KB/s of log) until the operator reboots.
      a2dpResampler->setSuspended(false);
      g_i2sRateChangeInProgress = false;
      g_currentI2sRateHz        = (uint32_t)detected;
      logLine("I2S recovery FAILED - audio will be silent until reboot");
      return;
    }
    g_currentI2sRateHz = 44100;
  } else {
    g_currentI2sRateHz = (uint32_t)detected;
  }

  multiOutput->add(*dac1I2s);
  a2dpResampler->setSuspended(false);
  g_i2sRateChangeInProgress = false;
  logLinef("I2S now at %u Hz (passthrough)", (unsigned)g_currentI2sRateHz);
}

// [pc-rate-fix] Belt-and-braces: if the I2S clock ends up on a non-
// standard rate and no writes have flowed for >2 s, revert to the last
// safe rate. Catches any path that slipped past applyDetectedSourceRate()
// (e.g. non-PC peer with a genuine 16 kHz source that the DMA cannot
// keep up with).
//
// [rate-wd-decls-moved] The two file-scope statics that used to live here
// are now declared ABOVE applyDetectedSourceRate() — that function reads
// and writes them, and C++ requires declaration before use in a single
// translation unit. Keeping them here produced:
//   error: 'g_prevI2sRateHz' was not declared in this scope
inline void serviceRateChangeWatchdog() {
  if (!a2dpRunning || !a2dpSink || !a2dpSink->is_connected()) return;
  if (!a2dpResampler) return;
  if (g_i2sRateChangeInProgress) return;
  uint32_t r = g_currentI2sRateHz;
  if (r == 44100 || r == 48000) return;            // safe rate, nothing to do
  if ((millis() - g_lastRateChangeMs) < 5000) return;
  uint32_t lastWr = a2dpResampler->getLastWriteMs();
  if (lastWr == 0) return;
  if ((millis() - lastWr) < 2000) return;          // audio still flowing
  logLinef("[RATE-WD] no audio >2s at %u Hz — reverting to %u Hz",
           (unsigned)r, (unsigned)g_prevI2sRateHz);
  // [rate-wd-no-authoritative-lock]
  // Do NOT call setPeerRate(): it sets _peerRateKnown, which
  // permanently disables Phase-2 codec-switch detection for this
  // connection. A later genuine 44.1 <-> 48 kHz switch would then be
  // missed and audio would play off-pitch until the peer reconnects.
  // Instead, force the *detected* rate back to the safe value so the
  // next applyDetectedSourceRate() pass rebuilds the I2S clock. If
  // the peer is genuinely still at 'r', the resampler will re-lock
  // naturally when its write() throughput matches 'r' again.
  a2dpResampler->reset();                       // clears _peerRateKnown
  a2dpResampler->setPeerType(peerIsComputer);   // preserve hint
  // reset() zeroes _srcRate, so write() re-enters Phase 1 and will
  // re-snap on the next 2 s window. Note the safe rate we WANT is
  // g_prevI2sRateHz, but the resampler will re-detect whatever the
  // peer is actually sending — which is exactly what we want:
  // applyDetectedSourceRate() will then rebuild I2S to match.
  g_lastRateChangeMs = millis();
}

// ---- Rate-lock logger (loop context) ---------------------------------------
// The resampler cannot log from its own write() because that runs on the
// BT stack task. Instead loop() polls the lock state once every iteration
// and logs the transition. Cheap: one read of two ints per pass.
inline void serviceRateLog() {
  if (!a2dpResampler) return;
  static int  lastLockedRate = -1;
  static bool lastLockedMode = false;
  static bool lastRateKnown = false;
  int  rate = a2dpResampler->getDetectedInputRate();
  bool locked = a2dpResampler->isLocked();
  bool rateKnown = a2dpResampler->hasPeerRate();
  if (locked && (!lastLockedMode || rate != lastLockedRate || rateKnown != lastRateKnown)) {
    // [log-honest-out-rate] Was g_currentI2sRateHz, which is a stale
    // 44100 since applyDetectedSourceRate() was removed from loop().
    // getOutputRate() returns _dstRate = SAMPLE_RATE = 48000, which is
    // what the resampler actually emits and what the I2S DMA is clocked
    // at. Without this fix the log prints "44100 -> 44100 (resampling)"
    // which is self-contradictory and made the whole log untrustworthy.
    logLinef("A2DP rate: %d Hz -> %d Hz (%s, %s)",
             rate, a2dpResampler->getOutputRate(), a2dpResampler->getMode(),
             rateKnown ? "codec" : "assumed");
    lastLockedRate = rate;
    lastLockedMode = true;
    lastRateKnown = rateKnown;
  } else if (!locked) {
    // Reset so a reconnect's fresh detection logs again.
    lastLockedRate = -1;
    lastLockedMode = false;
  }

  static uint32_t lastPeriodicMs = 0;
  uint32_t nowMs = millis();
  // [ratediag-60s-or-verbose] Was unconditional every 10 s. That cadence
  // fills the S3 log ring (16 entries) with routine text and pushes real
  // warnings out inside 30 s. 60 s keeps a slow-state audit trail;
  // gSerialLogLevel >= 3 restores the 10 s cadence when the operator is
  // actively debugging.
  const uint32_t periodMs = (gSerialLogLevel >= 3) ? 10000UL : 60000UL;
  if (nowMs - lastPeriodicMs >= periodMs) {
    lastPeriodicMs = nowMs;
    uint32_t lastWr = a2dpResampler->getLastWriteMs();
    uint32_t sil    = (lastWr == 0) ? 0 : (nowMs - lastWr);
    logLinef("RATEDIAG|in=%d out=%d mode=%s rate=%s sil=%lu gapMax=%lu outMax=%lu bufShort=%lu dropped=%lu",
             rate, a2dpResampler->getOutputRate(), a2dpResampler->getMode(),
             a2dpResampler->hasPeerRate() ? "codec" : "assumed",
             (unsigned long)sil, (unsigned long)a2dpResampler->getMaxWriteGapMs(),
             (unsigned long)a2dpResampler->getMaxOutputMs(),
             (unsigned long)a2dpResampler->getBufShortWrites(),
             (unsigned long)a2dpResampler->getDroppedBytes());
  }
}

// ---- Periodic full diagnostic block ----------------------------------------
// Fires every 10 s. Everything here is loop() context, so String and
// Serial.printf are safe. If the BT link drops or the audio garbles, the
// deltas between two consecutive dumps tell you which side changed.
// Every line goes through a stack buffer + Serial.write. Serial.printf may
// malloc for output >64 bytes, and the diag lines are long.
inline void serviceDiagLog() {
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if (now - lastMs < 10000) return;
  lastMs = now;

  bool connected = a2dpSink->is_connected();
  char line[192];

  auto emit = [&](const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n > 0 && n < (int)sizeof(line)) Serial.write((const uint8_t*)line, n);
  };

  emit("\n");
  emit("========= [DIAG] uptime=%lus  heap=%u  loopTicks=%lu =========\n",
       (unsigned long)(now / 1000), (unsigned)ESP.getFreeHeap(),
       (unsigned long)diagLoopTicks);
  emit("  BT        : running=%d  connected=%d  mac=%s  vol=%u\n",
       a2dpRunning ? 1 : 0, connected ? 1 : 0,
       currentConnectedMac.length() ? currentConnectedMac.c_str() : "(none)",
       (unsigned)activeBtVolume);
  emit("  A2DP evt  : connecting=%lu connected=%lu disconn=%lu disconning=%lu\n",
       (unsigned long)diagA2dpConnecting,
       (unsigned long)diagA2dpConnected,
       (unsigned long)diagA2dpDisconnecting,
       (unsigned long)diagA2dpDisconnected);
  emit("  AVRCP     : ct=%d  volEvents=%lu  volApplied=%lu\n",
       avrcConnected ? 1 : 0,
       (unsigned long)diagVolumeEvents,
       (unsigned long)diagVolumeApplied);
  emit("  UART RX   : bytes=%lu lines=%lu overflow=%lu\n",
       (unsigned long)diagUartRxBytes,
       (unsigned long)diagUartRxLines,
       (unsigned long)diagUartRxOverflow);
  emit("  Heartbeat : last GET_BT_DEVICES %lu ms ago%s\n",
       (unsigned long)(now - lastHeartbeatMs),
       (lastHeartbeatMs == 0 || (now - lastHeartbeatMs) > 15000)
         ? "   <== SERVER SILENT?" : "");
  emit("  Source    : %s  (changed %lu times)\n",
       activeSource.c_str(), (unsigned long)diagSourceChanges);
  emit("===========================================================\n");
}

// ---- Volume fallback for devices with no AVRCP volume ----------------------
// TVs commonly send only echoes of the connect volume, never a real change.
// If no real event arrives within CONNECT_VOL_FALLBACK_MS, the device cannot
// control the sink's volume — go to unity so it is not stuck at 10%.
inline void serviceVolumeFallback() {
  if (volFallbackAtMs == 0) return;
  if ((int32_t)(millis() - volFallbackAtMs) < 0) return;   // not yet
  volFallbackAtMs = 0;
  if (volEventReceived) return;                            // real event won
  if (!a2dpRunning || !a2dpSink->is_connected()) return;

  // Computers stay at the connect-forced 10%: the PC user controls level
  // from the host (Windows mixer / volume keys) and the forced 127 is
  // exactly the "jumps to 100% a second or two after connect" complaint.
  // TVs (and anything not matched as a computer) still get the unity
  // bump so they are audible before the remote is touched.
  if (peerIsComputer) {
    logLine("No AVRCP volume in 3s - COMPUTER peer, keeping 10%");
    return;
  }

  activeBtVolume = 127;
  a2dpSink->set_volume(activeBtVolume);
  logLine("No AVRCP volume in 3s - falling back to 127/127 (unity)");
}

void setup() {
  Serial.begin(LOG_BAUD);
  delay(200);
  LOGI("=========================================================");
  LOGI("Enkelvoud Source ESP32 booting (fw %s)", EVD_FW_VERSION);
  LOGI("=========================================================");

  // Deferred heap construction - see EVDBTState.h for why these are
  // pointers and not globals.
  LOGI("Constructing audio pipeline objects...");
  dac1I2s       = new I2SStream();
  multiOutput   = new MultiOutput();
  // Chain: a2dpSink -> a2dpResampler -> multiOutput -> dac1I2s
  // The resampler is passed to the sink as its output stream and forwards
  // to multiOutput after rate conversion. 48 kHz is the S3 server's
  // SOURCE_SAMPLE_RATE; keeping the source at that rate avoids a resample
  // step on the server and lets it encode straight into Opus.
  a2dpResampler = new A2DPRateResampler(*dac1I2s, SAMPLE_RATE);
  a2dpSink      = new BluetoothA2DPSink(*a2dpResampler);
  LOGI("  dac1I2s=%p  multiOutput=%p  a2dpResampler=%p  a2dpSink=%p",
       dac1I2s, multiOutput, a2dpResampler, a2dpSink);

  if (!dac1I2s || !multiOutput || !a2dpResampler || !a2dpSink) {
    LOGE("Audio pipeline allocation FAILED - halting");
    while (true) { delay(1000); }
  }

  // HardwareSerial::begin returns void on the ESP32 core, so there is no
  // return value to check. If the UART link silently fails, it will show up
  // as `lastHeartbeatMs` staying 0 in serviceDiagLog()'s Heartbeat line.
  cmdUart.begin(UART_BAUD, SERIAL_8N1, PIN_CMD_RX, PIN_CMD_TX);
  LOGI("UART: RX=%d TX=%d @ %d baud (link to S3 server)",
       PIN_CMD_RX, PIN_CMD_TX, UART_BAUD);

  a2dpSink->set_on_connection_state_changed(connection_state_changed);
  a2dpSink->set_sample_rate_callback(sample_rate_changed);
  a2dpSink->set_on_volumechange(volume_changed_callback);
  a2dpSink->set_auto_reconnect(true);

  startBluetooth();
  initAvrcController();
  setupInputs();

  LOGI("=========================================================");
  LOGI("Setup complete. Default source: %s", activeSource.c_str());
  LOGI("Listening for SRC:/CMD:/GET_BT_DEVICES from server...");
  LOGI("=========================================================");

  // Steady state: only warnings and errors reach the serial monitor.
  // Full diagnostics live in the server dashboard's Source tab.
  gSerialLogLevel = 1;
}

void loop() {
  diagLoopTicks++;

  drainBtLogQueue();          // gated internally now (see below)
  handleDeferredBtEvents();
  serviceVolumeReassert();
  serviceVolumeFallback();
  serviceReconnectWatchdog();
  serviceBtStallWatchdog();   // [bt-stall-watchdog]

  serviceRateLog();
  // [restore-resampling] applyDetectedSourceRate() and
  // serviceRateChangeWatchdog() are dead in the resampling architecture:
  // I2S is fixed at SAMPLE_RATE (48000) and the resampler converts peer
  // rate -> that rate on every write. Any call that tears down and
  // re-begins the I2S stream at the peer rate would break the resampler's
  // output/DMA balance. Leave the function definitions in place (they are
  // harmless) but do NOT call them from loop().
  // applyDetectedSourceRate();
  // serviceRateChangeWatchdog();
  serviceAvrcKey();
  handleUartCommands();

  // Only run the full 10 s diagnostic dump when verbosity is verbose. The
  // counters it prints are still polled by the server via GET_BT_DEVICES
  // (or a future LOG| push), so nothing is lost by skipping the print.
  if (gSerialLogLevel >= 3) serviceDiagLog();

  otaServiceWatchdog();   // [ota-source-uart] abort stalled OTA transfers

  // [heap-watchdog]
  // Heap trend watchdog. The 60 s print alone cannot diagnose an OOM crash
  // because the crash reboots the board before the next print is due. This
  // version tracks a 10 s low-watermark, and emits a dedicated alarm line
  // the first time free heap crosses 40 KiB (WARN) or 20 KiB (ERR) within
  // a 60 s window. All output goes through Serial.write on a stack buffer;
  // no heap, no Serial.printf, safe alongside the BT stack.
  {
    static uint32_t lowInWindow   = 0xFFFFFFFF;
    static uint32_t lastReportMs  = 0;
    static bool     warnFired     = false;
    static bool     errFired      = false;

    uint32_t nowHeap = millis();
    uint32_t freeH   = (uint32_t)ESP.getFreeHeap();
    if (freeH < lowInWindow) lowInWindow = freeH;

    if (!errFired && freeH < 20000) {
      errFired = true;
      char hb[96];
      int  hn = snprintf(hb, sizeof(hb),
                         "[HEAP-ERR] %8lu ms | free=%u low10=%u | OOM IMMINENT\n",
                         (unsigned long)nowHeap, (unsigned)freeH,
                         (unsigned)lowInWindow);
      if (hn > 0 && hn < (int)sizeof(hb)) Serial.write((const uint8_t*)hb, hn);
    } else if (!warnFired && freeH < 40000) {
      warnFired = true;
      char hb[96];
      int  hn = snprintf(hb, sizeof(hb),
                         "[HEAP-WARN] %8lu ms | free=%u low10=%u\n",
                         (unsigned long)nowHeap, (unsigned)freeH,
                         (unsigned)lowInWindow);
      if (hn > 0 && hn < (int)sizeof(hb)) Serial.write((const uint8_t*)hb, hn);
    }

    // [heap-trend-respects-verbosity]
    // The 60 s HEAP trend line and the WARN/ERR alarms above also
    // unconditionally Serial.write even when gSerialLogLevel == 0. The
    // WARN/ERR alarms SHOULD always print (they are load-bearing
    // diagnostics for "silent crash" reports), but the routine 60 s
    // HEAP line should respect the level gate so an operator who set
    // level 0 gets a genuinely silent board. WARN/ERR above remain
    // unconditional.
    if ((nowHeap - lastReportMs > 60000) && gSerialLogLevel >= 2) {
      lastReportMs = nowHeap;
      char hb[128];
      int  hn = snprintf(hb, sizeof(hb),
                         "[HEAP ] %8lu ms | free=%u low10=%u minFree=%u maxBlock=%u\n",
                         (unsigned long)nowHeap, (unsigned)freeH,
                         (unsigned)lowInWindow,
                         (unsigned)ESP.getMinFreeHeap(),
                         (unsigned)ESP.getMaxAllocHeap());
      if (hn > 0 && hn < (int)sizeof(hb)) Serial.write((const uint8_t*)hb, hn);
      lowInWindow = freeH;
      warnFired   = false;
      errFired    = false;
    } else if (nowHeap - lastReportMs > 60000) {
      // Level 0/1: clear the low-water window so it stays a 60 s
      // rolling figure rather than growing into a whole-uptime minimum
      // when the report is suppressed.
      lastReportMs = nowHeap;
      lowInWindow  = freeH;
      warnFired    = false;
      errFired     = false;
    }
  }

  delay(5);
}

#endif // EVDBTMAIN_H