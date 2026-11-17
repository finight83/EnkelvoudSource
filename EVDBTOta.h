#pragma once
#ifndef EVDBTOTA_H
#define EVDBTOTA_H
// ============================================================================
//  EVDBTOta.h - Source board OTA receiver over the UART link to the server
//
//  No network on this board. OTA arrives as hex-encoded chunks over the
//  same UART (pins 32/33) that already carries CMD:/SRC:/LOG:/SRCSTAT:.
//  Protocol is documented in the server's EVDOTA.h otaPushToSource().
// ============================================================================
#include "EVDBTHelpers.h"
#include "EVDBTResampler.h"
#include <Update.h>
#include <esp_ota_ops.h>

// ---- State -----------------------------------------------------------------
struct SourceOtaState {
  bool     receiving      = false;
  uint32_t totalSize      = 0;
  uint32_t bytesReceived  = 0;
  uint32_t lastActivityMs = 0;
  int      expectedSeq    = 0;
};
static SourceOtaState g_sourceOta;

// ---- CRC16-CCITT (XModem variant, poly 0x1021, init 0x0000) ----------------
static inline uint16_t otaCrc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++) {
      if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
      else              crc = (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// ---- Hex helpers ----------------------------------------------------------
static inline int otaHexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

static inline size_t otaHexDecode(const char *src, size_t srcLen,
                                  uint8_t *dst, size_t dstCap) {
  if ((srcLen & 1) != 0) return 0;
  size_t outLen = srcLen / 2;
  if (outLen > dstCap) return 0;
  for (size_t i = 0; i < outLen; i++) {
    int hi = otaHexNibble(src[i * 2]);
    int lo = otaHexNibble(src[i * 2 + 1]);
    if (hi < 0 || lo < 0) return 0;
    dst[i] = (uint8_t)((hi << 4) | lo);
  }
  return outLen;
}

// ---- Handlers --------------------------------------------------------------

// OTA_BEGIN|<version>|<size>|<md5_hex>
// args points to the text after the '|' following "OTA_BEGIN".
static inline bool otaHandleBegin(const char *args, char *errOut, size_t errCap) {
  if (g_sourceOta.receiving) {
    snprintf(errOut, errCap, "busy");
    return false;
  }

  const char *p1 = strchr(args, '|');
  if (!p1) { snprintf(errOut, errCap, "bad_args"); return false; }
  const char *p2 = strchr(p1 + 1, '|');
  if (!p2) { snprintf(errOut, errCap, "bad_args"); return false; }

  // [ota-begin-no-string] Was: String version = String(args).substring(0, p1-args);
  //                     String md5Str = String(p2 + 1);
  // Two heap allocations on the OTA begin path. The BT stack and the
  // audio pipeline are still live when this runs (audio is paused only
  // AFTER Update.begin succeeds, via setForceSilence), so a fragmented
  // heap can fail the String constructor and leave the OTA unstarted
  // with a misleading "bad_args" error. Stack-fixed buffers eliminate
  // the allocation. version is only used for the log line; md5Str is
  // the 32-char hex the Update library expects.
  char version[24];
  size_t versionLen = (size_t)(p1 - args);
  if (versionLen >= sizeof(version)) versionLen = sizeof(version) - 1;
  memcpy(version, args, versionLen);
  version[versionLen] = '\0';

  uint32_t totalSize = (uint32_t)strtoul(p1 + 1, nullptr, 10);

  char md5Str[33];
  {
    const char *src = p2 + 1;
    size_t n = strlen(src);
    if (n > 32) n = 32;
    memcpy(md5Str, src, n);
    md5Str[n] = '\0';
  }

  if (totalSize < 100 * 1024 || totalSize > 1900 * 1024) {
    snprintf(errOut, errCap, "size_%u_out_of_range", (unsigned)totalSize);
    return false;
  }
  if (strlen(md5Str) != 32) {
    snprintf(errOut, errCap, "bad_md5_len_%u", (unsigned)strlen(md5Str));
    return false;
  }
  for (size_t i = 0; i < 32; i++) {
    if (otaHexNibble(md5Str[i]) < 0) {
      snprintf(errOut, errCap, "bad_md5_char");
      return false;
    }
  }

  if (!Update.begin(totalSize)) {
    snprintf(errOut, errCap, "begin:%s", Update.errorString());
    return false;
  }
  if (!Update.setMD5(md5Str)) {
    Update.abort();
    snprintf(errOut, errCap, "setMD5");
    return false;
  }

  // Silence the resampler output (see [ota-force-silence] in
  // EVDBTResampler.h). If the resampler is missing (allocation failed at
  // boot), the OTA still proceeds — the source will just produce the
  // pre-existing glitch pattern during flash erases.
  if (a2dpResampler) a2dpResampler->setForceSilence(true);

  g_sourceOta.receiving      = true;
  g_sourceOta.totalSize      = totalSize;
  g_sourceOta.bytesReceived  = 0;
  g_sourceOta.expectedSeq    = 0;
  g_sourceOta.lastActivityMs = millis();

  logLinef("[OTA] Begin v%s size=%u md5=%s",
           version, (unsigned)totalSize, md5Str);
  return true;
}

// OTA_DATA|<seq>|<offset_hex>|<hex_payload>|<crc16_hex>
// Returns true on success, fills *outSeq with the acked seq.
// Returns false on error, fills *outSeq with the seq being NAK'd (or -1).
static inline bool otaHandleData(const char *args, char *errOut,
                                 size_t errCap, int *outSeq) {
  *outSeq = -1;
  if (!g_sourceOta.receiving) {
    snprintf(errOut, errCap, "not_receiving");
    return false;
  }

  const char *p1 = strchr(args, '|');
  if (!p1) { snprintf(errOut, errCap, "bad_args"); return false; }
  const char *p2 = strchr(p1 + 1, '|');
  if (!p2) { snprintf(errOut, errCap, "bad_args"); return false; }
  const char *p3 = strchr(p2 + 1, '|');
  if (!p3) { snprintf(errOut, errCap, "bad_args"); return false; }

  int seq = atoi(args);
  *outSeq = seq;

  const bool isDup = (g_sourceOta.expectedSeq > 0 &&
                      seq == g_sourceOta.expectedSeq - 1);
  if (!isDup && seq != g_sourceOta.expectedSeq) {
    snprintf(errOut, errCap, "seq_exp_%d_got_%d",
             g_sourceOta.expectedSeq, seq);
    return false;
  }

  uint32_t offset = (uint32_t)strtoul(p1 + 1, nullptr, 16);
  if (!isDup && offset != g_sourceOta.bytesReceived) {
    snprintf(errOut, errCap, "offset_exp_%u_got_%u",
             (unsigned)g_sourceOta.bytesReceived, (unsigned)offset);
    return false;
  }

  const char *hexStart = p2 + 1;
  size_t hexLen = (size_t)(p3 - hexStart);
  uint16_t expectedCrc = (uint16_t)strtoul(p3 + 1, nullptr, 16);

  static uint8_t decoded[512];
  size_t decodedLen = otaHexDecode(hexStart, hexLen, decoded, sizeof(decoded));
  if (decodedLen == 0) {
    snprintf(errOut, errCap, "hex_decode");
    return false;
  }

  uint16_t actualCrc = otaCrc16(decoded, decodedLen);
  if (actualCrc != expectedCrc) {
    snprintf(errOut, errCap, "crc_%04x_%04x", actualCrc, expectedCrc);
    return false;
  }

  // Idempotency: a well-formed duplicate of the last acked chunk is re-acked
  // without rewriting flash.
  if (isDup) return true;
  size_t written = Update.write(decoded, decodedLen);
  if (written != decodedLen) {
    snprintf(errOut, errCap, "write_%u_%u",
             (unsigned)written, (unsigned)decodedLen);
    Update.abort();
    if (a2dpResampler) a2dpResampler->setForceSilence(false);
    g_sourceOta.receiving = false;
    return false;
  }

  g_sourceOta.bytesReceived += decodedLen;
  g_sourceOta.expectedSeq++;
  g_sourceOta.lastActivityMs = millis();
  return true;
}

// OTA_END|<md5_hex>
static inline bool otaHandleEnd(const char *args, char *errOut, size_t errCap) {
  (void)args;  // md5 is verified internally by Update.end()
  if (!g_sourceOta.receiving) {
    snprintf(errOut, errCap, "not_receiving");
    return false;
  }

  if (g_sourceOta.bytesReceived != g_sourceOta.totalSize) {
    snprintf(errOut, errCap, "size_%u_%u",
             (unsigned)g_sourceOta.bytesReceived, (unsigned)g_sourceOta.totalSize);
    Update.abort();
    if (a2dpResampler) a2dpResampler->setForceSilence(false);
    g_sourceOta.receiving = false;
    return false;
  }

  if (!Update.end(true)) {
    snprintf(errOut, errCap, "end:%s", Update.errorString());
    Update.abort();
    if (a2dpResampler) a2dpResampler->setForceSilence(false);
    g_sourceOta.receiving = false;
    return false;
  }

  g_sourceOta.receiving = false;
  logLine("[OTA] Commit OK; rebooting");
  return true;
}

// ---- Watchdog: called from loop() -----------------------------------------
// If a chunk takes longer than 30 s to arrive, or the transfer stalls
// mid-stream, abort cleanly so the boot pointer stays on the current
// (old) firmware rather than leaving a half-written inactive partition
// that never gets committed.
static inline void otaServiceWatchdog() {
  if (!g_sourceOta.receiving) return;
  if ((millis() - g_sourceOta.lastActivityMs) < 30000) return;

  logLine("[OTA] Timeout (30 s no activity); aborting");
  Update.abort();
  if (a2dpResampler) a2dpResampler->setForceSilence(false);
  g_sourceOta.receiving = false;
}

#endif // EVDBTOTA_H