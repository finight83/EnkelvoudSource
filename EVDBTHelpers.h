#pragma once

#ifndef EVDBTHELPERS_H
#define EVDBTHELPERS_H
#include <stdarg.h>
#include <string.h>    // strlen / strncmp / memcpy
#include <strings.h>   // strcasecmp / strncasecmp (POSIX)
#include <stdlib.h>    // atoi

// ============================================================================
//  EVDBTHelpers.h - logLine, MAC helpers, NVS key helper
// ============================================================================
#include "EVDBTState.h"

// ---- Logging ---------------------------------------------------------------
// Fixed stack buffer - no heap. Callers that already have a String still
// allocate at their call site, so migrate those to logLinef() over time.
inline void logLine(const char *message) {
  char buf[192];
  snprintf(buf, sizeof(buf), "[%10lu] %s\n", millis(), message);
  size_t n = strlen(buf);
  Serial.write((const uint8_t*)buf, n);
  // [logline-no-silent-drop]
  // Do NOT gate on availableForWrite(). HardwareSerial's TX FIFO is
  // only 128 bytes, so availableForWrite() == 128 - used; a log line
  // longer than ~124 bytes is silently discarded even on an idle
  // UART. Loop() calls logLine at most a few times per second; the
  // worst-case blocking on a full FIFO is ~10 ms for a 90-byte line
  // at 115200 baud, which is far below the audio DMA ring. Blocking
  // is correct here; silent drops lose exactly the diagnostics the
  // S3 dashboard's log tail exists to surface.
  cmdUart.write((const uint8_t*)"LOG|", 4);
  cmdUart.write((const uint8_t*)buf, n);
}

// Compatibility overload for existing call sites that pass String.
inline void logLine(const String &message) {
  logLine(message.c_str());
}

// Formatted variant - builds directly into a stack buffer, so the call site
// allocates nothing. Preferred form for any dynamic log message.
inline void logLinef(const char *fmt, ...) {
  char msg[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(msg, sizeof(msg), fmt, ap);
  va_end(ap);
  logLine(msg);
}

// ---- MAC helpers -----------------------------------------------------------
inline String macToString(const uint8_t *bda) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
  return String(buf);
}

inline bool stringToMac(const String &str, esp_bd_addr_t bda) {
  int values[6];
  if (sscanf(str.c_str(), "%x:%x:%x:%x:%x:%x",
             &values[0], &values[1], &values[2],
             &values[3], &values[4], &values[5]) == 6) {
    for (int i = 0; i < 6; ++i) bda[i] = (uint8_t)values[i];
    return true;
  }
  return false;
}

// ---- Computer (PC/laptop) detection ---------------------------------------
// Windows PCs and TVs are both silent at connect: neither pushes an AVRCP
// absolute-volume event, so the behaviourally-observed state is identical.
// We still need to tell them apart because:
//   - PCs:  must stay at the forced 10% (user's own Windows volume is the
//           control surface; pushing sink to 127 is what caused the "jumps
//           to 100% a couple of seconds after connect" complaint).
//   - TVs:  need the 127 bump so they're audible before the user touches
//           the remote.
// Only reliable signal is the peer address. Match full MAC first (more
// specific), then a curated list of dedicated PC/laptop OUIs. Anything
// else falls through to the fallback (TV behaviour) — safer default.
//
// To add a new PC: connect it once, read the "BT Connected: xx:xx:..." line,
// then add either the full 6-byte MAC (preferred when the OUI is shared
// with non-PC products) or just the 3-byte OUI (preferred for dedicated
// PC vendors like Intel/Dell/Lenovo/Asus/MSI).
static const uint8_t kComputerMACs[][6] = {
    // User's current PC. Its OUI (00:18:E4) is not a dedicated PC vendor,
    // so match the exact device address instead of the whole OUI.
    {0x00,0x18,0xE4,0x0A,0x3F,0x0C},
};

static const uint8_t kComputerOUIs[][3] = {
    // Intel (integrated BT / wireless — desktop & laptop only)
    {0x00,0x1B,0x21}, {0x00,0x1E,0x64}, {0x00,0x1E,0x67}, {0x00,0x21,0x5C},
    {0x00,0x21,0x5D}, {0x00,0x21,0x6A}, {0x00,0x21,0x6B}, {0x00,0x22,0xFA},
    {0x00,0x22,0xFB}, {0x00,0x23,0x14}, {0x00,0x23,0x15}, {0x00,0x24,0xD7},
    {0x00,0x27,0x10}, {0xA0,0x36,0x9F}, {0xA4,0xBB,0x6D}, {0xE4,0xB3,0x18},
    // Dell
    {0x00,0x14,0x22}, {0x00,0x18,0x8B}, {0x00,0x1C,0x23}, {0x00,0x1E,0xC9},
    {0x00,0x21,0x70}, {0x00,0x21,0x9B}, {0x00,0x23,0xAE}, {0x00,0x24,0xE8},
    {0x00,0x26,0xB9}, {0xB8,0x85,0x84}, {0xF0,0x4D,0xA2}, {0xF8,0xB1,0x56},
    // Lenovo / IBM
    {0x00,0x18,0x7D}, {0x00,0x1A,0x64}, {0x00,0x1C,0x25}, {0x00,0x21,0xCC},
    {0x20,0x16,0xD8}, {0x28,0xD2,0x44}, {0xE4,0x54,0xE8},
    // Asus
    {0x00,0x18,0xF3}, {0x04,0xD9,0xF5}, {0x2C,0xFD,0xA1}, {0xF8,0x32,0xE4},
    // MSI
    {0x00,0x15,0x58}, {0x00,0x1D,0x72}, {0x00,0x21,0x85},
};

inline bool isComputerMac(const uint8_t *bda) {
  if (bda == nullptr) return false;
  for (size_t i = 0; i < sizeof(kComputerMACs) / sizeof(kComputerMACs[0]); i++) {
    if (memcmp(bda, kComputerMACs[i], 6) == 0) return true;
  }
  for (size_t i = 0; i < sizeof(kComputerOUIs) / sizeof(kComputerOUIs[0]); i++) {
    if (bda[0] == kComputerOUIs[i][0] &&
        bda[1] == kComputerOUIs[i][1] &&
        bda[2] == kComputerOUIs[i][2]) return true;
  }
  return false;
}

// (NVS helpers removed: getNvsKey was never called; per-device NVS keys are
// not used in this firmware.)

#endif // EVDBTHELPERS_H