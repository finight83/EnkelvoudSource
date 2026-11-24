#ifndef EVDBTUART_H
#define EVDBTUART_H
// ============================================================================
//  EVDBTUart.h - command parser for the S3 server link
//
//  Inbound (S3 -> source):
//    GET_BT_DEVICES  -> reply with the BT_JSON| line
//    KICK_BT         -> disconnect the current A2DP peer
//    REMOVE_BT <mac> -> forget a paired device
//    CMD:next        -> AVRCP passthrough FORWARD
//    CMD:prev        -> AVRCP passthrough BACKWARD
//    SRC:<name>      -> switch active source (BT / AUX / USB)
//
//  Every inbound line is logged verbatim, and every dispatch is logged with
//  an arrow so the source-side monitor shows exactly what the server asked
//  for and what the source did about it.
//
//  Note: the server's sendReceiverCommand() prefixes "CMD:" on everything it
//  sends. So GET_BT_DEVICES actually arrives as "CMD:GET_BT_DEVICES" and the
//  same is true for KICK_BT / REMOVE_BT. To stay robust against both
//  spellings this parser accepts the bare form AND the "CMD:"-prefixed form.
// ============================================================================
#include "EVDBTHelpers.h"
#include "EVDBTBluetooth.h"
#include "EVDBTInputs.h"
#include "EVDBTOta.h"  

// Fixed-size receive buffer; no String, no realloc, no per-command heap.
// A maximum command is REMOVE_BT + 17-char MAC + "CMD:" prefix ~= 30 bytes;
// 128 is generous headroom. Anything longer is dropped as overflow.
inline void handleUartCommands() {
  // [ota-uart-buffer] Was 130. The largest OTA_DATA line is
  //   "OTA_DATA|" (9) + seq up to 10 digits + "|" + 8 hex offset + "|"
  //   + 1024 hex chars + "|" + 4 hex crc + "\n" ≈ 1070 bytes.
  // 1200 gives ~130 B of slack for future protocol additions.
  static char   line[1200];
  static size_t lineLen = 0;

  while (cmdUart.available()) {
    char c = (char)cmdUart.read();
    diagUartRxBytes++;

    if (c == '\n' || c == '\r') {
      if (lineLen == 0) continue;
      line[lineLen] = '\0';
      // trim trailing whitespace in place
      while (lineLen > 0 &&
             (line[lineLen - 1] == ' ' || line[lineLen - 1] == '\t')) {
        line[--lineLen] = '\0';
      }
      if (lineLen == 0) continue;
      diagUartRxLines++;

      // [uart-rx-log-level] Log the raw line only at verbose level.
      // The S3 heartbeat (CMD:GET_BT_DEVICES) arrives every 5 s and
      // used to generate one LOG| line back to the S3 for every
      // heartbeat, filling the 16-entry log ring with noise and
      // pushing real warnings out within ~15 s. Individual commands
      // are still logged by the dispatch path at level 2 ("BT device
      // list sent", "AVRCP NEXT" etc.), so nothing operationally
      // important is lost by silence at level 1.
      if (gSerialLogLevel >= 3) {
        // [ota-uart-no-flood] OTA_DATA lines are ~1100 bytes and would
        // flood both the serial monitor and the S3 log ring (each echo
        // is a LOG| line ~1100 bytes). Log only the seq, not the payload.
        if (strncmp(line, "OTA_DATA|", 9) == 0) {
          const char *s = line + 9;
          const char *sep = strchr(s, '|');
          char shortLog[64];
          snprintf(shortLog, sizeof(shortLog), "UART RX: [OTA_DATA|%.20s]",
                   s);
          logLinef("%s", shortLog);
          (void)sep;
        } else {
          logLinef("UART RX: [%s]", line);
        }
      }

      // Case-insensitive compare of the whole line against a literal.
      auto eq = [](const char *a, const char *b) {
        return strcasecmp(a, b) == 0;
      };
      auto starts = [](const char *s, const char *prefix) {
        return strncmp(s, prefix, strlen(prefix)) == 0;
      };

      // [ota-source-uart] OTA commands. Recognized before the general
      // dispatch chain because OTA_DATA lines do not match any of the
      // existing command patterns. The handlers live in EVDBTOta.h.
      if (starts(line, "OTA_BEGIN|")) {
        char err[64];
        if (otaHandleBegin(line + 10, err, sizeof(err))) {
          cmdUart.println("OTA_READY");
        } else {
          cmdUart.print("OTA_ERR|");
          cmdUart.println(err);
        }
      } else if (starts(line, "OTA_DATA|")) {
        char err[64];
        int seq = -1;
        if (otaHandleData(line + 9, err, sizeof(err), &seq)) {
          cmdUart.print("OTA_ACK|");
          cmdUart.println(seq);
        } else {
          cmdUart.print("OTA_NAK|");
          cmdUart.print(seq);
          cmdUart.print("|");
          cmdUart.println(err);
        }
      } else if (starts(line, "OTA_END|")) {
        char err[64];
        if (otaHandleEnd(line + 8, err, sizeof(err))) {
          cmdUart.println("OTA_OK");
          delay(500);
          ESP.restart();
        } else {
          cmdUart.print("OTA_FAIL|");
          cmdUart.println(err);
        }
      } else if (eq(line, "GET_BT_DEVICES") || eq(line, "CMD:GET_BT_DEVICES")) {
        lastHeartbeatMs = millis();
        // [dispatch-log-verbose-only] Same flood source as above — one
        // "-> sendBtDeviceList()" line per 5 s heartbeat, 12/min.
        // Gate to verbose alongside the "BT device list sent" line in
        // sendBtDeviceList().
        if (gSerialLogLevel >= 3) {
          logLine("  -> sendBtDeviceList()");
        }
        sendBtDeviceList();
      } else if (eq(line, "KICK_BT") || eq(line, "CMD:KICK_BT")) {
        logLine("  -> a2dpSink->disconnect()");
        a2dpSink->disconnect();
      } else if (starts(line, "REMOVE_BT ") ||
                 starts(line, "CMD:REMOVE_BT ")) {
        const char *off = starts(line, "CMD:") ? line + 14 : line + 10;
        logLinef("  -> removeBtDevice(%s)", off);
        removeBtDevice(off);   // const char* signature, no allocation
      } else if (eq(line, "CMD:next")) {
        logLine("  -> AVRCP NEXT (0x4B)");
        requestAvrcKey(0x4B);
      } else if (eq(line, "CMD:prev")) {
        logLine("  -> AVRCP PREV (0x4C)");
        requestAvrcKey(0x4C);
      } else if (starts(line, "SRC:")) {
        logLinef("  -> setActiveSource(%.8s)", line + 4);
        setActiveSource(String(line + 4));   // conversion happens once, rare
      } else if (starts(line, "LOGLV:") || starts(line, "CMD:LOGLV:")) {
        // "LOGLV:"      = 6 chars  -> digit starts at +6
        // "CMD:LOGLV:"  = 10 chars -> digit starts at +10
        const char *v = starts(line, "CMD:LOGLV:") ? line + 10 : line + 6;
        int lv = atoi(v);
        if (lv < 0) lv = 0;
        if (lv > 3) lv = 3;
        gSerialLogLevel = (uint8_t)lv;
        char lb[48];
        int  n = snprintf(lb, sizeof(lb), "[%10lu] LOGLV -> %d\n",
                          millis(), (int)gSerialLogLevel);
        if (n > 0 && n < (int)sizeof(lb)) Serial.write((const uint8_t*)lb, n);
      } else {
        logLine("  -> UNHANDLED command");
      }
      lineLen = 0;
      continue;
    }

    // [uart-overflow-discard-until-newline]
    // Previous behaviour reset lineLen to 0 and continued reading, so
    // the TAIL of a runaway line became the "new" line and was dispatched
    // on the next '\n'. With a >1199-byte corrupted frame, the tail is
    // arbitrary and could (e.g.) match "CMD:next" if the corruption
    // happened to align that way. Discarding until the next line terminator
    // eliminates the class entirely. OTA_DATA is currently 1059 B max,
    // so this branch only fires on true corruption today, but the fix
    // costs nothing and removes the hazard for future protocol growth.
    static bool discardUntilNewline = false;
    if (discardUntilNewline) {
      if (c == '\n' || c == '\r') discardUntilNewline = false;
      continue;
    }
    if (lineLen >= sizeof(line) - 1) {
      // [b2-ota-nak-overflow]
      // This used to just drop the bytes and log locally. The server, waiting
      // for OTA_ACK|seq, saw nothing at all and retried the SAME chunk until
      // its own timeout — three full re-uploads of the image per failed
      // update. NAK with the sequence we were waiting for, so the server's
      // existing NAK path resends a chunk that fits and the update continues
      // from where it was instead of restarting.
      lineLen            = 0;
      discardUntilNewline = true;
      diagUartRxOverflow++;
      cmdUart.print("OTA_NAK|");
      cmdUart.print((long)g_sourceOta.expectedSeq);
      cmdUart.println("|line_overflow");
      logLine("UART RX overflow - NAKed, discarding until newline");
      continue;
    }
    line[lineLen++] = c;
  }
}

#endif // EVDBTUART_H