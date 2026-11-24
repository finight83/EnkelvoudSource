#pragma once

#ifndef EVDBTINPUTS_H
#define EVDBTINPUTS_H

// ============================================================================
//  EVDBTInputs.h - source selection + AUX / USB bring-up
//
//  Audio routing is intentionally stubbed: the A2DP path currently owns
//  multiOutput at all times. When AUX is wired in for real, it should mute the
//  A2DP stream and feed multiOutput itself inside setActiveSource.
//
//  USB is being implemented on the ESP32-S3 server instead of here; this file
//  keeps SOURCE_USB as a recognised identifier so the UART command handler
//  still logs it, but selecting USB does not change anything on this side.
// ============================================================================
#include "EVDBTHelpers.h"

// [b1-aux-not-implemented] Tell the server why a switch was refused. The server
// logs every SRCSTAT| line it receives, so "I picked AUX and nothing happened"
// becomes a visible rejection instead of silence. EVDBTUart.h owns the port and
// is included AFTER this file (see EnkelvoudSource.ino), so the definition
// lives here and works with cmdUart directly.
inline void uartReportSourceReject(const char *why) {
  cmdUart.print("SRCSTAT|rejected|");
  cmdUart.println(why ? why : "unsupported");
}

// ---- Source switch ---------------------------------------------------------
// Called from the UART command handler when the S3 server pushes "SRC:<name>".
// Startup default stays SOURCE_BT (set in EVDBTState.h); nothing else resets it.
inline void setActiveSource(const String &src) {
  String s = src;
  s.trim();
  s.toUpperCase();

  if (s.length() == 0) {
    logLine("setActiveSource: empty name, ignored");
    return;
  }
  if (s.length() > 8 || !(s == SOURCE_BT || s == SOURCE_AUX || s == SOURCE_USB)) {
    logLine("setActiveSource: unknown source rejected");
    return;
  }
  if (s == activeSource) {
    logLine("setActiveSource: already " + s + ", ignored");
    return;
  }

  String prev = activeSource;
  activeSource = s;
  diagSourceChanges++;

  // Loud banner so a source change from the server is unmistakable in the
  // source's serial log. If you change source on the dashboard and this line
  // does NOT appear here, the UART link or the server-side bridge is the
  // problem, not the source.
  Serial.printf("\n");
  logLine("*******************************************************");
  logLine("*** SOURCE CHANGED: " + prev + " -> " + s + " ***");
  logLine("*******************************************************");

  if (s == SOURCE_BT) {
    logLine("  routing: BT (A2DP sink -> multiOutput, live)");
  } else if (s == SOURCE_AUX) {
    // [b1-aux-not-implemented]
    // AUX used to be accepted here and then silently left A2DP owning
    // multiOutput — the operator selected AUX on the dashboard and got
    // whatever was on Bluetooth, or silence, with no error anywhere. Say so
    // and refuse the switch, so the source stays on the input it is actually
    // playing and the server's log shows why.
    activeSource = prev;
    logLine("  routing: AUX NOT IMPLEMENTED - source left at " + prev);
    uartReportSourceReject("AUX not implemented on this board");
    return;
  } else if (s == SOURCE_USB) {
    // USB audio is the SERVER's own UAC device. The server never needs to ask
    // this board to change anything, so a SRC:USB arriving here means the two
    // ends disagree about who owns the source. Refuse it and stay put.
    activeSource = prev;
    logLine("  routing: USB is a SERVER-side input - source left at " + prev);
    uartReportSourceReject("USB is a server-side input");
    return;
  } else {
    logLine("  routing: UNRECOGNISED source name '" + s + "'");
  }

  // TODO: hook actual AUX routing here:
  //   AUX -> mute A2DP, read PIN_AUX_L/R, feed multiOutput
}

// ---- AUX line-in -----------------------------------------------------------
inline void setupAuxInput() {
  pinMode(PIN_AUX_L, INPUT);
  pinMode(PIN_AUX_R, INPUT);
  // analogSetPinAttenuation(PIN_AUX_L, ADC_11db);
  // analogSetPinAttenuation(PIN_AUX_R, ADC_11db);
  LOGI("AUX line-in ready on L=%d R=%d (ADC, 12-bit)", PIN_AUX_L, PIN_AUX_R);
}

// ---- USB (placeholder) -----------------------------------------------------
// The USB audio implementation lives on the ESP32-S3 server. Nothing to do
// here; this hook exists only so setupInputs() reads uniformly.
inline void setupUsbInput() {
  LOGI("USB input: handled on the server side (no-op here)");
}

// ---- Bring-up --------------------------------------------------------------
inline void setupInputs() {
  setupAuxInput();
  setupUsbInput();
  LOGI("Active source at boot: %s", activeSource.c_str());
}

#endif // EVDBTINPUTS_H