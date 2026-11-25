#ifndef EVDBTSTATE_H
#define EVDBTSTATE_H
// ============================================================================
//  EVDBTState.h - mutable globals: audio objects, BT state, source state
// ============================================================================
#include "EVDBTConfig.h"

#include <Arduino.h>
#include <AudioTools.h>
#include <BluetoothA2DPSink.h>
#include <Preferences.h>
#include <HardwareSerial.h>
#include <esp_bt_defs.h>

using namespace audio_tools;

// ---- Audio pipeline objects ------------------------------------------------
// Deliberately pointers, constructed on the heap in setup() rather than as
// globals. Constructing these as statics runs their constructors during the
// C++ startup phase (_libc_init_array), before app_main()/setup() has run and
// before the serial port is up. Any fault or hang in those constructors
// produces a silent stop right after the bootloader hands off - no panic
// output, no reset log, nothing. Deferring to setup() sidesteps the whole
// class of problem and gives us a serial line we can actually print errors
// to if construction fails.
I2SStream      *dac1I2s    = nullptr;
MultiOutput    *multiOutput = nullptr;
BluetoothA2DPSink *a2dpSink = nullptr;

// A2DPRateResampler sits between a2dpSink and multiOutput. Full class
// definition lives in EVDBTResampler.h; forward-declared here so this
// header stays free of Stream/Print includes and the pointer is visible
// wherever state is queried (e.g. serviceRateLog in EVDBTMain.h,
// handleDeviceDisconnected in EVDBTBluetooth.h).
class A2DPRateResampler;
A2DPRateResampler *a2dpResampler = nullptr;

// [tone-stream] The connect/disconnect tone needs its OWN route into dac1I2s.
// It cannot write to the I2SStream directly: the A2DP writer is already
// pushing frames into it, I2SStream is not thread-safe, and a concurrent
// write corrupts the DMA ring - which is why the old playTone() was disabled
// in EVDBTBluetooth.h. Instead a second, quieter output is added to
// multiOutput, so the tone goes through the SAME write() call as the audio
// and can never interleave with it. VolumeStream scales the tone without
// touching the audio path: the tone is played over a 50 W amp and anything
// near full scale is far too loud.
VolumeStream  *toneVolume = nullptr;

// ---- Infrastructure --------------------------------------------------------
HardwareSerial cmdUart(2);
Preferences    preferences;

// ---- Runtime state ---------------------------------------------------------
bool   a2dpRunning = false;
String currentConnectedMac = "";

// ---- Per-device volume -----------------------------------------------------
// Written from BT task (volume_changed_callback) and loop (handleDeviceConnected,
// serviceVolumeReassert, sendBtDeviceList). volatile -> loop reloads each read.
volatile uint8_t activeBtVolume = 13;

// ---- Connection-guard bookkeeping ------------------------------------------
// lastConnectMs: written by loop (handleDeviceConnected), read by BT task
// (volume_changed_callback). volatile for cross-task visibility.
// reassertAtMs: loop-context only.
volatile unsigned long lastConnectMs = 0;
unsigned long          reassertAtMs  = 0;
// Fallback: if no real AVRCP volume event arrives after connect, the device
// likely cannot send one (common on TVs). Go to unity gain so the device's
// own volume control is not fighting a 10% attenuation it cannot change.
volatile bool          volEventReceived = false;
volatile unsigned long volFallbackAtMs  = 0;   // 0 = disabled

// ---- Deferred Bluetooth events ---------------------------------------------
// pendingDeviceListSend removed - never set true anywhere; sendBtDeviceList()
// is called directly from handleDeviceConnected / handleDeviceDisconnected.
volatile bool pendingConnectEvent    = false;
volatile bool pendingDisconnectEvent = false;
esp_bd_addr_t pendingPeerAddr;
volatile bool pendingPeerAddrValid   = false;
// True when the last CONNECTED peer was classified as a computer.
// Written by BT task in connection_state_changed() before publishing
// pendingConnectEvent; read by loop() in serviceVolumeFallback().
volatile bool peerIsComputer = false;

// ---- AVRCP controller (ESP-IDF v5.x API) -----------------------------------
// avrcConnected: written by BT task (avrcCtCallback), read by loop
// (requestAvrcKey, sendBtDeviceList). volatile for cross-task visibility.
// Everything else is loop-context only.
volatile bool    avrcConnected    = false;
uint8_t          avrcTl           = 0;
uint8_t          avrcPendingKey   = 0;
uint8_t          avrcPendingState = 0;
uint32_t         avrcPressAtMs    = 0;

// ---- Source selection ------------------------------------------------------
String activeSource = SOURCE_BT;

// ---- I2S output rate tracking (passthrough architecture) ------------------
// [passthrough-arch]
// The source board no longer resamples. BT PCM flows through the
// A2DPRateResampler (now a transparent rate detector) at its native rate,
// and the I2S output clock must match that rate or the audio plays at the
// wrong pitch. g_currentI2sRateHz tracks what the I2S hardware is
// currently configured for; applyDetectedSourceRate() in EVDBTMain.h
// updates it when the rate detector locks onto a different rate.
//
// 44100 as initial: Windows / iPhone / most Android SBC peers negotiate
// 44.1 kHz, so starting there gives correct pitch from frame 1 for the
// overwhelming majority of connects. Only 48 kHz Android peers hit the
// 800 ms detection window at wrong pitch — silently, since BT handshake
// finishes before the user starts playback.
// [i2s-rate-honest-init] Was 44100, a leftover from the passthrough
// architecture. startBluetooth() now configures I2S at SAMPLE_RATE (48000),
// and applyDetectedSourceRate() is no longer called, so this variable is
// never updated from boot onward. Initialising it to the rate the hardware
// actually runs at means any remaining reader of this variable (there are
// none after Edit 1/2 that matter, but it is exposed via diagnostics) sees
// a truthful value even if a future patch adds a reader by mistake.
volatile uint32_t g_currentI2sRateHz        = SAMPLE_RATE;
volatile bool     g_i2sRateChangeInProgress = false;

// ---- Runtime-gated serial logging ------------------------------------------
volatile uint8_t gSerialLogLevel = 3;   // verbose during boot; dropped at end of setup()
// ============================================================================
//  Diagnostics
// ============================================================================
// All of these are volatile because some are touched from the BT stack task
// via btLogPush()/connection_state_changed(). They are read only from loop().
volatile uint32_t diagA2dpConnecting    = 0;
volatile uint32_t diagA2dpConnected     = 0;
volatile uint32_t diagA2dpDisconnecting = 0;
volatile uint32_t diagA2dpDisconnected  = 0;

volatile uint32_t diagVolumeEvents  = 0;   // every callback, incl. suppressed
volatile uint32_t diagVolumeApplied = 0;   // only those that changed activeBtVolume

volatile uint32_t diagUartRxBytes    = 0;
volatile uint32_t diagUartRxLines    = 0;
volatile uint32_t diagUartRxOverflow = 0;

volatile uint32_t diagSourceChanges  = 0;

// Last time we saw a GET_BT_DEVICES (the server's 3 s heartbeat). Used by
// serviceDiagLog() to tell us if the S3 side has gone silent.
volatile uint32_t lastHeartbeatMs    = 0;

// Count of loop() iterations (cheap heartbeat to prove the main loop is
// still spinning even if the BT link is stuck).
volatile uint32_t diagLoopTicks = 0;

#endif // EVDBTSTATE_H