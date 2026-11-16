#pragma once

#ifndef EVDBTCONFIG_H
#define EVDBTCONFIG_H

// ============================================================================
//  EVDBTConfig.h - pins, compile-time constants, version, source IDs
// ============================================================================
#include <Arduino.h>

// ---- Logging ---------------------------------------------------------------
// Runtime-gated. 3=verbose(boot) 2=warn+error 1=error only 0=silent.
// setup() starts at 3 so the serial monitor gets a complete boot trace,
// then drops to 1. Toggle via UART command LOGLV:<0..3> from the server.
#define LOG_BAUD 115200
extern volatile uint8_t gSerialLogLevel;
#define LOGI(fmt, ...) do { if (gSerialLogLevel >= 3) \
    Serial.printf("[INFO ] %8lu ms | " fmt "\n", millis(), ##__VA_ARGS__); } while (0)
#define LOGW(fmt, ...) do { if (gSerialLogLevel >= 2) \
    Serial.printf("[WARN ] %8lu ms | " fmt "\n", millis(), ##__VA_ARGS__); } while (0)
#define LOGE(fmt, ...) do { if (gSerialLogLevel >= 1) \
    Serial.printf("[ERROR] %8lu ms | " fmt "\n", millis(), ##__VA_ARGS__); } while (0)

// ---- DAC 1 (feeds the ESP32-S3 server input) -------------------------------
static const int PIN_OUT1_LRC  = 25;
static const int PIN_OUT1_DOUT = 26;
static const int PIN_OUT1_BCLK = 27;

// ---- UART link to the S3 server --------------------------------------------
static const int PIN_CMD_RX = 32;
static const int PIN_CMD_TX = 33;
#define UART_BAUD 115200

// ---- AUX line-in (ADC) -----------------------------------------------------
// Classic ESP32. GPIO 1 is U0TXD and GPIO 2 is a boot-strapping pin on this
// chip; both are unusable for ADC line-in if we want to keep serial logging
// alive. GPIO 34/35 are input-only, ADC1_CH6/CH7, no strapping conflict, no
// UART conflict.
static const int PIN_AUX_L = 34;
static const int PIN_AUX_R = 35;

// ---- Audio format ----------------------------------------------------------
// I2S OUTPUT rate. BCLK = 48000 x 16 x 2 = 1.536 MHz.
//
// The A2DP codec callback supplies the input rate; until it arrives the
// adapter assumes 44.1 kHz and reports that assumption in RATEDIAG.
// Cubic interpolation converts the decoded stereo PCM to this fixed rate.
#define SAMPLE_RATE      48000
#define CHANNELS         2
#define BITS_PER_SAMPLE  16

// ---- Timings ---------------------------------------------------------------
#define CONNECT_VOL_GUARD_MS  4000   // ignore device AVRCP volume echo on connect
#define CONNECT_FORCED_VOLUME 13     // volume applied at connect (13/127 = 10%)
#define RECONNECT_IDLE_MS     8000   // reconnect watchdog: nudge after this idle
#define CONNECT_VOL_FALLBACK_MS 3000 // if no AVRCP vol by then, go to 127/127
#define PC_VOL_HOLD_MS        10000 

// ---- Source identifiers ----------------------------------------------------
static const char *SOURCE_BT  = "BT";
static const char *SOURCE_AUX = "AUX";
static const char *SOURCE_USB = "USB";

#endif // EVDBTCONFIG_H