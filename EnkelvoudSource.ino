#include <Arduino.h>

#define EVD_FW_VERSION "2.1.0.1" // 

// ---- Static-init probe -----------------------------------------------------
// Runs before setup() and before any of the audio pipeline globals are
// constructed. If this line does not print on the serial monitor, the chip
// is faulting during C++ static initialisation, which happens before
// app_main()/setup() gets control and therefore produces no panic output.
// Once the pipeline is heap-constructed (see EVDBTState.h / EVDBTMain.h),
// this should always print and can be left in place as a cheap boot marker.
// Lightweight static-init marker: writes one byte to the UART TX FIFO directly
// (no HardwareSerial object access, no Arduino constructors). This is the
// earliest possible evidence that the app reached static init. If it appears
// but normal boot output does not, the fault is in a later static constructor.
struct BootProbe {
  BootProbe() {
    // UART0 TX FIFO register on ESP32 (0x3FF40000 + 0x0). Push one '.' then
    // give the peripheral a moment to shift it out. This bypasses the
    // HardwareSerial global whose constructor may not have run yet.
    const uint32_t UART0_FIFO = 0x3FF40000;
    *(volatile uint32_t*)UART0_FIFO = '.';
    for (volatile int i = 0; i < 1000000; ++i) { /* delay for FIFO drain */ }
  }
};
BootProbe bootProbeInstance;

/*
  ============================================================================
  EnkelvoudSource (ESP32) - Bluetooth / AUX / USB -> single I2S DAC -> S3 server

  BT   - default at boot; A2DP sink feeds multiOutput
  AUX  - ADC line-in on PIN_AUX_L / PIN_AUX_R (audio path is a stub for now)
  USB  - placeholder for TinyUSB UAC host (audio path is a stub for now)

  Layout:
    EVDBTConfig.h     - pins, compile-time constants, version, source IDs
    EVDBTState.h      - mutable globals (audio objects, BT state, source state)
    EVDBTHelpers.h    - logLine, MAC helpers, NVS key helper
    EVDBTBluetooth.h  - A2DP sink, device mgmt, AVRCP passthrough, callbacks
    EVDBTInputs.h     - source selection + AUX / USB bring-up
    EVDBTUart.h       - command parser for the S3 server link (SRC:, CMD:, ...)
    EVDBTMain.h       - setup(), loop(), deferred-event plumbing

  Source selection:
    BT   - default at boot; A2DP sink feeds multiOutput
    AUX  - ADC line-in on PIN_AUX_L / PIN_AUX_R (audio path is a stub for now)
    USB  - placeholder for TinyUSB UAC host (audio path is a stub for now)

  The S3 server pushes source changes over the same UART link via "SRC:<name>\n".
  ============================================================================
*/

#include "EVDBTConfig.h"
#include "EVDBTState.h"
#include "EVDBTHelpers.h"
#include "EVDBTResampler.h"
#include "EVDBTOta.h"        // [ota-source-uart] must precede EVDBTUart.h
#include "EVDBTBluetooth.h"
#include "EVDBTInputs.h"
#include "EVDBTUart.h"
#include "EVDBTMain.h"