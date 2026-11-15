// File: EVDBTResampler.h
// Full replacement. Save as-is to overwrite the existing file.
// Fix summary:
//   * Codec-switch confirmation: 2 consecutive tight windows required
//     before accepting a new snap. Prevents bursty 44.1 kHz measurements
//     from being misread as a real 32 kHz codec switch.
//   * Ratio uses the snapped INTEGER (_srcRate), not the EMA (_srcRateHz).
//   * Diagnostics (getMode / getDetectedInputRate) report the snapped int,
//     so the log matches what the resampler actually does.
#pragma once
#ifndef EVDBTRESAMPLER_H
#define EVDBTRESAMPLER_H

#include <Arduino.h>
#include <Stream.h>
#include <string.h>

// ============================================================================
//  EVDBTResampler.h - A2DP PCM rate adapter
//
//  BluetoothA2DPSink does NOT resample. It forwards the codec's decoded PCM
//  rate verbatim to whatever Stream was passed in the constructor. On the
//  Stream-output path, that rate is set by the SBC/AAC negotiation:
//    - SBC: 44.1 kHz for virtually every Windows / iPhone / Android peer
//    - SBC: 48 kHz on some Android phones (rare in practice)
//    - AAC: matches the source's AAC rate (44.1 or 48 kHz)
//  If we clock output I2S at 48 kHz while the sink feeds 44.1 kHz, the I2S
//  DMA underruns ~8.8 % of every second -> periodic silent gaps -> audible
//  stutter at every S3 node downstream.
//
//  This adapter sits between the sink and I2S and does four things:
//    1. Detects the peer's actual PCM rate from write throughput over an
//       800 ms window at the start of each connection. Bursty Windows
//       delivery measures ~38 900 Hz on a true 44.1 kHz SBC peer; the
//       wide snapRate() window below maps that back to 44 100 (the
//       nearest candidate), NOT 48 000, so the ratio is correct.
//    2. Snaps the measurement to the NEAREST of
//       {96000, 88200, 48000, 44100, 32000, 24000, 22050, 16000, 8000}
//       within +/-40 %. This is deliberately wide: it is the specific
//       property that recovers the correct 44 100 on a bursty PC.
//    3. After lock, the SNAPPED integer rate drives the ratio; a 500 ms
//       post-lock tracker only looks for a genuine codec switch. A switch
//       requires TWO CONSECUTIVE windows that (a) snap to the SAME new
//       rate and (b) each measure within 2 % of that snap. A bursty
//       transient never satisfies both conditions, so it cannot flip the
//       ratio into a self-locking spiral.
//    4. Cubic Catmull-Rom interpolation to the output rate whenever the
//       snapped integer differs from it.
//
//  Called from the Bluetooth A2DP task (write) and from loop() (reset,
//  diagnostics). reset() only touches single-word state (int/bool/double
//  atomic on ESP32), so a concurrent BT-task read sees either the old or
//  the new value, never a torn one.
// ============================================================================

class A2DPRateResampler : public Stream {
public:
    explicit A2DPRateResampler(Print &destination, int outputRate = 48000)
        : _dest(destination), _dstRate(outputRate) {}

    // ---- Stream interface ----------------------------------------------
    size_t write(const uint8_t *data, size_t len) override;
    size_t write(uint8_t b) override { return write(&b, 1); }
    int    available() override { return 0; }
    int    read()      override { return -1; }
    int    peek()      override { return -1; }
    void   flush()     override { _dest.flush(); }

    // ---- Connection lifecycle ------------------------------------------
    // Called from handleDeviceConnected() / handleDeviceDisconnected()
    // (loop context, not BT task). Clears rate lock and resampler state so
    // the next peer's rate is measured from scratch. Safe while the BT
    // task is writing: single-word fields, transient re-detection is
    // harmless (peers send silence for the first 100-300 ms anyway).
    void reset() {
        // [detection-window-shipping-fix]
        // Was: _srcRate = 0, _rateLocked = false. That forced the resampler
        // through 6-10 s of Phase-1 passthrough on every connect where the
        // A2DP stack could not report sample_rate() at CONNECTED (many
        // Windows/iPhone peers). During Phase 1, 44.1 kHz PCM feeds a
        // 48 kHz DMA and pitches up 8.8 % with DMA underruns — audible as
        // "the first few seconds of music sound wrong".
        //
        // Now: default-assume 44.1 kHz, which is what the overwhelming
        // majority of A2DP peers negotiate. Phase 2's two-window switch
        // detector (already in place) will correct to 48 kHz in ~1 s on
        // the rare Android peer that actually sends 48 kHz — so worst
        // case is 1 s of 8.8 % pitch error on a minority case, instead
        // of 6-10 s on the majority case. handleDeviceConnected() can
        // still call setPeerRate() to harden further when the stack
        // reports the rate correctly.
        _srcRate             = 44100;
        _srcRateHz           = 44100.0;
        _rateLocked          = true;   // skip Phase-1 entirely
        _detectStartMs       = 0;
        _detectBytes         = 0;
        _lastWriteMs         = 0;
        _pos                 = 0.0;
        _haveHistory         = false;
        _trackStartMs        = 0;
        _trackBytes          = 0;
        _switchCandidateHz   = 0.0;
        _switchCandidateHits = 0;
        _peerRateKnown       = false;  // Phase-2 detector stays armed
        _detectConfirm       = 0;
        _lastSnapped         = 0;
        _suspended           = false;
    }

    // ---- Diagnostics ---------------------------------------------------
    // [diagnostics-snapped-rate]
    // Report the SNAPPED integer rate, which is exactly what
    // writeResampled() uses to compute the ratio. Reporting the EMA
    // (_srcRateHz) here was misleading: it let the log claim "32 kHz"
    // while the resampler was actually rendering a correct 44100 -> 48000
    // conversion, and made the "which rate am I really at?" question
    // unanswerable from the logs.
    int         getDetectedInputRate() const {
        return _rateLocked ? _srcRate : 0;
    }
    int         getOutputRate()        const { return _dstRate; }
    bool        isLocked()             const { return _rateLocked; }
    const char *getMode()              const {
        if (!_rateLocked) return "detecting";
        // [restore-resampling] Resampling is back on. Report it so the
        // source-board log and the S3's RATEINFO line stop claiming
        // "passthrough" while a real conversion is happening.
        return (_srcRate != _dstRate) ? "resampling" : "passthrough";
    }

    // [bt-stall-watchdog]
    // Time of the last successful write() call from the BT stack.
    // 0 = no write since reset/connect. The watchdog in EVDBTMain.h
    // compares this against millis() to detect "connected but silent",
    // which is_connected() cannot see (it stays true through a stall).
    uint32_t getLastWriteMs()          const { return _lastWriteMs; }

    // [buf-short-writes] Lifetime count of short downstream writes.
    // Non-zero means the I2S DMA ring could not accept a full flush and
    // audio samples were silently dropped. Reset to 0 by reset().
    uint32_t getBufShortWrites()       const { return _bufShortWrites; }

    // [passthrough-arch]
    // Called from loop() around an I2S end/begin rate change. While
    // suspended, write() returns immediately without calling _dest.write()
    // so the AudioTools I2SStream is never touched while the DMA ring is
    // torn down. Suspension window is ~50 ms; the BT stack holds its
    // frames internally during this time and delivers them when we resume.
    void setSuspended(bool s)                { _suspended = s; }

    // [peer-rate-authoritative] Lock immediately to a rate reported by the
    // A2DP stack (get_sample_rate()). Skips byte-count detection AND codec-
    // switch re-detection. A real codec switch renegotiates and re-fires
    // CONNECTED, which calls setPeerRate() again — no need to detect it.
    void setPeerRate(int rate) {
        if (rate <= 0) return;
        _srcRate             = rate;
        _srcRateHz           = (double)rate;
        _peerRateKnown       = true;   // publish before the commit store
        _rateLocked          = true;   // write() gates on this last
        _detectStartMs       = 0;
        _detectBytes         = 0;
        _detectConfirm       = 0;
        _lastSnapped         = 0;
        _switchCandidateHz   = 0.0;
        _switchCandidateHits = 0;
        _trackStartMs        = 0;
        _trackBytes          = 0;
    }

    // [peer-type-hint] Restricts snapRate() to {48000,44100} for PCs so a
    // bursty 44.1 kHz measurement can never mis-lock to 32/24/16/8 kHz
    // (the source of the "16 kHz passthrough" incident).
    void setPeerType(bool isComputer) { _peerIsComputer = isComputer; }
    // [ota-force-silence]
    // Set true during a source OTA. writeResampled() outputs zeros at
    // exactly _dstRate instead of real audio, so the I2S DMA stays fed
    // and a flash-erase stall (which disables cache for ~50-100 ms on
    // both cores) does not produce an audible glitch downstream.
    void setForceSilence(bool s) { _forceSilence = s; }

private:
    Print &_dest;
    // [ctor-defaults-match-reset]
    // Constructor defaults match reset() so the very first write() goes
    // straight through Phase 2 (correct 44.1 kHz ratio) instead of Phase 1
    // passthrough. Closes a race where the BT stack could deliver audio
    // before handleDeviceConnected() runs from loop() and calls reset().
    // 44.1 kHz is the overwhelming A2DP default; Phase 2's 500 ms switch
    // detector corrects a genuine 48 kHz peer in ~1 s.
    int    _srcRate    = 44100;
    int    _dstRate    = 48000;
    bool   _rateLocked = true;

    // Rate-detection window (pre-lock)
    uint32_t _detectStartMs = 0;
    uint32_t _detectBytes   = 0;
    uint32_t _lastWriteMs   = 0;

    // Post-lock 500 ms rolling measurement window. Used ONLY to detect a
    // genuine codec switch; small drift does not change the ratio (see
    // the write() Phase-2 block below).
    uint32_t _trackStartMs = 0;
    uint32_t _trackBytes   = 0;

    // [switch-confirmation]
    // A single bursty 500 ms window on a 44.1 kHz PC peer can measure
    // ~32 kHz (27 % low), which the previous code accepted as a real
    // codec switch, drove the ratio to 32000/48000 = 0.667, and then
    // became self-sustaining (the ratio's backpressure kept the
    // measurement low forever). Require TWO CONSECUTIVE windows that
    // both (a) snap to the SAME new rate and (b) each measure within
    // 2 % of that snap. A genuine 32 kHz codec produces tight, repeated
    // measurements and switches in ~1.5 s; a bursty transient produces
    // scattered, loose measurements that never accumulate hits.
    double  _switchCandidateHz   = 0.0;
    uint8_t _switchCandidateHits = 0;

    // Slow EMA of the measured rate. Diagnostics only — NOT used for the
    // ratio. Kept so /api/health can still show the peer's crystal rate.
    double _srcRateHz = 0.0;

    // Cubic resampler state (Catmull-Rom, same kernel as the S3 server)
    double  _pos         = 0.0;
    int16_t _prevL       = 0, _prevR = 0;
    int16_t _lastL       = 0, _lastR = 0;
    bool    _haveHistory = false;

    // [passthrough-arch] Set by loop() around an I2S rate change. write()
    // checks this and drops data while set, so the BT stack does not call
    // into a torn-down I2SStream.
    volatile bool _suspended = false;
    bool _forceSilence = false;   // [ota-force-silence]

    bool    _peerIsComputer  = false;   // [peer-type-hint] snapRate restriction
    bool    _peerRateKnown   = false;   // [peer-rate-authoritative] setPeerRate fired
    uint8_t _detectConfirm   = 0;       // [detect-confirmation] stable windows
    int     _lastSnapped     = 0;       // [detect-confirmation] last snap result

    static const int OUT_FRAMES = 512;   // max frames per downstream flush
    int16_t _outBuf[OUT_FRAMES * 2];

    // [buf-short-writes] Counter of downstream write() calls that returned
    // fewer bytes than requested. AudioTools' I2SStream returns partial
    // writes under DMA backpressure; those lost samples are audio. Diagnostics
    // only: does not change behaviour. Written from the BT task, read from
    // loop() (RATEDIAG). 32-bit aligned on ESP32 means single-word reads
    // are atomic; volatile is enough for cross-task visibility.
    volatile uint32_t _bufShortWrites = 0;

    // [snapRate-decl] Required declaration for the out-of-class definition
    // near the bottom of this file. write() calls snapRate() in both
    // Phase 1 and Phase 2; without this line both calls fail to resolve
    // and the bottom definition has no matching declaration to attach to.
    int snapRate(uint32_t measuredHz) const;

    // [restore-resampling]
    // Cubic Catmull-Rom interpolator that converts the peer's PCM rate to
    // _dstRate. Called from write() once _rateLocked is true; during Phase 1
    // (detection) write() still passthroughs so the ratio is not applied to
    // a rate we do not yet believe in. Same kernel the S3 already uses.
    size_t writeResampled(const uint8_t *data, size_t len);
};

// ---- Stream::write --------------------------------------------------------
// [passthrough-arch]
// Rate detection only. Every frame is forwarded to _dest.write() at its
// native rate, never resampled. The I2S clock is expected to match the
// peer's rate (see applyDetectedSourceRate() in EVDBTMain.h); a mismatch
// is only possible during the first 800 ms after connect (detection
// window) and is silent because BT handshake completes before any real
// audio plays.
//
// The detection logic below is unchanged from the resampling version:
// Phase 1 measures over 800 ms and snaps to the nearest standard rate.
// Phase 2 keeps looking for a genuine codec switch (2 consecutive tight
// windows on a new rate) but no longer drives a ratio — that state is
// only used for diagnostics and to trigger a rate change in loop().
inline size_t A2DPRateResampler::write(const uint8_t *data, size_t len) {
    if (len == 0) return 0;

    // [passthrough-suspend] Loop() sets this around an I2S end/begin.
    // Drop data rather than touch a torn-down DMA ring.
    if (_suspended) return len;

    uint32_t now = millis();

    // ---- Phase 1: rate detection ---------------------------------------
    // [detect-confirmation] Require 3 consecutive 2 s windows snapping to
    // the same rate. A single 800 ms window on a bursty PC measures 12-17
    // kHz and used to lock I2S 3x too slow (sink blocks forever).
    // [peer-type-hint] For PC peers snapRate() only considers
    // {48000,44100}; a low-biased measurement yields snapped==0 and keeps
    // retrying instead of selecting a wrong candidate.
    if (!_rateLocked) {
        if (_detectStartMs == 0 || (now - _lastWriteMs) > 500) {
            _detectStartMs = now;
            _detectBytes   = 0;
            _detectConfirm = 0;
            _lastSnapped   = 0;
        }
        _detectBytes += len;
        _lastWriteMs  = now;

        if (now - _detectStartMs >= 2000) {
            uint32_t elapsedMs = now - _detectStartMs;
            uint32_t measuredHz =
                (uint32_t)(((uint64_t)_detectBytes / 4) * 1000ULL / elapsedMs);
            int snapped = snapRate(measuredHz);

            if (snapped > 0) {
                if (snapped == _lastSnapped) _detectConfirm++;
                else { _lastSnapped = snapped; _detectConfirm = 1; }
                if (_detectConfirm >= 3) {
                    _srcRate             = snapped;
                    _srcRateHz           = (double)snapped;
                    _rateLocked          = true;
                    _trackBytes          = 0;
                    _trackStartMs        = now;
                    _switchCandidateHz   = 0.0;
                    _switchCandidateHits = 0;
                }
            } else if (elapsedMs >= 10000 &&
                       measuredHz > 4000 && measuredHz < 200000) {
                // [force-lock-pc-safe] PC peers force to 44100; the raw
                // byte-count fallback is unreliable enough that 16/24/32
                // kHz is never a real PC rate.
                int forced = _peerIsComputer
                           ? 44100
                           : (int)((measuredHz + 50u) / 100u) * 100;
                _srcRate             = forced;
                _srcRateHz           = (double)forced;
                _rateLocked          = true;
                _trackBytes          = 0;
                _trackStartMs        = now;
                _switchCandidateHz   = 0.0;
                _switchCandidateHits = 0;
            }
            _detectStartMs = now;
            _detectBytes   = 0;
        }
        return _dest.write(data, len);
    }

    // ---- Phase 2: locked — detect codec switch only ---------------------
    _lastWriteMs = now;
    // [peer-rate-authoritative] Authoritative rate: a codec switch
    // renegotiates and re-fires CONNECTED → setPeerRate() again. Skip
    // the 500 ms window tracking (it was only needed when guessing).
    if (_peerRateKnown) {
        return writeResampled(data, len);
    }
    _trackBytes += len;

    uint32_t trackElapsed = now - _trackStartMs;
    if (trackElapsed >= 500) {
        double measured = ((double)_trackBytes / 4.0) * 1000.0 /
                          (double)trackElapsed;
        _trackBytes   = 0;
        _trackStartMs = now;

        if (measured > 4000.0 && measured < 200000.0) {
            double relDiff = fabs(measured - (double)_srcRate) /
                             (double)_srcRate;

            if (relDiff <= 0.08) {
                _srcRateHz += 0.20 * (measured - _srcRateHz);
                _switchCandidateHz   = 0.0;
                _switchCandidateHits = 0;
            } else {
                int newSnap = snapRate((uint32_t)(measured + 0.5));
                if (newSnap > 0 && newSnap != _srcRate) {
                    double tightness = fabs(measured - (double)newSnap) /
                                       (double)newSnap;
                    if (tightness <= 0.02) {
                        if (_switchCandidateHz > 0.0 &&
                            fabs((double)newSnap - _switchCandidateHz) /
                                _switchCandidateHz <= 0.02) {
                            _switchCandidateHits++;
                        } else {
                            _switchCandidateHz   = (double)newSnap;
                            _switchCandidateHits = 1;
                        }
                        if (_switchCandidateHits >= 2) {
                            // [switch-ratio-sanity-cap]
                            // Real A2DP codec switches change the rate by at
                            // most ~1.5x (48<->32). Pause-induced false
                            // positives can differ by 2x (44.1 -> 22.05) or
                            // more. Reject anything > 1.8x: no real codec
                            // swap needs it, but a pause-derived measurement
                            // will always trigger it.
                            double _ratio = (newSnap > _srcRate)
                                            ? (double)newSnap / (double)_srcRate
                                            : (double)_srcRate / (double)newSnap;
                            if (_ratio > 1.8) {
                                // Implausibly large change — discard the
                                // candidate ladder and keep the current rate.
                                _switchCandidateHz   = 0.0;
                                _switchCandidateHits = 0;
                            } else {
                                _srcRate             = newSnap;
                                _srcRateHz           = (double)newSnap;
                                _switchCandidateHz   = 0.0;
                                _switchCandidateHits = 0;
                            }
                        }
                    } else {
                        _switchCandidateHz   = 0.0;
                        _switchCandidateHits = 0;
                    }
                } else {
                    _switchCandidateHz   = 0.0;
                    _switchCandidateHits = 0;
                }
            }
        }
    }

    // [restore-resampling] Locked and peer not authoritative: run the
    // interpolator. Emits exactly _dstRate output samples per second
    // regardless of peer wobble, which is what stops the S3 ratio from
    // chasing ±0.06 % crystal drift.
    return writeResampled(data, len);
}

// ---- Cubic resampling (peer rate -> _dstRate) -----------------------------
// [restore-resampling]
// Same Catmull-Rom kernel as the S3 server's stage-1 resampler. Four taps
// (pm, p0, p1, p2); the `idx < 0` branch pulls pm/p0 from `_prevL/_lastL`
// so a fractional carry from the previous call is honoured. `_pos` is
// allowed to go negative after `_pos -= inFrames`; the next call's floor()
// yields idx < 0 and takes the history branch, which is correct.
//
// We do NOT need a p2 fallback when `idx+2 >= inFrames`: p2 is only the
// fourth tap; clipping it to p1 is what the S3 code does and it introduces
// at most one LSB of error on the very last frame of each flush.
inline size_t A2DPRateResampler::writeResampled(const uint8_t *data, size_t len) {
    const int16_t *in = (const int16_t *)data;
    size_t inFrames = len / 4;   // stereo, 16-bit
    if (inFrames == 0) return len;

    // [ota-force-silence] While a source OTA is in flight, keep the I2S
    // DMA fed with silence rather than real audio. Reset _pos and
    // _haveHistory so the first real frame after OTA starts on a clean
    // timeline (a stale carried-over _pos would cause a brief pitch
    // glide when silence ends).
    if (_forceSilence) {
        _pos         = 0.0;
        _haveHistory = false;
        size_t outFrames = (size_t)(((double)inFrames * (double)_dstRate) /
                                    (double)_srcRate + 0.5);
        while (outFrames > 0) {
            size_t chunk = outFrames > OUT_FRAMES ? OUT_FRAMES : outFrames;
            memset(_outBuf, 0, chunk * 4);
            _dest.write((const uint8_t *)_outBuf, chunk * 4);
            outFrames -= chunk;
        }
        return len;
    }

    if (!_haveHistory) {
        _prevL = _lastL = in[0];
        _prevR = _lastR = in[1];
        _haveHistory = true;
    }

    const double ratio = (double)_srcRate / (double)_dstRate;
    size_t outCount = 0;

    for (;;) {
        long idx = (long)floor(_pos);
        if (idx >= (long)inFrames - 1) break;

        const double frac = _pos - (double)idx;

        int16_t pmL, pmR, p0L, p0R, p1L, p1R, p2L, p2R;
        if (idx < 0) {
            pmL = _prevL; pmR = _prevR;
            p0L = _lastL; p0R = _lastR;
            p1L = in[0];  p1R = in[1];
            if (inFrames >= 2) { p2L = in[2]; p2R = in[3]; }
            else               { p2L = p1L;   p2R = p1R;   }
        } else {
            pmL = (idx == 0) ? _lastL : in[(idx - 1) * 2 + 0];
            pmR = (idx == 0) ? _lastR : in[(idx - 1) * 2 + 1];
            p0L = in[ idx      * 2 + 0];
            p0R = in[ idx      * 2 + 1];
            p1L = in[(idx + 1) * 2 + 0];
            p1R = in[(idx + 1) * 2 + 1];
            if ((size_t)(idx + 2) < inFrames) {
                p2L = in[(idx + 2) * 2 + 0];
                p2R = in[(idx + 2) * 2 + 1];
            } else {
                p2L = p1L; p2R = p1R;
            }
        }

        const float t  = (float)frac;
        const float t2 = t * t;
        const float t3 = t2 * t;
        const float am = -0.5f * t3 +        t2 - 0.5f * t;
        const float a0 =  1.5f * t3 - 2.5f * t2              + 1.0f;
        const float a1 = -1.5f * t3 + 2.0f * t2 + 0.5f * t;
        const float a2 =  0.5f * t3 - 0.5f * t2;

        const float outL = am * pmL + a0 * p0L + a1 * p1L + a2 * p2L;
        const float outR = am * pmR + a0 * p0R + a1 * p1R + a2 * p2R;

        int32_t li = (int32_t)(outL + (outL >= 0.0f ? 0.5f : -0.5f));
        int32_t ri = (int32_t)(outR + (outR >= 0.0f ? 0.5f : -0.5f));
        if (li >  32767) li =  32767;
        if (li < -32768) li = -32768;
        if (ri >  32767) ri =  32767;
        if (ri < -32768) ri = -32768;

        _outBuf[outCount * 2 + 0] = (int16_t)li;
        _outBuf[outCount * 2 + 1] = (int16_t)ri;
        outCount++;

        if (outCount >= OUT_FRAMES) {
            // [buf-short-writes] AudioTools' I2SStream returns the number of
            // bytes actually accepted; under DMA backpressure this can be
            // less than requested. Those samples are silently lost. Count
            // it; do NOT retry inline — retrying blocks the BT stack task
            // inside its own write callback, which is the exact pattern the
            // [pc-burst-headroom] comment in EVDBTBluetooth.h warns against.
            const size_t want  = outCount * 4;
            const size_t wrote = _dest.write((const uint8_t *)_outBuf, want);
            if (wrote < want) _bufShortWrites++;
            outCount = 0;
        }
        _pos += ratio;
    }

    if (outCount > 0) {
        // [buf-short-writes] Same accounting as the loop-end flush above.
        const size_t want  = outCount * 4;
        const size_t wrote = _dest.write((const uint8_t *)_outBuf, want);
        if (wrote < want) _bufShortWrites++;
    }

    if (inFrames >= 2) {
        _prevL = in[(inFrames - 2) * 2 + 0];
        _prevR = in[(inFrames - 2) * 2 + 1];
    }
    _lastL = in[(inFrames - 1) * 2 + 0];
    _lastR = in[(inFrames - 1) * 2 + 1];

    _pos -= (double)inFrames;
    return len;
}

// ---- Snap measurement to a standard A2DP rate -----------------------------
inline int A2DPRateResampler::snapRate(uint32_t measuredHz) const {
    // [snap-wide-tolerance-for-bursty-pc]
    // Windows A2DP delivers PCM in bursts with inter-burst gaps that bias
    // a wall-clock measurement 5-20 % LOW on a true 44.1 kHz SBC peer
    // (measured 38 900 Hz against a true 44 100 Hz). The old +/-3 %
    // window rejected those readings, so the code force-locked at 38 900
    // -> ratio 0.81 -> audio played 23 % SLOW (the "PC sounds slow" bug).
    //
    // New rule: pick the NEAREST standard candidate within +/-40 %.
    //   * 38 900 Hz -> 44 100 (12 % off) rather than 48 000 (19 % off). OK
    //   * 42 000 Hz -> 44 100 (4.8 % off) rather than 48 000 (12.5 % off). OK
    //   * 44 000 Hz -> 44 100 (0.2 % off). OK
    //   * 48 200 Hz -> 48 000 (0.4 % off). OK
    //   * 32 000 Hz -> 32 000 (0 % off). OK
    // Codec switches (44.1 <-> 48 = 8.4 %) still resolve to the right
    // candidate; the +/-40 % window's only risk is a >40 % biased reading,
    // which does not happen in practice.
    // [peer-type-hint] For PCs only {44100,48000} are valid A2DP rates.
    // Restricting candidates makes a bursty 44.1 kHz measurement unable to
    // select 32/24/22.05/16/8 kHz — the exact failure that locked I2S to
    // 16 kHz and stalled the sink.
    static const int kPcCandidates[]  = { 48000, 44100 };
    static const int kAnyCandidates[] = {
        96000, 88200, 48000, 44100, 32000, 24000, 22050, 16000, 8000
    };
    const int   *candidates = _peerIsComputer ? kPcCandidates : kAnyCandidates;
    const size_t nCand      = _peerIsComputer
                            ? sizeof(kPcCandidates)  / sizeof(kPcCandidates[0])
                            : sizeof(kAnyCandidates) / sizeof(kAnyCandidates[0]);
    int      best      = 0;
    uint32_t bestDelta = 0xFFFFFFFFu;
    for (size_t i = 0; i < nCand; i++) {
        int r = candidates[i];
        if (measuredHz * 100u < (uint32_t)r * 60u)  continue;  // < 60 % of r
        if (measuredHz * 100u > (uint32_t)r * 140u) continue;  // > 140 % of r
        // [snap-bias-higher]
        // Real A2DP peers overwhelmingly negotiate 44.1 or 48 kHz. A
        // bursty burst makes the measurement read LOW by 15-30 %; the
        // nearest candidate is then a lower rate (32 kHz) that would play
        // at 38 % pitch-up if we let it through. Bias the delta so a
        // higher candidate always wins a tie against a lower one, and
        // penalise low candidates by 25 % of the raw delta. The cost is
        // that a real 32 kHz peer measuring 34000 will prefer 44100
        // (12 % off) over 32000 (6 % off) — acceptable because 32 kHz
        // peers are vanishingly rare on A2DP, whereas bursty 44.1 is the
        // standard Windows pattern.
        uint32_t delta = (measuredHz > (uint32_t)r)
                       ? (measuredHz - (uint32_t)r)
                       : ((uint32_t)r - measuredHz);
        // Penalise below-measurement candidates by 1.25x
        if ((uint32_t)r < measuredHz) delta = (delta * 5u) / 4u;
        if (delta < bestDelta) { bestDelta = delta; best = r; }
    }
    return best;
}

#endif // EVDBTRESAMPLER_H