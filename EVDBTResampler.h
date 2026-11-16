#pragma once
#ifndef EVDBTRESAMPLER_H
#define EVDBTRESAMPLER_H

#include <Arduino.h>
#include <Stream.h>
#include <string.h>

// Convert decoded stereo PCM to the fixed I2S rate. Codec callbacks publish
// the rate; only the PCM writer changes interpolation state.

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
    void reset() {
        __atomic_store_n(&_pendingRate, 0, __ATOMIC_RELEASE);
        __atomic_add_fetch(&_resetGeneration, 1, __ATOMIC_RELEASE);
    }

    // Applied by the PCM writer, never by the connection callback.
    void resetWriter() {
        _srcRate             = 44100;
        _rateLocked          = true;   // skip Phase-1 entirely
        _lastWriteMs         = 0;
        _pos                 = 0.0;
        _haveHistory         = false;
        _peerRateKnown       = false;
    }

    // ---- Diagnostics ---------------------------------------------------
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

    // Lifetime counters remain comparable across reconnects.
    uint32_t getBufShortWrites()       const { return _bufShortWrites; }
    uint32_t getDroppedBytes()         const { return _droppedBytes; }
    uint32_t getMaxWriteGapMs()        const { return _maxWriteGapMs; }
    uint32_t getMaxOutputMs()          const { return _maxOutputMs; }
    bool hasPeerRate()                 const { return _peerRateKnown; }

    // [passthrough-arch]
    // Called from loop() around an I2S end/begin rate change. While
    // suspended, write() returns immediately without calling _dest.write()
    // so the AudioTools I2SStream is never touched while the DMA ring is
    // torn down. Suspension window is ~50 ms; the BT stack holds its
    // frames internally during this time and delivers them when we resume.
    void setSuspended(bool s)                { _suspended = s; }

    // Called for every codec configuration, including mid-stream changes.
    void setPeerRate(int rate) {
        if (rate > 0) __atomic_store_n(&_pendingRate, rate, __ATOMIC_RELEASE);
    }

    void setPeerType(bool) {}
    // [ota-force-silence]
    // Set true during a source OTA. writeResampled() outputs zeros at
    // exactly _dstRate instead of real audio, so the I2S DMA stays fed
    // and a flash-erase stall (which disables cache for ~50-100 ms on
    // both cores) does not produce an audible glitch downstream.
    void setForceSilence(bool s) { _forceSilence = s; }

private:
    Print &_dest;
    int _pendingRate = 0;
    uint32_t _resetGeneration = 0;
    uint32_t _writerGeneration = 0;
    int    _srcRate    = 44100;
    int    _dstRate    = 48000;
    bool   _rateLocked = true;
    uint32_t _lastWriteMs   = 0;

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

    volatile bool _peerRateKnown = false;

    static const int OUT_FRAMES = 512;   // max frames per downstream flush
    int16_t _outBuf[OUT_FRAMES * 2];

    volatile uint32_t _bufShortWrites = 0;
    volatile uint32_t _droppedBytes = 0;
    volatile uint32_t _maxWriteGapMs = 0;
    volatile uint32_t _maxOutputMs = 0;

    void writeOutput(const uint8_t *data, size_t len) {
        size_t offset = 0;
        uint32_t started = millis();
        uint32_t stalledSince = started;
        while (offset < len) {
            size_t remaining = len - offset;
            size_t written = _dest.write(data + offset, remaining);
            if (written > remaining) written = remaining;
            if (written < remaining) ++_bufShortWrites;
            if (written) {
                offset += written;
                stalledSince = millis();
            } else if ((uint32_t)(millis() - stalledSince) >= 250) {
                _droppedBytes += len - offset;
                break;
            } else {
                delay(1);
            }
            uint32_t elapsed = millis() - started;
            if (elapsed > _maxOutputMs) _maxOutputMs = elapsed;
        }
    }

    size_t writeResampled(const uint8_t *data, size_t len);
};

// ---- Stream::write --------------------------------------------------------
inline size_t A2DPRateResampler::write(const uint8_t *data, size_t len) {
    if (len == 0) return 0;

    int peerRate = __atomic_exchange_n(&_pendingRate, 0, __ATOMIC_ACQ_REL);
    uint32_t generation = __atomic_load_n(&_resetGeneration, __ATOMIC_ACQUIRE);
    if (generation != _writerGeneration) {
        resetWriter();
        _writerGeneration = generation;
    }
    if (peerRate > 0) {
        _srcRate = peerRate;
        _peerRateKnown = true;
    }

    // [passthrough-suspend] Loop() sets this around an I2S end/begin.
    // Drop data rather than touch a torn-down DMA ring.
    if (_suspended) return len;

    uint32_t now = millis();

    if (_lastWriteMs != 0) {
        uint32_t gap = now - _lastWriteMs;
        if (gap > _maxWriteGapMs) _maxWriteGapMs = gap;
    }
    _lastWriteMs = now;
    // Arrival throughput is coupled to I2S backpressure and cannot reliably
    // distinguish 44.1 kHz from 48 kHz. Only codec configuration changes it.
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

        _outBuf[outCount * 2 + 0] = _forceSilence ? 0 : (int16_t)li;
        _outBuf[outCount * 2 + 1] = _forceSilence ? 0 : (int16_t)ri;
        outCount++;

        if (outCount >= OUT_FRAMES) {
            writeOutput((const uint8_t *)_outBuf, outCount * 4);
            outCount = 0;
        }
        _pos += ratio;
    }

    if (outCount > 0) {
        writeOutput((const uint8_t *)_outBuf, outCount * 4);
    }

    if (inFrames >= 2) {
        _prevL = in[(inFrames - 2) * 2 + 0];
        _prevR = in[(inFrames - 2) * 2 + 1];
    } else {
        _prevL = _lastL;
        _prevR = _lastR;
    }
    _lastL = in[(inFrames - 1) * 2 + 0];
    _lastR = in[(inFrames - 1) * 2 + 1];

    _pos -= (double)inFrames;
    return len;
}

#endif // EVDBTRESAMPLER_H