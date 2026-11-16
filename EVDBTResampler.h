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
        : _dest(destination), _dstRate(outputRate > 0 ? outputRate : 48000) {}

    // ---- Stream interface ----------------------------------------------
    size_t write(const uint8_t *data, size_t len) override;
    size_t write(uint8_t b) override { return write(&b, 1); }
    int    available() override { return 0; }
    int    read()      override { return -1; }
    int    peek()      override { return -1; }
    void   flush()     override {
        uint32_t generation = __atomic_load_n(&_resetGeneration, __ATOMIC_ACQUIRE);
        if (generation != _writerGeneration) {
            resetWriter();
            _writerGeneration = generation;
        }
        if (!_suspended && writeOutput() && produceOutput() && writeOutput())
            _dest.flush();
    }

    // ---- Connection lifecycle ------------------------------------------
    void reset() {
        __atomic_store_n(&_pendingRate, 0, __ATOMIC_RELEASE);
        __atomic_add_fetch(&_resetGeneration, 1, __ATOMIC_RELEASE);
    }

    // Applied by the PCM writer, never by the connection callback.
    void resetWriter() {
        // A downstream partial write can end in the middle of a frame.
        // Complete that frame before output from the new connection.
        size_t tail = _outOffset % 4 ? 4 - _outOffset % 4 : 0;
        _droppedBytes += _outBytes - _outOffset - tail;
        if (tail) {
            uint8_t *bytes = reinterpret_cast<uint8_t *>(_outBuf);
            memmove(bytes, bytes + _outOffset, tail);
        }
        _discardedInputBytes += _pcmBytes;
        _srcRate             = 44100;
        _rateLocked          = true;   // skip Phase-1 entirely
        _lastWriteMs         = 0;
        clearInputState();
        _outBytes = tail;
        _outOffset = 0;
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
    uint32_t getRejectedInputBytes()   const { return _rejectedInputBytes; }
    uint32_t getDiscardedInputBytes()  const { return _discardedInputBytes; }
    uint32_t getMaxWriteGapMs()        const { return _maxWriteGapMs; }
    uint32_t getMaxOutputMs()          const { return _maxOutputMs; }
    bool hasPeerRate()                 const { return _peerRateKnown; }

    // [passthrough-arch]
    // Called from loop() around an I2S end/begin rate change. While
    // suspended, write() accepts no input and does not touch the I2S stream.
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

    // Four most recent complete frames, and the next output's input position.
    int16_t _frames[4][2] = {};
    uint64_t _inputFrames = 0;
    uint64_t _nextFrame = 0;
    uint32_t _fraction = 0; // denominator: _dstRate
    uint8_t _pcm[4] = {};
    size_t _pcmBytes = 0;

    // [passthrough-arch] Set by loop() around an I2S rate change. write()
    // checks this and rejects data while set, so the BT stack does not call
    // into a torn-down I2SStream.
    volatile bool _suspended = false;
    bool _forceSilence = false;   // [ota-force-silence]

    volatile bool _peerRateKnown = false;

    static const int OUT_FRAMES = 512;   // max frames per downstream flush
    int16_t _outBuf[OUT_FRAMES * 2];
    size_t _outBytes = 0;
    size_t _outOffset = 0;

    volatile uint32_t _bufShortWrites = 0;
    volatile uint32_t _droppedBytes = 0;
    volatile uint32_t _rejectedInputBytes = 0;
    volatile uint32_t _discardedInputBytes = 0;
    volatile uint32_t _maxWriteGapMs = 0;
    volatile uint32_t _maxOutputMs = 0;

    void clearInputState() {
        _inputFrames = _nextFrame = 0;
        _fraction = 0;
        _pcmBytes = 0;
    }

    bool writeOutput() {
        uint32_t started = millis();
        uint32_t stalledSince = started;
        while (_outOffset < _outBytes) {
            size_t remaining = _outBytes - _outOffset;
            size_t written = _dest.write(
                reinterpret_cast<const uint8_t *>(_outBuf) + _outOffset, remaining);
            if (written > remaining) written = remaining;
            if (written < remaining) ++_bufShortWrites;
            if (written) {
                _outOffset += written;
                stalledSince = millis();
            } else if ((uint32_t)(millis() - stalledSince) >= 250) {
                uint32_t elapsed = millis() - started;
                if (elapsed > _maxOutputMs) _maxOutputMs = elapsed;
                return false;
            } else {
                delay(1);
            }
            uint32_t elapsed = millis() - started;
            if (elapsed > _maxOutputMs) _maxOutputMs = elapsed;
        }
        _outBytes = _outOffset = 0;
        return true;
    }

    bool produceOutput();
};

// A successful return means those input bytes are owned by this adapter,
// including any output retained after a downstream timeout. The caller must
// not replay accepted input, and may retry only the unaccepted suffix.
inline size_t A2DPRateResampler::write(const uint8_t *data, size_t len) {
    if (!len) return 0;

    uint32_t generation = __atomic_load_n(&_resetGeneration, __ATOMIC_ACQUIRE);
    if (generation != _writerGeneration) {
        resetWriter();
        _writerGeneration = generation;
    }
    if (_suspended) {
        _rejectedInputBytes += len;
        return 0;
    }

    // Finish old-rate output before applying a codec change. Neither pending
    // output nor the input that produced it may be replayed after a timeout.
    if (!writeOutput() || !produceOutput()) {
        _rejectedInputBytes += len;
        return 0;
    }
    int peerRate = __atomic_exchange_n(&_pendingRate, 0, __ATOMIC_ACQ_REL);
    if (peerRate > 0) {
        if (peerRate != _srcRate) {
            _discardedInputBytes += _pcmBytes;
            clearInputState();
        }
        _srcRate = peerRate;
        _peerRateKnown = true;
    }

    uint32_t now = millis();
    if (_lastWriteMs != 0) {
        uint32_t gap = now - _lastWriteMs;
        if (gap > _maxWriteGapMs) _maxWriteGapMs = gap;
    }
    _lastWriteMs = now;

    size_t accepted = 0;
    bool outputReady = true;
    while (accepted < len) {
        _pcm[_pcmBytes++] = data[accepted++];
        if (_pcmBytes == 4) {
            int16_t *frame = _frames[_inputFrames % 4];
            memcpy(frame, _pcm, 4);
            ++_inputFrames;
            _pcmBytes = 0;
            if (!produceOutput()) {
                outputReady = false;
                break;
            }
        }
    }
    _rejectedInputBytes += len - accepted;
    // On timeout the unwritten suffix remains in _outBuf; all accepted
    // source bytes are still owned, even when this is the end of the call.
    if (outputReady && _outBytes) writeOutput();
    return accepted;
}

// Wait for the real p2 instead of fabricating one at a write boundary.
// This introduces two input frames of lookahead, shared across calls.
inline bool A2DPRateResampler::produceOutput() {
    while (_inputFrames >= 3 && _nextFrame <= _inputFrames - 3) {
        const int16_t *pm = _frames[(_nextFrame ? _nextFrame - 1 : 0) % 4];
        const int16_t *p0 = _frames[_nextFrame % 4];
        const int16_t *p1 = _frames[(_nextFrame + 1) % 4];
        const int16_t *p2 = _frames[(_nextFrame + 2) % 4];
        const float t = (float)_fraction / (float)_dstRate;
        const float t2 = t * t;
        const float t3 = t2 * t;
        const float am = -0.5f * t3 +        t2 - 0.5f * t;
        const float a0 =  1.5f * t3 - 2.5f * t2              + 1.0f;
        const float a1 = -1.5f * t3 + 2.0f * t2 + 0.5f * t;
        const float a2 =  0.5f * t3 - 0.5f * t2;
        for (int channel = 0; channel < 2; ++channel) {
            float sample = am * pm[channel] + a0 * p0[channel] +
                           a1 * p1[channel] + a2 * p2[channel];
            int32_t value = (int32_t)(sample + (sample >= 0.0f ? 0.5f : -0.5f));
            if (value > 32767) value = 32767;
            if (value < -32768) value = -32768;
            _outBuf[_outBytes / 2 + channel] = _forceSilence ? 0 : (int16_t)value;
        }
        _outBytes += 4;
        uint32_t phase = _fraction + (uint32_t)_srcRate;
        _nextFrame += phase / (uint32_t)_dstRate;
        _fraction = phase % (uint32_t)_dstRate;
        if (_outBytes == sizeof(_outBuf) && !writeOutput()) return false;
    }
    return true;
}

#endif // EVDBTRESAMPLER_H
