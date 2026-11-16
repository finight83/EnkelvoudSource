#include <algorithm>
#include <cassert>
#include <cstdio>
#include <vector>
#include "../EVDBTResampler.h"

uint32_t testMillis = 1;

class Output : public Print {
public:
    std::vector<uint8_t> bytes;
    size_t limit = 100000;
    int zeros = 0;
    bool stalled = false;
    size_t write(uint8_t b) override { return write(&b, 1); }
    size_t write(const uint8_t *data, size_t len) override {
        if (stalled || zeros-- > 0) return 0;
        size_t n = std::min(len, limit);
        bytes.insert(bytes.end(), data, data + n);
        return n;
    }
};

static void feed(A2DPRateResampler &r, int frames, int burst, int gap) {
    std::vector<int16_t> pcm(burst * 2);
    for (int i = 0; i < frames;) {
        int n = std::min(burst, frames - i);
        for (int j = 0; j < n; ++j) {
            pcm[2 * j] = (i + j) % 20000;
            pcm[2 * j + 1] = -pcm[2 * j];
        }
        assert(r.write(reinterpret_cast<const uint8_t *>(pcm.data()), n * 4) == size_t(n * 4));
        i += n;
        testMillis += gap;
    }
}

int main() {
    for (int rate : {16000, 32000, 44100, 48000}) {
        Output output;
        A2DPRateResampler r(output);
        r.setPeerRate(rate);
        feed(r, rate * 2, 441, 80);
        assert(r.hasPeerRate() && r.getDetectedInputRate() == rate);
        assert(std::abs(static_cast<int>(output.bytes.size() / 4) - 96000) <= 6);
        assert(r.getMaxWriteGapMs() == 80);
    }
    Output expected, partial;
    partial.limit = 37;
    partial.zeros = 3;
    A2DPRateResampler a(expected), b(partial);
    a.setPeerRate(44100);
    b.setPeerRate(44100);
    feed(a, 4410, 147, 2);
    feed(b, 4410, 147, 2);
    assert(expected.bytes == partial.bytes);
    assert(b.getBufShortWrites() > 0 && b.getDroppedBytes() == 0);

    b.reset();
    b.setPeerRate(48000);
    feed(b, 480, 120, 500);
    assert(b.hasPeerRate() && b.getDetectedInputRate() == 48000);
    b.setPeerRate(32000);
    feed(b, 320, 80, 500);
    assert(b.getDetectedInputRate() == 32000);
    b.reset();
    feed(b, 100, 100, 1);
    assert(!b.hasPeerRate() && b.getDetectedInputRate() == 44100);

    Output muted;
    A2DPRateResampler d(muted);
    d.setPeerRate(44100);
    d.setForceSilence(true);
    feed(d, 4410, 7, 1);
    assert(std::abs(static_cast<int>(muted.bytes.size() / 4) - 4800) <= 2);
    assert(std::all_of(muted.bytes.begin(), muted.bytes.end(), [](uint8_t v) { return v == 0; }));

    Output blocked;
    blocked.stalled = true;
    A2DPRateResampler c(blocked);
    feed(c, 100, 100, 1);
    assert(c.getDroppedBytes() > 0 && c.getMaxOutputMs() >= 250);
    std::puts("resampler tests passed");
}
