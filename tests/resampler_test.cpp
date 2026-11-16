#include <algorithm>
#include <cassert>
#include <cmath>
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
    size_t budget = size_t(-1);
    size_t write(uint8_t b) override { return write(&b, 1); }
    size_t write(const uint8_t *data, size_t len) override {
        if (stalled || zeros-- > 0 || !budget) return 0;
        size_t n = std::min(std::min(len, limit), budget);
        budget -= n;
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

static std::vector<uint8_t> signal(int frames, int rate) {
    std::vector<int16_t> pcm(frames * 2);
    for (int i = 0; i < frames; ++i) {
        double wave = std::sin(2.0 * 3.141592653589793 * 18000 * i / rate);
        pcm[2 * i] = static_cast<int16_t>(wave * 30000);
        pcm[2 * i + 1] = static_cast<int16_t>(-wave * 27000);
    }
    const uint8_t *begin = reinterpret_cast<const uint8_t *>(pcm.data());
    return std::vector<uint8_t>(begin, begin + pcm.size() * sizeof(int16_t));
}

static void feedBytes(A2DPRateResampler &r, const std::vector<uint8_t> &pcm,
                      size_t burst) {
    for (size_t pos = 0; pos < pcm.size();) {
        size_t n = std::min(burst, pcm.size() - pos);
        assert(r.write(pcm.data() + pos, n) == n);
        pos += n;
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
    blocked.budget = 37; // even a partial PCM frame may have reached I2S
    A2DPRateResampler c(blocked);
    std::vector<uint8_t> input = signal(2000, 44100);
    Output reference;
    A2DPRateResampler ref(reference);
    feedBytes(ref, input, input.size());
    size_t accepted = c.write(input.data(), input.size());
    assert(accepted > 0 && accepted < input.size());
    assert(c.getDroppedBytes() == 0 && c.getMaxOutputMs() >= 250);
    assert(c.write(input.data() + accepted, input.size() - accepted) == 0);
    blocked.budget = size_t(-1);
    assert(c.write(input.data() + accepted, input.size() - accepted) == input.size() - accepted);
    assert(blocked.bytes == reference.bytes);
    assert(c.getRejectedInputBytes() > 0);

    Output oddBlocked;
    oddBlocked.budget = 37;
    A2DPRateResampler odd(oddBlocked);
    assert(odd.write(input.data(), 3) == 3);
    size_t oddAccepted = odd.write(input.data() + 3, input.size() - 3);
    assert(oddAccepted % 4 == 1 && oddAccepted < input.size() - 3);
    oddBlocked.budget = size_t(-1);
    assert(odd.write(input.data() + 3 + oddAccepted, input.size() - 3 - oddAccepted) ==
           input.size() - 3 - oddAccepted);
    assert(oddBlocked.bytes == reference.bytes);

    // A timeout after accepting all input retains the unwritten output.
    Output smallBlocked, smallReference;
    smallBlocked.stalled = true;
    A2DPRateResampler sb(smallBlocked), sr(smallReference);
    auto small = signal(100, 44100);
    assert(sb.write(small.data(), small.size()) == small.size());
    assert(sb.write(small.data(), 1) == 0);
    smallBlocked.stalled = false;
    assert(sb.write(small.data(), small.size()) == small.size());
    feedBytes(sr, small, small.size());
    feedBytes(sr, small, small.size());
    assert(smallBlocked.bytes == smallReference.bytes);

    Output flushed, flushReference;
    flushed.stalled = true;
    A2DPRateResampler flushAdapter(flushed), flushRef(flushReference);
    feedBytes(flushAdapter, small, small.size());
    feedBytes(flushRef, small, small.size());
    flushed.stalled = false;
    flushAdapter.flush();
    assert(flushed.bytes == flushReference.bytes);

    for (int rate : {16000, 44100, 48000, 96000}) {
        auto pcm = signal(rate == 44100 ? rate : 4000, rate);
        Output whole;
        A2DPRateResampler one(whole);
        one.setPeerRate(rate);
        feedBytes(one, pcm, pcm.size());
        for (size_t chunk : {size_t(1), size_t(3), size_t(4), size_t(7), size_t(441 * 4)}) {
            Output split;
            A2DPRateResampler fragmented(split);
            fragmented.setPeerRate(rate);
            feedBytes(fragmented, pcm, chunk);
            assert(split.bytes == whole.bytes);
        }
    }

    // Same-rate notifications preserve history and partial PCM; actual rate
    // changes and reconnects start fresh, including an incomplete input frame.
    auto before = signal(123, 44100);
    auto after = signal(321, 32000);
    Output switched, separate;
    A2DPRateResampler sw(switched), first(separate), second(separate);
    feedBytes(sw, before, 7);
    feedBytes(first, before, 7);
    const uint8_t fragment[] = {42, 43, 44};
    assert(sw.write(fragment, sizeof(fragment)) == sizeof(fragment));
    sw.setPeerRate(32000);
    second.setPeerRate(32000);
    feedBytes(sw, after, 3);
    feedBytes(second, after, 3);
    assert(switched.bytes == separate.bytes);
    assert(sw.getDiscardedInputBytes() == sizeof(fragment));

    Output sameRate, sameReference;
    A2DPRateResampler same(sameRate), sameRef(sameReference);
    assert(same.write(input.data(), 3) == 3);
    same.setPeerRate(44100);
    assert(same.write(input.data() + 3, input.size() - 3) == input.size() - 3);
    feedBytes(sameRef, input, input.size());
    assert(sameRate.bytes == sameReference.bytes);

    Output suspended;
    A2DPRateResampler s(suspended);
    s.setSuspended(true);
    assert(s.write(input.data(), input.size()) == 0);
    assert(suspended.bytes.empty());
    s.setSuspended(false);
    feedBytes(s, input, 7);
    assert(suspended.bytes == reference.bytes);

    Output resetBlocked;
    resetBlocked.stalled = true;
    A2DPRateResampler rb(resetBlocked);
    assert(rb.write(small.data(), small.size()) == small.size());
    rb.reset();
    resetBlocked.stalled = false;
    feedBytes(rb, after, after.size());
    Output resetReference;
    A2DPRateResampler rr(resetReference);
    feedBytes(rr, after, after.size());
    assert(resetBlocked.bytes == resetReference.bytes);
    assert(rb.getDroppedBytes() > 0);

    Output resetPartial;
    resetPartial.budget = 37;
    A2DPRateResampler rp(resetPartial);
    assert(rp.write(small.data(), small.size()) == small.size());
    rp.reset();
    resetPartial.budget = size_t(-1);
    feedBytes(rp, after, after.size());
    std::vector<uint8_t> expectedReset(smallReference.bytes.begin(),
                                       smallReference.bytes.begin() + 40);
    expectedReset.insert(expectedReset.end(), resetReference.bytes.begin(), resetReference.bytes.end());
    assert(resetPartial.bytes == expectedReset);
    std::puts("resampler tests passed");
}
