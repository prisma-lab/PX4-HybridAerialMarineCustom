#include "ZohUtils.hpp"
#include <cstring>

namespace zoh_utils {

bool ZOHProcessor::bufferDataMulti(const float* data_sources[], float* buffers[],
                                  size_t num_channels, int &idx, int max_rows)
{
    //  O(1) insertion at the current position (buffer "head")
    for (size_t i = 0; i < num_channels; i++) {
        if (data_sources[i] != nullptr && buffers[i] != nullptr) {
            buffers[i][idx] = *data_sources[i];
        }
    }

    // Circular index advancement
    idx = (idx + 1) % max_rows;

    // 3. "Buffer Ready" logic
    // In sliding window mode, once the buffer has been filled for the first time,
    // it is ALWAYS ready (returns true on every new sample inserted).
    // Note: To handle the initial filling phase, you may need an external flag,
    // but technically here we return true to trigger the computation.
    return true;
}
ZOHProcessor::Result ZOHProcessor::process(Input topics[], size_t count) {
    Result r{};
    r.reference_index = -1;
    r.updated = false;

    if (count == 0 || count > MAX_TOPICS) return r;

    float current_frequencies[MAX_TOPICS];
    bool updated_flags[MAX_TOPICS];

    for (size_t i = 0; i < count; i++) {
        Input &t = topics[i];
        updated_flags[i] = t.updated;

        // 1. estimate frequency
        if (t.updated) {
            current_frequencies[i] = estimate_frequency(i, t.timestamp);

            // 2. zoh
            size_t sz = (t.size > MAX_DATA_SIZE) ? MAX_DATA_SIZE : t.size;
            memcpy(_hold_data[i], t.data, sz);
        } else {
            current_frequencies[i] = _freq_states[i].current_freq;
        }

        // pointer to last saved data
        r.outputs[i] = _hold_data[i];
    }

    // 3. select index of fastest message
    int best_idx = select_best_topic(current_frequencies, updated_flags, count);

    if (best_idx != -1 && updated_flags[best_idx]) {
        r.updated = true;
        r.reference_index = best_idx;
    }

    return r;
}

float ZOHProcessor::estimate_frequency(size_t i, uint64_t ts) {
    if (ts == 0) return 0.0f;

    FreqState &s = _freq_states[i];
    s.timestamps[s.index] = ts;
    s.index = (s.index + 1) % MAX_SAMPLES;
    if (s.count < MAX_SAMPLES) s.count++;

    if (s.count < 2) return 0.0f;

    size_t oldest = (s.count < MAX_SAMPLES) ? 0 : s.index;
    size_t latest = (s.index == 0) ? MAX_SAMPLES - 1 : s.index - 1;

    uint64_t dt = s.timestamps[latest] - s.timestamps[oldest];
    if (dt > 0) {
        s.current_freq = static_cast<float>(s.count - 1) / (static_cast<float>(dt) / 1000000.0f);
    }
    return s.current_freq;
}

int ZOHProcessor::select_best_topic(const float freqs[], const bool updated[], size_t count) {
    int best_idx = -1;
    float max_f = -1.0f;

    for (size_t i = 0; i < count; i++) {
        // check if it is faster than the pevious selected with a tolerance
        if (updated[i]) {
            if (best_idx == -1 || (freqs[i] - max_f) > FREQ_TOLERANCE) {
                max_f = freqs[i];
                best_idx = static_cast<int>(i);
            }
        }
    }
    return best_idx;
}

void ZOHProcessor::synchronize(const void* src, void* dst, size_t size) {
    if (src && dst) memcpy(dst, src, size);
}

void ZOHProcessor::reset() {
    memset(_freq_states, 0, sizeof(_freq_states));
    memset(_hold_data, 0, sizeof(_hold_data));
}

} // namespace zoh_utils
