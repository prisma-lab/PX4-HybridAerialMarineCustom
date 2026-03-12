#include "ZohUtils.hpp"
#include <cstring>

namespace zoh_utils {

    // --- FrequencyEstimator Implementation ---
    float FrequencyEstimator::estimate(uint64_t current_timestamp)
    {
        if (current_timestamp == 0) {
            return 0.0f;
        }

        // circular buffer
        _timestamps[_index] = current_timestamp;
        _index = (_index + 1) % MAX_SAMPLES;

        if (_count < MAX_SAMPLES) {
            _count++;
        }

        if (_count < 2) {
            return 0.0f;
        }

        // IDENTIFY LAST INDEX : The oldest element is at the current position _index if the buffer is full
        const size_t oldest_idx = (_count < MAX_SAMPLES) ? 0 : _index;
        const size_t latest_idx = (_index == 0) ? MAX_SAMPLES - 1 : _index - 1;

        const uint64_t dt_total = _timestamps[latest_idx] - _timestamps[oldest_idx];

        // RETURN FREQUENCY IN Hz
        if (dt_total > 0) {
            return static_cast<float>(_count - 1) / (static_cast<float>(dt_total) / 1000000.0f);
        }

        return 0.0f;
    }

    void FrequencyEstimator::reset()
    {
        _index = 0;
        _count = 0;
        memset(_timestamps, 0, sizeof(_timestamps));
    }

    // --- TopicSelector Implementation ---
    int TopicSelector::getBestIndex(const Candidate candidates[], size_t count)
    {
        if (count == 0) {
            return -1;
        }

        int best_idx = 0;
        float best_freq = candidates[0].frequency;

        for (size_t i = 1; i < count; ++i) {

            float freq = candidates[i].frequency;

            // change index only if the topic is cleary faster it is to avoid jitter
            if ((freq - best_freq) > FREQ_TOLERANCE) {
                best_freq = freq;
                best_idx = static_cast<int>(i);
            }
        }

        return best_idx;
    }

    const TopicSelector::Candidate* TopicSelector::getBestCandidate(const Candidate candidates[], size_t count)
    {
        int index = getBestIndex(candidates, count);

        if (index != -1) {
            return &candidates[index];
        }

        return nullptr;
    }

    // --- ZOHHandler Implementation ---
    void ZOHHandler::synchronize(const void* src, void* dst, size_t size)
    {
        if (src != nullptr && dst != nullptr) {
            memcpy(dst, src, size);
        }
    }

    bool ZOHHandler::bufferData(const float* data, size_t data_len,
                            float* buffer, int &idx, int max_rows)
    {
        if (idx >= 0 && idx < max_rows) {
            // Compute the offset in the buffer (assuming a linear buffer or contiguous rows)
            float* target_row = buffer + (idx * data_len);
            memcpy(target_row, data, data_len * sizeof(float));

            idx++;
            return (idx >= max_rows);
        }
        return true;
    }


    ZOHProcessor::Result ZOHProcessor::process(Input topics[], size_t count)
    {
        Result r{};

        if (count == 0 || count > MAX_TOPICS) {
            return r; // nothing to do
        }

        // 1️ Build candidates and update holding
        TopicSelector::Candidate candidates[MAX_TOPICS];

        for (size_t i = 0; i < count; i++) {
            Input &t = topics[i];

            // Estimate frequency internally if topic updated
            float freq = 0.0f;
            if (t.updated) {
                freq = _freq_est[i].estimate(t.timestamp);
            }

            candidates[i].frequency = freq;
            candidates[i].updated = t.updated;

            // Update holding if new data
            if (t.updated) {
                size_t sz = t.size;
                if (sz > sizeof(_hold_data[i])) sz = sizeof(_hold_data[i]);
                memcpy(_hold_data[i], t.data, sz);
            }

            // Assign output always from holding
            r.outputs[i] = _hold_data[i];
        }

        //  Choose the fastest topic
        const TopicSelector::Candidate* best = TopicSelector::getBestCandidate(candidates, count);
        if (best && best->updated) {
            r.updated = true;
            r.reference_index = static_cast<int>(best - candidates);
        }

        return r;
    }

} // namespace zoh_utils
