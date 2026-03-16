/**
 * @file ZohUtils.hpp
 * Zero-Order Hold (ZOH) processor and frequency-based topic selector.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>

namespace zoh_utils {

static constexpr size_t MAX_SAMPLES = 10;
static constexpr size_t MAX_TOPICS = 5;
static constexpr float FREQ_TOLERANCE = 0.5f;
static constexpr size_t MAX_DATA_SIZE = 128;

/**
 * @class ZOHProcessor
 * * Manages multiple data streams, estimates their publishing frequencies,
 * and synchronizes them using a Zero-Order Hold approach based on the fastest update.
 */
class ZOHProcessor {
public:
    /**
     * @struct Input
     * Input structure for each topic to be processed.
     */
    struct Input {
        void* data;          /**< Pointer to the topic data buffer */
        uint64_t timestamp;  /**< Current timestamp of the data (microseconds) */
        size_t size;         /**< Size of the data buffer in bytes */
        bool updated;        /**< Flag indicating if new data is available in this cycle */
    };

    /**
     * @struct Result
     * Result of the processing cycle.
     */
    struct Result {
        void* outputs[MAX_TOPICS]; /**< Array of pointers to the ZOH-buffered data */
        int reference_index;       /**< Index of the fastest/reference topic */
        bool updated;              /**< True if the reference topic was updated this cycle */
    };

    ZOHProcessor() { reset(); }

    /**
     * Process multiple input streams, update internal frequency estimates,
     * and determine the best (fastest) topic.
     * @return Result containing synchronized pointers and update status.
     */
    Result process(Input topics[], size_t count);

    /**
     * Reset internal state, frequency counters, and data buffers.
     */
    void reset();

    /**
     * Utility method for memory synchronization (memcpy wrapper).
     */
    static void synchronize(const void* src, void* dst, size_t size);

    /**
     * Utility method to buffer data into a matrix-like structure (e.g., for FFT).
     * @return True if the buffer reached the max_rows limit.
     */
    static bool bufferData(const float* data, size_t data_len, float* buffer, int &idx, int max_rows);

private:
    /**
     * @struct FreqState
     * Internal state for frequency estimation per topic.
     */
    struct FreqState {
        uint64_t timestamps[MAX_SAMPLES];
        size_t index;
        size_t count;
        float current_freq;
    };

    FreqState _freq_states[MAX_TOPICS];
    uint8_t   _hold_data[MAX_TOPICS][MAX_DATA_SIZE];

    /**
     * Update frequency estimation for a specific topic index.
     */
    float estimate_frequency(size_t topic_idx, uint64_t timestamp);

    /**
     * Select the best candidate index based on frequency and update status.
     */
    int select_best_topic(const float frequencies[], const bool updated[], size_t count);
};

} // namespace zoh_utils

