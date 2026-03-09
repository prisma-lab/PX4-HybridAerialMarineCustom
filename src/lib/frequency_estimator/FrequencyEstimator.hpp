#pragma once

#include <cstdint>
#include <cstddef>

/**
 * @class FrequencyEstimator
 * @brief Utility per stimare la frequenza di campionamento basata su un buffer circolare di timestamp.
 */
class FrequencyEstimator
{
public:
    static constexpr size_t MAX_SAMPLES = 256;

    FrequencyEstimator() = default;
    ~FrequencyEstimator() = default;

    /**
     * @brief Inserisce un timestamp e calcola la frequenza media nel buffer.
     * @param current_timestamp Tempo attuale in microsecondi (us).
     */
    float estimate(uint64_t current_timestamp);

    void reset();

private:
    uint64_t _timestamps[MAX_SAMPLES]{};
    size_t   _index{0};
    size_t   _count{0};
};
