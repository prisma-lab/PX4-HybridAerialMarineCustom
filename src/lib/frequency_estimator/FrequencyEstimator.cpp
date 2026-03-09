#include "FrequencyEstimator.hpp"

float FrequencyEstimator::estimate(uint64_t current_timestamp)
{
    if (current_timestamp == 0) {
        return 0.0f;
    }

    // Inserimento circolare
    _timestamps[_index] = current_timestamp;
    _index = (_index + 1) % MAX_SAMPLES;

    if (_count < MAX_SAMPLES) {
        _count++;
    }

    if (_count < 2) {
        return 0.0f;
    }

    // Identificazione indici estremi
    // L'elemento più vecchio è alla posizione attuale _index se il buffer è pieno
    const size_t oldest_idx = (_count < MAX_SAMPLES) ? 0 : _index;
    const size_t latest_idx = (_index == 0) ? MAX_SAMPLES - 1 : _index - 1;

    const uint64_t dt_total = _timestamps[latest_idx] - _timestamps[oldest_idx];

    // Ritorna frequenza in Hz: (N campioni - 1) / (tempo totale in secondi)
    // Conversione esplicita per evitare warning di compilazione PX4
    if (dt_total > 0) {
        return static_cast<float>(_count - 1) / (static_cast<float>(dt_total) / 1000000.0f);
    }

    return 0.0f;
}

void FrequencyEstimator::reset()
{
    _index = 0;
    _count = 0;
}
