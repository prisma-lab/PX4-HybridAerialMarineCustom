#include "TopicSelector.hpp"

int TopicSelector::getBestIndex(const Candidate candidates[], size_t count)
{
    if (count == 0) {
        return -1;
    }

    int best_idx = -1;
    float max_freq = 0.0f;

    for (size_t i = 0; i < count; ++i) {
        // Logica di confronto: cerchiamo la frequenza massima positiva
        if (candidates[i].frequency > max_freq) {
            max_freq = candidates[i].frequency;
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
