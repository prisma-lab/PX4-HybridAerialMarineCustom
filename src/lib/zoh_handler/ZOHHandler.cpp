#include "ZOHHandler.hpp"
#include <string.h>

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
        // Calcoliamo l'offset nel buffer (assumendo buffer lineare o righe contigue)
        float* target_row = buffer + (idx * data_len);
        memcpy(target_row, data, data_len * sizeof(float));

        idx++;
        return (idx >= max_rows);
    }
    return true;
}
