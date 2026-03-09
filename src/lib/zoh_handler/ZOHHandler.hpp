#pragma once

#include <stdint.h>
#include <stddef.h>

class ZOHHandler
{
public:
    /**
     * @brief Copia generica di dati da un'origine a una destinazione (ZOH).
     * @param src       Puntatore ai dati sorgente (es. la struct uORB)
     * @param dst       Puntatore alla destinazione (es. campo del msg ROS)
     * @param size      Dimensione in byte da copiare
     */
    static void synchronize(const void* src, void* dst, size_t size);

    /**
     * @brief Inserisce un set di float in un buffer riga per riga.
     * @param data      Array di float sorgente
     * @param data_len  Numero di elementi in data
     * @param buffer    Puntatore al buffer (spianato o multidimensionale)
     * @param idx       Indice di riga corrente (verrà incrementato)
     * @param max_rows  Numero massimo di righe nel buffer
     * @return true se il buffer è pieno
     */
    static bool bufferData(const float* data, size_t data_len,
                          float* buffer, int &idx, int max_rows);
};
