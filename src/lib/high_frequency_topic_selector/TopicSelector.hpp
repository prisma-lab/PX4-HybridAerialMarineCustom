#pragma once

#include <cstdint>
#include <cstddef>

/**
 * @class TopicSelector
 * @brief Utility per la selezione della sorgente dati con frequenza maggiore.
 * Implementa una logica di arbitraggio leggera per sistemi real-time.
 */
class TopicSelector
{
public:
    /**
     * @struct Candidate
     * @brief Rappresenta una potenziale sorgente di dati.
     */
    struct Candidate {
        const char* identifier; ///< Nome identificativo del topic
        float       frequency;  ///< Frequenza stimata in Hz
        bool        updated;    ///< Stato di aggiornamento nel ciclo corrente
    };

    TopicSelector() = default;
    ~TopicSelector() = default;

    /**
     * @brief Seleziona l'indice del miglior candidato basandosi sulla frequenza.
     * @param candidates Array di strutture Candidate.
     * @param count Numero di candidati presenti nell'array.
     * @return int Indice del candidato vincente, oppure -1 se nessuno è valido.
     */
    static int getBestIndex(const Candidate candidates[], size_t count);

    /**
     * @brief Restituisce il puntatore al miglior candidato.
     * @param candidates Array di strutture Candidate.
     * @param count Numero di candidati presenti nell'array.
     * @return const Candidate* Puntatore al candidato vincente, o nullptr se fallisce.
     */
    static const Candidate* getBestCandidate(const Candidate candidates[], size_t count);
};
