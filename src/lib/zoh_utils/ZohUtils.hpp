#pragma once

#include <cstdint>
#include <cstddef>

namespace zoh_utils  {


	/**
	* @class FrequencyEstimator
	* @brief // Utility for estimating the sampling frequency using a circular buffer of timestamps.
	*/
	class FrequencyEstimator
	{
		public:
		static constexpr size_t MAX_SAMPLES = 256;

		FrequencyEstimator() = default;
		~FrequencyEstimator() = default;

		/**
		* @brief insert a  timestamp to  compute average frequency in the  buffer.
		* @param current_timestamp  Current time in microseconds (us).
		*/
		float estimate(uint64_t current_timestamp);

		void reset();

		private:
		uint64_t _timestamps[MAX_SAMPLES]{};
		size_t   _index{0};
		size_t   _count{0};
	};


		/**
	* @class TopicSelector
	* @brief Utility to select message with highest frequency
	*/
	class TopicSelector
	{
		public:
		/**
		* @struct Candidate
		* @brief select message for the frequency comparison .
		*/
		struct Candidate {
			const char* identifier; ///message name
			float       frequency;  /// Frequency in Hz
			bool        updated;    /// cycle update status
		};

		TopicSelector() = default;
		~TopicSelector() = default;

		/**
		* @brief select best candidate index : the one with the highest frequency
		* @param candidates Array of stract  Candidate.
		* @param count number of candidate in the array .
		* @return int index of   the choosen message .
		*/
		static int getBestIndex(const Candidate candidates[], size_t count);

		/**
		* @brief return pointer to highest frequency message .
		* @param candidates Array of stract  Candidate.
		* @param count number of candidate in the array .
		* @return const Candidate* pointer to highest frequency message , or nullptr if it fails .
		*/
		static const Candidate* getBestCandidate(const Candidate candidates[], size_t count);

		private:
		static constexpr float FREQ_TOLERANCE = 2.0f;
	};


	class ZOHHandler
	{
		public:
		/**
		* @brief Generic data copy from a source to a destination (ZOH).
		* @param src   Pointer to the source data (e.g., a uORB struct)
		* @param dst   Pointer to the destination (e.g., a field in a ROS message)
		* @param size  Number of bytes to copy
		*/
		static void synchronize(const void* src, void* dst, size_t size);

		/**
		* @brief Inserts a set of floats into a buffer row by row.
		* @param data      Source float array
		* @param data_len  Number of elements in data
		* @param buffer    Pointer to the buffer (flattened or multidimensional)
		* @param idx       Current row index (will be incremented)
		* @param max_rows  Maximum number of rows in the buffer
		* @return true if the buffer is full
		*/
		static bool bufferData(const float* data, size_t data_len,
					float* buffer, int &idx, int max_rows);
	};




        class ZOHProcessor
	{
		public:

		/**
		* @brief Maximum number of topics supported by the processor.
		*/
		static constexpr int MAX_TOPICS = 16;

		/**
		* @brief Maximum size in bytes for each topic's data.
		*/
		static constexpr size_t MAX_DATA_SIZE = 64;

		// ----------------------
		// Input structure for process()
		// ----------------------
		/**
		* @brief Represents a single topic input to the ZOH processor.
		*/
		struct Input {
			const void* data;     /**< Pointer to the topic data (e.g., uORB struct) */
			uint64_t timestamp;   /**< Timestamp of the topic (from uORB) */
			size_t size;          /**< Size of the data in bytes */
			bool updated;         /**< True if the topic has new data this iteration */
		};

		// ----------------------
		// Output structure of process()
		// ----------------------
		/**
		* @brief Result of calling process() with multiple topics.
		*/
		struct Result {
			bool updated = false;                     /**< True if the fastest topic updated this iteration */
			int reference_index = -1;                 /**< Index of the fastest topic in the input array */
			const void* outputs[MAX_TOPICS] = {};     /**< Pointers to synchronized data (ZOH) for each topic */
		};

		// ----------------------
		// Main function
		// ----------------------
		/**
		* @brief Process multiple topics, hold previous values (ZOH), estimate frequencies, and select fastest topic.
		* @param topics  Array of topic inputs
		* @param count   Number of topics in the array
		* @return Result structure containing synchronized outputs and fastest topic info
		*/
		Result process(Input topics[], size_t count);

		private:

		// ----------------------
		// Internal storage
		// ----------------------
		/**
		* @brief Last held values for each topic. Updated automatically on new data.
		*/
		uint8_t _hold_data[MAX_TOPICS][MAX_DATA_SIZE] = {};

		/**
		* @brief Frequency estimators for each topic. Used internally to pick fastest topic.
		*/
		FrequencyEstimator _freq_est[MAX_TOPICS];
	};

}
