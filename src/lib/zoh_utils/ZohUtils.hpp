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

}
