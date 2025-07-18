/*
 * filter.h
 *
 *  Created on: Jul 5, 2024
 *      Author: user
 */

#ifndef INC_FILTERS_H_
#define INC_FILTERS_H_

typedef struct
{
  const float alpha;
  float yz1;
}
iir_ma_state_t;

float iir_ma(iir_ma_state_t *state, float currentReading);

#include <cstddef>   // for size_t
#include <cstdint>   // for uint8_t
#include <cmath>     // for fabsf

template <size_t WindowSize>
class SMA {
    static_assert(WindowSize > 0, "WindowSize must be > 0");

public:
    SMA(float outlierThresholdRatio = 0.0f,
        float outlierScaleFactor = 1.0f,
        bool clampOutliers = false)
        : index(0), sum(0.0f),
          scale(1.0f / WindowSize),
          outlierThresholdRatio(outlierThresholdRatio),
          outlierScaleFactor(outlierScaleFactor),
          clampOutliers(clampOutliers),
          previousAverage(0.0f)
    {
        for (size_t i = 0; i < WindowSize; ++i) {
            buffer[i] = 0.0f;
        }
    }

    float add(float value) {
        float avg = previousAverage;

        float deviationAbs = fabsf(value - avg);
        float limit = outlierThresholdRatio * fabsf(avg);

        if (deviationAbs > limit) {
            if (clampOutliers && avg>0) {
                // Clamp outlier
                if (value > avg + limit) value = avg + limit;
                else if (value < avg - limit) value = avg - limit;
            } else {
                // Scale outlier influence directly
                value = avg + (value - avg) * outlierScaleFactor;
            }
        }

        // Remove oldest value
        sum -= buffer[index];

        // Add new value
        buffer[index] = value;
        sum += value;

        index = (index + 1) % WindowSize;

        float result = sum * scale;

        previousAverage = result;
        return result;
    }


private:
    float buffer[WindowSize];
    size_t index;
    float sum;
    const float scale;

    const float outlierThresholdRatio;   // e.g. 0.5 = 50% deviation
    const float outlierScaleFactor;      // e.g. 0.25 = 25% influence if outlier
    const bool clampOutliers;            // NEW: clamp instead of scale
    float previousAverage;
};


#endif /* INC_FILTER_H_ */
