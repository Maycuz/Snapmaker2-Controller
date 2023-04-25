#pragma once

#include <cstdint>
#include "../../Marlin.h"
#include "../../../../snapmaker/src/common/debug.h"

class TimeGenFunc {
public:
  const static uint32_t TGF_VALID_FLAG = 1<<0;
  const static uint32_t TGF_SYNC_FLAG = 1<<1;

public:
  uint32_t flag;
  int monotone;            // -1 monotone decreasing; 0 = const; 1:monotone increasing
  uint32_t start_tick;
  uint32_t end_tick;
  float time_wind;              // millisecond
  float start_pos, end_pos;
  // int start_pos, end_pos;
  // float coef_a, coef_b, coef_c;
  float coef_a, coef_b;
  // float avg_itv;

public:
  void log(uint32_t idx) {
    LOG_I("TGF(%d): tick: %u ~ %u, pos %f ~ %f, time_wind %.3fms, coef_a %f, coef_b %f\r\n",
    idx, start_tick, end_tick, start_pos, end_pos, time_wind, coef_a, coef_b);
    // LOG_I("TGF(%d): tick: %u ~ %u, pos %u ~ %u, time_wind %.3fms, coef_a %f, coef_b %f\r\n",
    // idx, start_tick, end_tick, start_pos, end_pos, time_wind, coef_a, coef_b);
  }

  float dist2time(float s) {
    // float c = coef_c - s;
    float c = s;

    if (IS_ZERO(coef_a)) {
      // if (!avg_itv && coef_b > 20.0 && time_wind > 100.0) {
      // if (IS_ZERO(avg_itv)) {
      //   avg_itv = 1.0 / coef_b;
      //   LOG_I("-------------------------------------: %f %f avg_itv %f\n", coef_a, coef_b, avg_itv);
      // }
      return (-c / coef_b);
    }

    float sqrt_d = coef_b * coef_b - 4 * coef_a * c;
    if (sqrt_d < 0.0)
      sqrt_d = 0;

    float d = SQRT(sqrt_d);
    if (monotone < 0) {
      return (-coef_b - d) / (2 * coef_a);
    }
    else {
      return (-coef_b + d) / (2 * coef_a);
    }
  }
};