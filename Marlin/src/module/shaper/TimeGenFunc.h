#pragma once

#include <cstdint>
#include "../../Marlin.h"
#include "../../../../snapmaker/src/common/debug.h"

class TimeGenFunc {
public:
  const static uint32_t TGF_STEP_FLAG = 1<<0;
  const static uint32_t TGF_SYNC_FLAG = 1<<1;
  const static uint32_t TGF_FILE_POS_SYNC_FLAG = 1<<2;
  const static uint32_t TGF_CHG_EXTRUDER = 1<<3;

public:
  uint32_t flag;
  int monotone;                 // -1 monotone decreasing; 0 = const; 1:monotone increasing
  uint32_t start_tick;
  uint32_t end_tick;
  float time_wind;              // millisecond
  float start_pos, end_pos;
  float coef_a, coef_b;

public:
  void log(uint32_t idx) {
    LOG_I("TGF(%d): tick: %u ~ %u, pos %f ~ %f, time_wind %.3fms, coef_a %f, coef_b %f\r\n",
    idx, start_tick, end_tick, start_pos, end_pos, time_wind, coef_a, coef_b);
  }

  float dist2time(float s) {
    float c = s;

    if (IS_ZERO(coef_a)) {
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