#pragma once

#include <cstdint>
#include "../../MarlinCore.h"
#include "../../../../snapmaker/debug/debug.h"
#include "AxisInputShaper.h"
#include "CircularBuffer.h"

struct err_print_tick {
  uint8_t   axis;
  uint32_t  cur_tick;
  uint32_t  out_tick;
};

struct pos_trace {
  float axis[4];
};

void circular_buffer_test(uint32_t round);
extern circular_buffer<struct pos_trace> cb_pos_trace;