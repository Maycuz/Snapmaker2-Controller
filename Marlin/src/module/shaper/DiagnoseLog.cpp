#include "DiagnoseLog.h"
#include <stdlib.h>
#include "../../../../snapmaker/src/common/debug.h"

circular_buffer<int> cb_int(8);
circular_buffer<struct pos_trace> cb_pos_trace(32);

void circular_buffer_test(uint32_t round) {
  while(round--) {
    srand(millis());
    int c = rand();
    if (c&0x1) {
      srand(millis());
      int v = rand() % 100;
      LOG_I("+++ PUSH %d\r\n", v);
      cb_int.push(v);
    }
    else {
      int v;
      if( cb_int.pop(v) ) {
        LOG_I("--- POP %d\r\n", v);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  for (uint32_t i = 0; i < cb_int.capacity(); i++) {
    LOG_I("cb_int[%d] = %d\n", i, cb_int[i]);
  }
}