#include "cmsis_os.h"
#include "struct.hpp"

constexpr int N = 15;

Pos nav_pos;

struct pos_increase
{
  double x, y, z;
  bool wati_claw;
} goal[10] = {

  {0.0, 0.0, 0.0, false}, {0.0, 0.0, 0.0, false}, {0.0, 0.0, 0.0, false}, {0.0, 0.0, 0.0, false},
  {0.0, 0.0, 0.0, false}, {0.0, 0.0, 0.0, false}, {0.0, 0.0, 0.0, false}, {0.0, 0.0, 0.0, false},
  {0.0, 0.0, 0.0, false}, {0.0, 0.0, 0.0, false},

};

extern "C" {
void navigation_task()
{
  while (1) {
    for (int i = 0; i < N; i++) {
      vTaskDelay(1000);
    }
  }
}
}