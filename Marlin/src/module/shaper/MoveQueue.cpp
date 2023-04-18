#include "MoveQueue.h"
#include "AxisInputShaper.h"
#include "../../../../snapmaker/src/common/debug.h"

MoveQueue moveQueue;
uint32_t MoveQueue::ms2tick;

static float ZERO_AXIS_R[NUM_AXIS] = {0, 0, 0, 0, 0};

MoveQueue::MoveQueue() {
  move_tail = move_head = 0;
  moves_head_tick = 0;
  moves_tail_tick = 0;
  LOOP_SHAPER_AXES(i) {
    last_mq_pos[i] = 0;
  }
}

void MoveQueue::init(uint32_t sm2t, uint32_t mswt, uint32_t mswrdt) {
  MoveQueue::ms2tick = sm2t;
  max_shape_window_tick = mswt;
  max_shape_window_right_delta_tick = mswrdt;
}

void Move::log(uint8_t idx) {
  LOG_I("\r\n===== move(%d) =====\r\n", idx);
  LOG_I("flag: %d, [%d %d %d]\r\n", flag, sync_target_pos[0], sync_target_pos[1], sync_target_pos[2]);
  LOG_I("dist mm: %f\r\n", distance);
  LOG_I("time msec: %f\r\n", ((float)(end_tick - start_tick))/MoveQueue::ms2tick);
  LOG_I("acc mm/msec^2: %f\r\n", accelerate);
  LOG_I("start_v steps/ms: %f\r\n", start_v);
  LOG_I("end_v steps/ms: %f\r\n", end_v);
  LOG_I("start tick: %d\r\n", start_tick);
  LOG_I("end tick: %d\r\n", end_tick);
  LOG_I("start pos: %f, %f, %f, %f\r\n", start_pos[0], start_pos[1], start_pos[2], start_pos[3]);
  LOG_I("end pos: %f, %f, %f, %f\r\n", end_pos[0], end_pos[1], end_pos[2], end_pos[3]);
  // LOG_I("start pos: %u, %u, %u, %u\r\n", start_pos[0], start_pos[1], start_pos[2], start_pos[3]);
  // LOG_I("end pos: %u, %u, %u, %u\r\n", end_pos[0], end_pos[1], end_pos[2], end_pos[3]);
  LOG_I("axis ratio:%f, %f, %f, %f, %f\r\n", axis_r[X_AXIS], axis_r[Y_AXIS], axis_r[Z_AXIS], axis_r[B_AXIS], axis_r[E_AXIS]);
}

void MoveQueue::reset() {
  move_tail = 0;
  move_head = 0;
  moves_head_tick = 0;
  moves_tail_tick = 0;
  LOOP_SHAPER_AXES(i) {
    last_mq_pos[i] = 0;
  }
}

bool MoveQueue::genMoves(block_t* block) {
  if (getFreeMoveSize() < 3) {
    // LOG_I("Move not enough\r\n");
    return false;
  }

  if (block->flag & BLOCK_FLAG_SYNC_POSITION) {
    addSyncMove(block->position);
    return true;
  }

  float millimeters   = block->millimeters;
  float entry_speed   = block->initial_speed / 1000.0f;
  float leave_speed   = block->final_speed / 1000.0f;
  float cruise_speed  = block->cruise_speed / 1000.0f;

  // 747 DEBUG
  // LOG_I("==== block =====\r\n");
  // LOG_I("millimeters: %f\r\n", block->millimeters);
  // LOG_I("initial_speed: %f\r\n", block->initial_speed);
  // LOG_I("final_speed: %f\r\n", block->final_speed);
  // LOG_I("cruise_speed: %f\r\n", block->cruise_speed);
  // LOG_I("acceleration: %f\r\n", block->acceleration);

  if (cruise_speed < EPSILON) {
    LOG_I("A zero speed block has no move\r\n");
    return false;
  }

  float i_cruise_speed = 1000.0f / block->cruise_speed;
  float acceleration = LROUND(block->acceleration) / 1000000.0F;
  float i_acceleration = 1000000.0f / LROUND(block->acceleration);

  float accelDistance = Planner::estimate_acceleration_distance(entry_speed, cruise_speed, acceleration);
  float decelDistance = Planner::estimate_acceleration_distance(cruise_speed, leave_speed, -acceleration);
  if (accelDistance < EPSILON) {
    accelDistance = 0;
  }
  if (decelDistance < EPSILON) {
    decelDistance = 0;
  }

  uint32_t acc_tick = ms2tick * (cruise_speed - entry_speed) * i_acceleration;
  uint32_t decel_tick = ms2tick * (cruise_speed - leave_speed) * i_acceleration;
  float plateau = millimeters - accelDistance - decelDistance;
  if (plateau < 0) {
    float newAccelDistance = Planner::intersection_distance(entry_speed, leave_speed, acceleration, millimeters);
    if (newAccelDistance > millimeters) {
      newAccelDistance = millimeters;
    }
    if (newAccelDistance < EPSILON) {
      newAccelDistance = 0;
    }
    if ((millimeters - newAccelDistance) < EPSILON) {
      newAccelDistance = millimeters;
    }
    accelDistance = newAccelDistance;

    cruise_speed = SQRT(2 * acceleration * newAccelDistance + sq(entry_speed));
    if (cruise_speed < leave_speed) {
      cruise_speed = leave_speed;
    }
    i_cruise_speed = 1 / cruise_speed;
    acc_tick = ms2tick * (cruise_speed - entry_speed) * i_acceleration;
    decelDistance = millimeters - accelDistance;
    decel_tick = ms2tick * (cruise_speed - leave_speed) * i_acceleration;
    plateau = 0;
  }

  float axis_r[NUM_AXIS];
  axis_r[X_AXIS] = block->axis_r[X_AXIS];
  axis_r[Y_AXIS] = block->axis_r[Y_AXIS];
  axis_r[Z_AXIS] = block->axis_r[Z_AXIS];
  axis_r[B_AXIS] = block->axis_r[B_AXIS];
  axis_r[E_AXIS] = block->axis_r[E_AXIS];

  file_pos = block->filePos;
  if (accelDistance > EPSILON) {
    Move * am = addMove(entry_speed, cruise_speed, acceleration, accelDistance, axis_r, acc_tick);
    am->file_pos = file_pos;
    #if ENABLED(LIN_ADVANCE)
    float K = block->use_advance_lead ? planner.extruder_advance_K[active_extruder] * 1000 : 0;
    am->delta_v = IS_ZERO(acceleration) ? 0 : K * acceleration;
    am->use_advance = true;
    #endif
  }
  if (plateau > EPSILON) {
    uint32_t plateau_tick = ms2tick * plateau * i_cruise_speed;
    Move * am = addMove(cruise_speed, cruise_speed, 0, plateau, axis_r, plateau_tick);
    am->file_pos = file_pos;
    #if ENABLED(LIN_ADVANCE)
    am->delta_v = 0.0;
    am->use_advance = false;
    #endif
  }
  if (decelDistance > EPSILON) {
    Move * am = addMove(cruise_speed, leave_speed, -acceleration, decelDistance, axis_r, decel_tick);
    am->file_pos = file_pos;
    #if ENABLED(LIN_ADVANCE)
    float K = block->use_advance_lead ? planner.extruder_advance_K[active_extruder] * 1000 : 0;
    am->delta_v = IS_ZERO(acceleration) ? 0 : K * (-acceleration);
    am->use_advance = true;
    #endif
  }

  return true;
}

Move *MoveQueue::addMove(float start_v, float end_v, float accelerate, float distance, float axis_r[], uint32_t t) {

  Move &move = moves[move_head];
  move.use_advance = false;
  move.flag = 0;
  move.start_v = start_v;
  move.end_v = end_v;
  move.accelerate = accelerate;
  move.distance = distance;
  move.t = t;
  move.file_pos = file_pos;

  move.axis_r[X_AXIS] = axis_r[X_AXIS];
  move.axis_r[Y_AXIS] = axis_r[Y_AXIS];
  move.axis_r[Z_AXIS] = axis_r[Z_AXIS];
  move.axis_r[B_AXIS] = axis_r[B_AXIS];
  move.axis_r[E_AXIS] = axis_r[E_AXIS];

  move.start_tick = moves_head_tick;
  move.end_tick = move.start_tick + t;
  moves_head_tick = move.end_tick;
  can_print_head_tick = moves_head_tick - max_shape_window_right_delta_tick;

  for (int i = 0; i < NUM_AXIS; ++i) {
    move.start_pos[i] = last_mq_pos[i];
    last_mq_pos[i] = move.end_pos[i] = move.start_pos[i] + move.distance * move.axis_r[i];
  }

  Move *add_move = &moves[move_head];
  move_head = nextMoveIndex(move_head);

  return add_move;
}

void MoveQueue::addEmptyMove(uint32_t time) {
  addMove(0, 0, 0, 0, ZERO_AXIS_R, time);
}

void MoveQueue::addSyncMove(int32_t *sync_pos) {
  Move *add_move = addMove(0, 0, 0, 0, ZERO_AXIS_R, 0);
  add_move->flag = BLOCK_FLAG_SYNC_POSITION;
  LOOP_SHAPER_AXES(i) {
    add_move->sync_target_pos[i] = sync_pos[i];
  }
}

float MoveQueue::getAxisPosition(int move_index, int axis, uint32_t tick) {
// int MoveQueue::getAxisPosition(int move_index, int axis, uint32_t tick) {
  Move &move = moves[move_index];
  float axis_r = move.axis_r[axis];
  float delta_time = (float)(tick - move.start_tick) / ms2tick;
  // LOG_I("Tick %d, move.start_tick %d, delta_time %f\r\n", tick, move.start_tick, delta_time);
  // int move_dist = LROUND((move.start_v + 0.5f * move.accelerate * delta_time) * delta_time * axis_r);
  // float move_dist = (move.start_v + 0.5f * move.accelerate * delta_time) * delta_time * axis_r;
  // return move.start_pos[axis] + move_dist;
  // return  move_dist;
  return move.start_pos[axis] + (move.start_v + 0.5f * move.accelerate * delta_time) * delta_time * axis_r;
}

void MoveQueue::moveTailForward(uint32_t print_tick) {
  while ((move_tail != move_head) && ELAPSED(print_tick, moves[move_tail].end_tick + SAFE_ISOLATION_TIME_STRIP)) {
    // LOG_I("print_tick %d, move[tail] end_tick %d\r\n", print_tick, moves[move_tail].end_tick);
    moves[move_tail].reset();
    move_tail = MOVE_MOD(move_tail + 1);
    // LOG_I("MT to %d\r\n", move_tail);
    #ifdef SHAPER_LOG_ENABLE
    LOG_I("MT to %d\r\n", move_tail);
    #endif
  }
  moves_tail_tick = moves[move_tail].start_tick;
  if ((moves_head_tick - moves_tail_tick) < max_shape_window_tick && getMoveSize() > MOVE_SIZE - 5) {
    LOG_E("#### Move queue spend tick < max shaper window tick\n");
  }
}

bool MoveQueue::haveMotion() {
  if (move_head == move_tail)
    return false;

  uint8_t h_idx = prevMoveIndex(move_head);
  for (int i = 0; i < NUM_AXIS; i++) {
    if (moves[move_tail].start_pos[i] != moves[h_idx].end_pos[i])
      return true;
  }
  return false;
}

float MoveQueue::spandTimeWindow() {
  return (moves_head_tick - moves_tail_tick) / ms2tick;
}

void MoveQueue::log() {
  uint8_t m_idx = move_tail;
  while (!RB_IS_EMPTY(move_head, m_idx)) {
    moves[m_idx].log(m_idx);
    m_idx = nextMoveIndex(m_idx);
  }
}