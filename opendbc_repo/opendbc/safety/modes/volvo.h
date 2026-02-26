#pragma once

#include "opendbc/safety/declarations.h"

#define GET_BUS(msg) ((msg)->bus)
#define GET_ADDR(msg) ((msg)->addr)
#define GET_BYTE(msg, b) ((msg)->data[(b)])

// Volvo Electronic Control Units abbreviations and network topology
// Platforms C1/EUCD

// diagnostic msgs
#define MSG_VOLVO_DIAG_CEM 0x726U
#define MSG_VOLVO_DIAG_PSCM 0x730U
#define MSG_VOLVO_DIAG_FSM 0x764U
#define MSG_VOLVO_DIAG_CVM 0x793U
#define MSG_VOLVO_DIAG_BROADCAST 0x7dfU

// platform C1 (V40)
#define MSG_VOLVO_C1_BTNS       0x010U  // Steering wheel buttons
#define MSG_VOLVO_C1_FSM0       0x030U  // ACC status message
#define MSG_VOLVO_C1_FSM1       0x0d0U  // LKA steering message
#define MSG_VOLVO_C1_FSM2       0x160U
#define MSG_VOLVO_C1_FSM3       0x270U
#define MSG_VOLVO_C1_FSM4       0x280U
#define MSG_VOLVO_C1_FSM5       0x355U
#define MSG_VOLVO_C1_PSCM0      0x0e0U
#define MSG_VOLVO_C1_PSCM1      0x125U  // Steering
#define MSG_VOLVO_C1_ACC_PEDAL  0x055U  // Gas pedal
#define MSG_VOLVO_C1_SPEED      0x130U  // Speed signal

// CAN bus numbers
#define VOLVO_MAIN 0
#define VOLVO_CAM  2

// safety params for C1 platform
const float VOLVO_DEG_TO_CAN = 1.0f / 0.04395f;  // 22.753, inverse of dbc scaling
const int VOLVO_MAX_DELTA_OFFSET_ANGLE = (int)(20.0f / 0.04395f) - 1;  // max degrees divided by k factor
const int VOLVO_MAX_ANGLE_REQ = 8189;   // max angle req, set at 2 steps from max
const int VOLVO_MIN_ANGLE_REQ = -8190;  // min angle req

// Globals for Volvo safety
static int volvo_giraffe_forward_camera = 0;
static int volvo_acc_active_prev = 0;
static int volvo_acc_ped_val_prev = 0;
static int volvo_desired_angle_last = 0;
static float volvo_speed = 0.0f;

// Angle rate limits lookup table
static const float VOLVO_ANGLE_RATE_UP_BP[] = {7.0f, 17.0f, 36.0f};  // m/s
static const float VOLVO_ANGLE_RATE_UP_V[] = {2.0f, 0.25f, 0.1f};   // deg/step
static const float VOLVO_ANGLE_RATE_DOWN_BP[] = {7.0f, 17.0f, 36.0f};
static const float VOLVO_ANGLE_RATE_DOWN_V[] = {2.0f, 0.25f, 0.1f};

// Interpolate function for angle rate limits
static float volvo_interpolate(const float *bp, const float *v, int len, float x) {
  if (x <= bp[0]) return v[0];
  if (x >= bp[len - 1]) return v[len - 1];
  
  for (int i = 1; i < len; i++) {
    if (x < bp[i]) {
      return v[i - 1] + (v[i] - v[i - 1]) * (x - bp[i - 1]) / (bp[i] - bp[i - 1]);
    }
  }
  return v[len - 1];
}

static safety_config volvo_c1_init(uint16_t param) {
  SAFETY_UNUSED(param);
  controls_allowed = false;
  volvo_giraffe_forward_camera = 0;
  volvo_acc_active_prev = 0;
  volvo_acc_ped_val_prev = 0;
  volvo_desired_angle_last = 0;
  volvo_speed = 0.0f;

  static const CanMsg VOLVO_C1_TX_MSGS[] = {
    {MSG_VOLVO_C1_FSM0, 0, 8, .check_relay = true},
    {MSG_VOLVO_C1_FSM1, 0, 8, .check_relay = true},
    {MSG_VOLVO_C1_FSM2, 0, 8, .check_relay = true},
    {MSG_VOLVO_C1_FSM3, 0, 8, .check_relay = true},
    {MSG_VOLVO_C1_FSM4, 0, 8, .check_relay = true},
    {MSG_VOLVO_C1_BTNS, 0, 8, .check_relay = false},
    {MSG_VOLVO_C1_PSCM0, 2, 8, .check_relay = true},
    {MSG_VOLVO_C1_PSCM1, 2, 8, .check_relay = true},
    {MSG_VOLVO_DIAG_FSM, 2, 8, .check_relay = false},
    {MSG_VOLVO_DIAG_PSCM, 0, 8, .check_relay = false},
    {MSG_VOLVO_DIAG_CEM, 0, 8, .check_relay = false},
    {MSG_VOLVO_DIAG_CVM, 0, 8, .check_relay = false},
    {MSG_VOLVO_DIAG_BROADCAST, 0, 8, .check_relay = false},
    {MSG_VOLVO_DIAG_BROADCAST, 2, 8, .check_relay = false},
  };

  static RxCheck volvo_c1_rx_checks[] = {
    {.msg = {{MSG_VOLVO_C1_FSM0, 2, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_VOLVO_C1_FSM1, 2, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_VOLVO_C1_PSCM0, 0, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_VOLVO_C1_PSCM1, 0, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_VOLVO_C1_ACC_PEDAL, 0, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  return BUILD_SAFETY_CFG(volvo_c1_rx_checks, VOLVO_C1_TX_MSGS);
}

static void volvo_c1_rx_hook(const CANPacket_t *msg) {
  int bus = GET_BUS(msg);
  int addr = GET_ADDR(msg);

  // Check ACC status from camera
  if ((addr == MSG_VOLVO_C1_FSM0) && (bus == VOLVO_CAM)) {
    volvo_giraffe_forward_camera = 1;
    bool acc_active = (GET_BYTE(msg, 7) & 0x04U) != 0U;

    // Only allow lateral control when ACC active
    if (acc_active && !volvo_acc_active_prev) {
      controls_allowed = true;
    }
    if (!acc_active) {
      controls_allowed = false;
    }
    volvo_acc_active_prev = acc_active;
  }

  if (bus == VOLVO_MAIN) {
    // Current steering angle
    if (addr == MSG_VOLVO_C1_PSCM1) {
      int angle_meas_new = (GET_BYTE(msg, 5) << 8) | GET_BYTE(msg, 6);
      // Remove offset
      angle_meas_new = angle_meas_new - 32768;
      // Update array of samples
      update_sample(&angle_meas, angle_meas_new);
    }

    // Get current speed
    if (addr == MSG_VOLVO_C1_SPEED) {
      // Factor 0.01 km/h
      volvo_speed = (float)(((GET_BYTE(msg, 3) << 8) | GET_BYTE(msg, 4))) * 0.01f / 3.6f;
    }

    // Disengage when accelerator pedal pressed
    if (addr == MSG_VOLVO_C1_ACC_PEDAL) {
      int hbyte = (GET_BYTE(msg, 1) & 0x03U) << 8;
      int acc_ped_val = hbyte + GET_BYTE(msg, 2);
      if ((acc_ped_val > 50) && (volvo_acc_ped_val_prev <= 50) && (volvo_speed > 1.0f)) {
        controls_allowed = false;
      }
      volvo_acc_ped_val_prev = acc_ped_val;
      gas_pressed = acc_ped_val > 50;
    }

    // Don't forward if FSM0 message is on bus 0
    if (addr == MSG_VOLVO_C1_FSM0) {
      volvo_giraffe_forward_camera = 0;
    }
  }
}

static bool volvo_c1_tx_hook(const CANPacket_t *msg) {
  bool tx = true;
  int addr = GET_ADDR(msg);
  bool violation = false;

  if (relay_malfunction) {
    tx = false;
  }

  if (addr == MSG_VOLVO_C1_FSM1) {
    int desired_angle = ((GET_BYTE(msg, 4) & 0x3fU) << 8) | GET_BYTE(msg, 5);  // 14 bits
    bool lka_active = (GET_BYTE(msg, 7) & 0x03U) > 0U;  // Steer direction > 0

    // Remove offset
    desired_angle = desired_angle - 8192;

    if (controls_allowed && lka_active) {
      // Calculate rate limits
      float delta_angle_up_f = volvo_interpolate(VOLVO_ANGLE_RATE_UP_BP, VOLVO_ANGLE_RATE_UP_V, 3, volvo_speed) * VOLVO_DEG_TO_CAN + 1.0f;
      float delta_angle_down_f = volvo_interpolate(VOLVO_ANGLE_RATE_DOWN_BP, VOLVO_ANGLE_RATE_DOWN_V, 3, volvo_speed) * VOLVO_DEG_TO_CAN + 1.0f;
      int delta_angle_up = (int)delta_angle_up_f;
      int delta_angle_down = (int)delta_angle_down_f;
      
      int highest_desired_angle = volvo_desired_angle_last + ((volvo_desired_angle_last > 0) ? delta_angle_up : delta_angle_down);
      int lowest_desired_angle = volvo_desired_angle_last - ((volvo_desired_angle_last >= 0) ? delta_angle_down : delta_angle_up);

      // Max request offset from actual angle
      int hi_angle_req = SAFETY_MIN(angle_meas.max + VOLVO_MAX_DELTA_OFFSET_ANGLE, VOLVO_MAX_ANGLE_REQ);
      int lo_angle_req = SAFETY_MAX(angle_meas.min - VOLVO_MAX_DELTA_OFFSET_ANGLE, VOLVO_MIN_ANGLE_REQ);

      // Check for violation
      if (desired_angle > highest_desired_angle) violation = true;
      if (desired_angle < lowest_desired_angle) violation = true;
      if (desired_angle > hi_angle_req) violation = true;
      if (desired_angle < lo_angle_req) violation = true;
    }

    if (controls_allowed && lka_active) {
      volvo_desired_angle_last = desired_angle;
    } else {
      volvo_desired_angle_last = angle_meas.values[0];
    }

    // Desired steer angle should be the same as measured when controls off
    if (!controls_allowed && lka_active) {
      violation = true;
    }
  }

  // ACC button check, only allow cancel button
  if (addr == MSG_VOLVO_C1_BTNS) {
    // Violation if any button other than cancel is pressed
    bool btn_violation = ((GET_BYTE(msg, 7) & 0xefU) > 0U) || (GET_BYTE(msg, 6) > 0U);
    if (btn_violation && !controls_allowed) {
      violation = true;
    }
  }

  if (violation) {
    controls_allowed = false;
    tx = false;
  }

  return tx;
}

static bool volvo_c1_fwd_hook(int bus_num, int addr) {
  bool block = false;

  if (volvo_giraffe_forward_camera) {
    if (bus_num == VOLVO_MAIN) {
      if (addr == MSG_VOLVO_C1_PSCM1) {
        block = true;
      }
    }

    if (bus_num == VOLVO_CAM) {
      if (addr == MSG_VOLVO_C1_FSM1) {  // block if lkas msg
        block = true;
      }
    }
  } else {
    block = true;
  }
  return block;
}

const safety_hooks volvo_c1_hooks = {
  .init = volvo_c1_init,
  .rx = volvo_c1_rx_hook,
  .tx = volvo_c1_tx_hook,
  .fwd = volvo_c1_fwd_hook,
};
