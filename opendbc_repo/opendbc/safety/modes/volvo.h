#pragma once

#include "opendbc/safety/declarations.h"

//#define DEBUG_VOLVO

/*
Volvo Electronic Control Units abbreviations and network topology
Platforms C1/EUCD

Look in selfdrive/car/volvo/values.py for more information.
*/

// Globals
static int giraffe_forward_camera_volvo = 0;
static int acc_ped_val_prev = 0;
static int volvo_desired_angle_last = 0;
static float volvo_speed = 0;

// diagnostic msgs
#define MSG_DIAG_CEM 0x726U
#define MSG_DIAG_PSCM 0x730U
#define MSG_DIAG_FSM 0x764U
#define MSG_DIAG_CVM 0x793U
#define MSG_DIAG_BROADCAST 0x7dfU

// platform C1
#define MSG_BTNS_VOLVO_C1 0x10U
#define MSG_FSM0_VOLVO_C1 0x30U
#define MSG_FSM1_VOLVO_C1 0xd0U
#define MSG_FSM2_VOLVO_C1 0x160U
#define MSG_FSM3_VOLVO_C1 0x270U
#define MSG_FSM4_VOLVO_C1 0x280U
#define MSG_FSM5_VOLVO_C1 0x355U
#define MSG_PSCM0_VOLVO_C1 0xe0U
#define MSG_PSCM1_VOLVO_C1 0x125U
#define MSG_ACC_PEDAL_VOLVO_C1 0x55U
#define MSG_SPEED_VOLVO_C1 0x130U

// platform eucd
#define MSG_FSM0_VOLVO_V60 0x51U
#define MSG_FSM1_VOLVO_V60 0x260U
#define MSG_FSM2_VOLVO_V60 0x262U
#define MSG_FSM3_VOLVO_V60 0x270U
#define MSG_FSM4_VOLVO_V60 0x31aU
#define MSG_FSM5_VOLVO_V60 0x3fdU
#define MSG_PSCM1_VOLVO_V60 0x246U
#define MSG_ACC_PEDAL_VOLVO_V60 0x20U
#define MSG_BTNS_VOLVO_V60 0x127U

// safety params
static const float DEG_TO_CAN_VOLVO_C1 = 1.0f/0.04395f;
static const int VOLVO_MAX_DELTA_OFFSET_ANGLE = (int)(20.0f/0.04395f)-1;
static const int VOLVO_MAX_ANGLE_REQ = 8189;
static const int VOLVO_MIN_ANGLE_REQ = -8190;

static const struct lookup_t VOLVO_LOOKUP_ANGLE_RATE_UP = {
  {7.0f, 17.0f, 36.0f},
  {2.0f, 0.25f, 0.1f}
};
static const struct lookup_t VOLVO_LOOKUP_ANGLE_RATE_DOWN = {
  {7.0f, 17.0f, 36.0f},
  {2.0f, 0.25f, 0.1f}
};

static struct sample_t volvo_angle_meas;

// TX checks - production (no diagnostic messages)
static const CanMsg VOLVO_C1_TX_MSGS[] = {
  {MSG_FSM0_VOLVO_C1, 0, 8, .check_relay = false},
  {MSG_FSM1_VOLVO_C1, 0, 8, .check_relay = true},
  {MSG_FSM2_VOLVO_C1, 0, 8, .check_relay = false},
  {MSG_FSM3_VOLVO_C1, 0, 8, .check_relay = false},
  {MSG_FSM4_VOLVO_C1, 0, 8, .check_relay = false},
  {MSG_BTNS_VOLVO_C1, 0, 8, .check_relay = false},
  {MSG_PSCM0_VOLVO_C1, 2, 8, .check_relay = false},
  {MSG_PSCM1_VOLVO_C1, 2, 8, .check_relay = false},
};

static const CanMsg VOLVO_EUCD_TX_MSGS[] = {
  {MSG_FSM0_VOLVO_V60, 0, 8, .check_relay = false},
  {MSG_FSM1_VOLVO_V60, 0, 8, .check_relay = false},
  {MSG_FSM2_VOLVO_V60, 0, 8, .check_relay = true},
  {MSG_FSM3_VOLVO_V60, 0, 8, .check_relay = false},
  {MSG_FSM4_VOLVO_V60, 0, 8, .check_relay = false},
  {MSG_FSM5_VOLVO_V60, 0, 8, .check_relay = false},
  {MSG_PSCM1_VOLVO_V60, 2, 8, .check_relay = false},
  {MSG_BTNS_VOLVO_V60, 0, 8, .check_relay = false},
};

// RX checks
static RxCheck volvo_c1_rx_checks[] = {
  {.msg = {{MSG_FSM0_VOLVO_C1,       2, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  {.msg = {{MSG_FSM1_VOLVO_C1,       2, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  {.msg = {{MSG_PSCM0_VOLVO_C1,      0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  {.msg = {{MSG_PSCM1_VOLVO_C1,      0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  {.msg = {{MSG_ACC_PEDAL_VOLVO_C1,  0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
};

static RxCheck volvo_eucd_rx_checks[] = {
  {.msg = {{MSG_PSCM1_VOLVO_V60,     0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  {.msg = {{MSG_FSM0_VOLVO_V60,      2, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  {.msg = {{MSG_ACC_PEDAL_VOLVO_V60, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
};

static void volvo_c1_rx_hook(const CANPacket_t *msg) {
  int bus = msg->bus;
  int addr = msg->addr;

  if (bus == 0) {
    // Current steering angle
    if (addr == (int)MSG_PSCM1_VOLVO_C1) {
      int angle_meas_new = (msg->data[5] << 8) | (msg->data[6]);
      angle_meas_new = angle_meas_new - 32768;
      update_sample(&volvo_angle_meas, angle_meas_new);
    }

    // Update vehicle speed
    if (addr == (int)MSG_SPEED_VOLVO_C1) {
      volvo_speed = ((msg->data[3] << 8) | (msg->data[4])) * 0.01f * KPH_TO_MS;
      UPDATE_VEHICLE_SPEED(volvo_speed);
    }

    // Gas pedal
    if (addr == (int)MSG_ACC_PEDAL_VOLVO_C1) {
      int hbyte = (msg->data[1] & 0x03U) << 8;
      int acc_ped_val = hbyte + msg->data[2];
      gas_pressed = acc_ped_val > 0;
      gas_pressed_prev = gas_pressed;

      // Disengage when accelerator pedal pressed at speed
      if ((acc_ped_val > 50) && (acc_ped_val_prev <= 50) && (volvo_speed > 1.0f)) {
        controls_allowed = false;
      }
      acc_ped_val_prev = acc_ped_val;
    }

    // If LKA msg is on bus 0, then relay is unexpectedly closed
    if ((safety_mode_cnt > 1U) && (addr == (int)MSG_FSM1_VOLVO_C1)) {
      relay_malfunction = true;
    }

    // If camera message is on bus 0, no forwarding
    if (addr == (int)MSG_FSM0_VOLVO_C1) {
      giraffe_forward_camera_volvo = 0;
    }
  }

  // ACC status from camera (bus 2)
  if ((addr == (int)MSG_FSM0_VOLVO_C1) && (bus == 2)) {
    giraffe_forward_camera_volvo = 1;
    bool cruise_engaged = (msg->data[7] & 0x04U) != 0U;
    pcm_cruise_check(cruise_engaged);
  }
}


static void volvo_eucd_rx_hook(const CANPacket_t *msg) {
  int bus = msg->bus;
  int addr = msg->addr;

  if (bus == 0) {
    // Gas pedal
    if (addr == (int)MSG_ACC_PEDAL_VOLVO_V60) {
      int acc_ped_val = ((msg->data[2] & 0x03U) << 8) | msg->data[3];
      gas_pressed = acc_ped_val > 0;
      gas_pressed_prev = gas_pressed;

      // Disengage when accelerator pedal pressed
      if ((acc_ped_val > 100) && (acc_ped_val_prev <= 100)) {
        controls_allowed = false;
      }
      acc_ped_val_prev = acc_ped_val;
    }

    // If LKA msg is on bus 0, then relay is unexpectedly closed
    if ((safety_mode_cnt > 1U) && (addr == (int)MSG_FSM2_VOLVO_V60)) {
      relay_malfunction = true;
    }

    // If camera message is on bus 0, no forwarding
    if (addr == (int)MSG_FSM0_VOLVO_V60) {
      giraffe_forward_camera_volvo = 0;
    }
  }

  // ACC status from camera (bus 2)
  if ((addr == (int)MSG_FSM0_VOLVO_V60) && (bus == 2)) {
    giraffe_forward_camera_volvo = 1;
    int acc_status = (msg->data[2] & 0x07U);
    bool cruise_engaged = (acc_status >= 6);
    pcm_cruise_check(cruise_engaged);
  }
}


static bool volvo_c1_tx_hook(const CANPacket_t *to_send) {
  bool tx = true;
  int addr = to_send->addr;
  bool violation = false;

  if (addr == (int)MSG_FSM1_VOLVO_C1) {
    int desired_angle = ((to_send->data[4] & 0x3fU) << 8) | (to_send->data[5]);
    bool lka_active = (to_send->data[7] & 0x3U) > 0;

    desired_angle = desired_angle - 8192;

    if (controls_allowed && lka_active) {
      float delta_angle_float;
      delta_angle_float = (safety_interpolate(VOLVO_LOOKUP_ANGLE_RATE_UP, volvo_speed) * DEG_TO_CAN_VOLVO_C1) + 1.0f;
      int delta_angle_up = (int)(delta_angle_float);
      delta_angle_float = (safety_interpolate(VOLVO_LOOKUP_ANGLE_RATE_DOWN, volvo_speed) * DEG_TO_CAN_VOLVO_C1) + 1.0f;
      int delta_angle_down = (int)(delta_angle_float);
      int highest_desired_angle = volvo_desired_angle_last + ((volvo_desired_angle_last > 0) ? delta_angle_up : delta_angle_down);
      int lowest_desired_angle = volvo_desired_angle_last - ((volvo_desired_angle_last >= 0) ? delta_angle_down : delta_angle_up);

      int hi_angle_req = SAFETY_MIN(desired_angle + VOLVO_MAX_DELTA_OFFSET_ANGLE, VOLVO_MAX_ANGLE_REQ);
      int lo_angle_req = SAFETY_MAX(desired_angle - VOLVO_MAX_DELTA_OFFSET_ANGLE, VOLVO_MIN_ANGLE_REQ);

      if (desired_angle > highest_desired_angle || desired_angle < lowest_desired_angle) {
        violation = true;
      }
      if (desired_angle > hi_angle_req || desired_angle < lo_angle_req) {
        violation = true;
      }
    }
    volvo_desired_angle_last = desired_angle;

    if ((!controls_allowed)
          && ((volvo_angle_meas.min - 1) >= VOLVO_MAX_ANGLE_REQ)
          && ((volvo_angle_meas.max + 1) <= VOLVO_MIN_ANGLE_REQ)
          && ((desired_angle < (volvo_angle_meas.min - 1)) || (desired_angle > (volvo_angle_meas.max + 1)))) {
      violation = true;
    }

    if (!controls_allowed && lka_active) {
      violation = true;
    }
  }

  // acc button check, only allow cancel button to be sent
  if (addr == (int)MSG_BTNS_VOLVO_C1) {
    violation = ((to_send->data[7] & 0xefU) > 0) | (to_send->data[6] > 0);
  }

  if (violation) {
    controls_allowed = false;
    tx = false;
  }

  return tx;
}


static bool volvo_eucd_tx_hook(const CANPacket_t *to_send) {
  SAFETY_UNUSED(to_send);
  bool tx = true;
  return tx;
}


static bool volvo_c1_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;

  // Block FSM1 (LKA message) from bus 2 to bus 0
  if (bus_num == 2 && addr == (int)MSG_FSM1_VOLVO_C1) {
    block_msg = true;
  }

  return block_msg;
}


static bool volvo_eucd_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;

  // Block FSM2 (LKA message) from bus 2 to bus 0
  if (bus_num == 2 && addr == (int)MSG_FSM2_VOLVO_V60) {
    block_msg = true;
  }

  return block_msg;
}


static safety_config volvo_c1_init(uint16_t param) {
  SAFETY_UNUSED(param);
  controls_allowed = false;
  relay_malfunction = false;
  giraffe_forward_camera_volvo = 0;

  for (int i = 0; i < MAX_SAMPLE_VALS; i++) {
    volvo_angle_meas.values[i] = 0;
  }
  volvo_angle_meas.min = 0;
  volvo_angle_meas.max = 0;

  return BUILD_SAFETY_CFG(volvo_c1_rx_checks, VOLVO_C1_TX_MSGS);
}


static safety_config volvo_eucd_init(uint16_t param) {
  SAFETY_UNUSED(param);
  controls_allowed = false;
  relay_malfunction = false;
  giraffe_forward_camera_volvo = 0;

  return BUILD_SAFETY_CFG(volvo_eucd_rx_checks, VOLVO_EUCD_TX_MSGS);
}


const safety_hooks volvo_c1_hooks = {
  .init = volvo_c1_init,
  .rx = volvo_c1_rx_hook,
  .tx = volvo_c1_tx_hook,
  .fwd = volvo_c1_fwd_hook,
};


const safety_hooks volvo_eucd_hooks = {
  .init = volvo_eucd_init,
  .rx = volvo_eucd_rx_hook,
  .tx = volvo_eucd_tx_hook,
  .fwd = volvo_eucd_fwd_hook,
};
