#pragma once

#include "opendbc/safety/declarations.h"

// diagnostic msgs
#define MSG_DIAG_CEM_VOLVO        0x726U
#define MSG_DIAG_PSCM_VOLVO       0x730U
#define MSG_DIAG_FSM_VOLVO        0x764U
#define MSG_DIAG_CVM_VOLVO        0x793U
#define MSG_DIAG_BROADCAST_VOLVO  0x7dfU

// platform C1 (V40)
#define MSG_BTNS_VOLVO_C1         0x10U
#define MSG_FSM0_VOLVO_C1         0x30U
#define MSG_FSM1_VOLVO_C1         0xd0U
#define MSG_FSM2_VOLVO_C1         0x160U
#define MSG_FSM3_VOLVO_C1         0x270U
#define MSG_FSM4_VOLVO_C1         0x280U
#define MSG_FSM5_VOLVO_C1         0x355U
#define MSG_PSCM0_VOLVO_C1        0xe0U
#define MSG_PSCM1_VOLVO_C1        0x125U
#define MSG_ACC_PEDAL_VOLVO_C1    0x55U
#define MSG_SPEED_VOLVO_C1        0x130U

// platform EUCD (V60)
#define MSG_FSM0_VOLVO_V60        0x51U
#define MSG_FSM1_VOLVO_V60        0x260U
#define MSG_FSM2_VOLVO_V60        0x262U
#define MSG_FSM3_VOLVO_V60        0x270U
#define MSG_FSM4_VOLVO_V60        0x31aU
#define MSG_FSM5_VOLVO_V60        0x3fdU
#define MSG_PSCM1_VOLVO_V60       0x246U
#define MSG_ACC_PEDAL_VOLVO_V60   0x20U
#define MSG_BTNS_VOLVO_V60        0x127U

// safety params
static const float DEG_TO_CAN_VOLVO_C1 = 1.0f / 0.04395f;
static const int VOLVO_MAX_DELTA_OFFSET_ANGLE = (int)(20.0f / 0.04395f) - 1;
static const int VOLVO_MAX_ANGLE_REQ = 8189;
static const int VOLVO_MIN_ANGLE_REQ = -8190;
static const int VOLVO_RELAY_TRNS_TIMEOUT = 1;

static const struct lookup_t VOLVO_LOOKUP_ANGLE_RATE_UP = {
  {7., 17., 36.},
  {2., .25, .1}
};

static const struct lookup_t VOLVO_LOOKUP_ANGLE_RATE_DOWN = {
  {7., 17., 36.},
  {2., .25, .1}
};

// Globals
static int giraffe_forward_camera_volvo = 0;
static int acc_active_prev_volvo = 0;
static int acc_ped_val_prev_volvo = 0;
static int volvo_desired_angle_last = 0;
static float volvo_speed = 0.0f;
static struct sample_t volvo_angle_meas;

// TX messages for C1
static const CanMsg VOLVO_C1_TX_MSGS[] = {
  {MSG_FSM0_VOLVO_C1, 0, 8, .check_relay = true},
  {MSG_FSM1_VOLVO_C1, 0, 8, .check_relay = true},
  {MSG_FSM2_VOLVO_C1, 0, 8, .check_relay = true},
  {MSG_FSM3_VOLVO_C1, 0, 8, .check_relay = true},
  {MSG_FSM4_VOLVO_C1, 0, 8, .check_relay = true},
  {MSG_BTNS_VOLVO_C1, 0, 8, .check_relay = false},
  {MSG_PSCM0_VOLVO_C1, 2, 8, .check_relay = true},
  {MSG_PSCM1_VOLVO_C1, 2, 8, .check_relay = true},
  {MSG_DIAG_FSM_VOLVO, 2, 8, .check_relay = false},
  {MSG_DIAG_PSCM_VOLVO, 0, 8, .check_relay = false},
  {MSG_DIAG_CEM_VOLVO, 0, 8, .check_relay = false},
  {MSG_DIAG_CVM_VOLVO, 0, 8, .check_relay = false},
  {MSG_DIAG_BROADCAST_VOLVO, 0, 8, .check_relay = false},
  {MSG_DIAG_BROADCAST_VOLVO, 2, 8, .check_relay = false},
};

// TX messages for EUCD
static const CanMsg VOLVO_EUCD_TX_MSGS[] = {
  {MSG_FSM0_VOLVO_V60, 0, 8, .check_relay = true},
  {MSG_FSM1_VOLVO_V60, 0, 8, .check_relay = true},
  {MSG_FSM2_VOLVO_V60, 0, 8, .check_relay = true},
  {MSG_FSM3_VOLVO_V60, 0, 8, .check_relay = true},
  {MSG_FSM4_VOLVO_V60, 0, 8, .check_relay = true},
  {MSG_FSM5_VOLVO_V60, 0, 8, .check_relay = true},
  {MSG_PSCM1_VOLVO_V60, 2, 8, .check_relay = true},
  {MSG_BTNS_VOLVO_V60, 0, 8, .check_relay = false},
  {MSG_DIAG_FSM_VOLVO, 2, 8, .check_relay = false},
  {MSG_DIAG_PSCM_VOLVO, 0, 8, .check_relay = false},
  {MSG_DIAG_CEM_VOLVO, 0, 8, .check_relay = false},
  {MSG_DIAG_CVM_VOLVO, 0, 8, .check_relay = false},
  {MSG_DIAG_BROADCAST_VOLVO, 0, 8, .check_relay = false},
  {MSG_DIAG_BROADCAST_VOLVO, 2, 8, .check_relay = false},
};

// RX checks for C1
static RxCheck volvo_c1_rx_checks[] = {
  {.msg = {{MSG_FSM0_VOLVO_C1, 2, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  {.msg = {{MSG_FSM1_VOLVO_C1, 2, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  {.msg = {{MSG_PSCM0_VOLVO_C1, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  {.msg = {{MSG_PSCM1_VOLVO_C1, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  {.msg = {{MSG_ACC_PEDAL_VOLVO_C1, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
};

// RX checks for EUCD
static RxCheck volvo_eucd_rx_checks[] = {
  {.msg = {{MSG_PSCM1_VOLVO_V60, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  {.msg = {{MSG_FSM0_VOLVO_V60, 2, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  {.msg = {{MSG_ACC_PEDAL_VOLVO_V60, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
};

static void volvo_c1_rx_hook(const CANPacket_t *msg) {
  int bus = msg->bus;
  int addr = msg->addr;

  if ((addr == MSG_FSM0_VOLVO_C1) && (bus == 2)) {
    giraffe_forward_camera_volvo = 1;
    bool acc_active = (msg->data[7] & 0x4U) != 0U;

    if (acc_active && !acc_active_prev_volvo) {
      controls_allowed = true;
    }
    if (!acc_active) {
      controls_allowed = false;
    }
    acc_active_prev_volvo = acc_active ? 1 : 0;
  }

  if (bus == 0) {
    if (addr == MSG_PSCM1_VOLVO_C1) {
      int angle_meas_new = (msg->data[5] << 8) | msg->data[6];
      angle_meas_new = angle_meas_new - 32768;
      update_sample(&volvo_angle_meas, angle_meas_new);
    }

    if (addr == MSG_SPEED_VOLVO_C1) {
      volvo_speed = (float)((msg->data[3] << 8) | msg->data[4]) * 0.01f / 3.6f;
    }

    if (addr == MSG_ACC_PEDAL_VOLVO_C1) {
      int hbyte = (msg->data[1] & 0x3) << 8;
      int acc_ped_val = hbyte | msg->data[2];
      if ((acc_ped_val > 50) && (acc_ped_val_prev_volvo <= 50) && (volvo_speed > 1.0f)) {
        controls_allowed = false;
      }
      acc_ped_val_prev_volvo = acc_ped_val;
    }

    if (addr == MSG_FSM0_VOLVO_C1) {
      giraffe_forward_camera_volvo = 0;
    }

    if ((safety_mode_cnt > VOLVO_RELAY_TRNS_TIMEOUT) && (addr == MSG_FSM1_VOLVO_C1)) {
      relay_malfunction = true;
    }
  }
}

static void volvo_eucd_rx_hook(const CANPacket_t *msg) {
  int bus = msg->bus;
  int addr = msg->addr;

  if ((addr == MSG_FSM0_VOLVO_V60) && (bus == 2)) {
    giraffe_forward_camera_volvo = 1;
    int acc_status = msg->data[2] & 0x7U;
    bool acc_active = acc_status >= 6;

    if (acc_active && !acc_active_prev_volvo) {
      controls_allowed = true;
    }
    if (!acc_active) {
      controls_allowed = false;
    }
    acc_active_prev_volvo = acc_active ? 1 : 0;
  }

  if ((addr == MSG_ACC_PEDAL_VOLVO_V60) && (bus == 0)) {
    int acc_ped_val = ((msg->data[2] & 0x3) << 8) | msg->data[3];
    if ((acc_ped_val > 100) && (acc_ped_val_prev_volvo <= 100)) {
      controls_allowed = false;
    }
    acc_ped_val_prev_volvo = acc_ped_val;
  }

  if ((addr == MSG_FSM0_VOLVO_V60) && (bus == 0)) {
    giraffe_forward_camera_volvo = 0;
  }

  if ((safety_mode_cnt > VOLVO_RELAY_TRNS_TIMEOUT) && (addr == MSG_FSM2_VOLVO_V60) && (bus == 0)) {
    relay_malfunction = true;
  }
}

static bool volvo_c1_tx_hook(const CANPacket_t *msg) {
  int addr = msg->addr;
  bool tx = true;

  if (relay_malfunction) {
    tx = false;
  }

  if (addr == MSG_FSM1_VOLVO_C1) {
    int desired_angle = ((msg->data[4] & 0x3f) << 8) | msg->data[5];
    bool lka_active = (msg->data[7] & 0x3) > 0;

    desired_angle = desired_angle - 8192;

    if (controls_allowed && lka_active) {
      float delta_angle_float = (safety_interpolate(VOLVO_LOOKUP_ANGLE_RATE_UP, volvo_speed) * DEG_TO_CAN_VOLVO_C1) + 1.0f;
      int delta_angle_up = (int)delta_angle_float;
      delta_angle_float = (safety_interpolate(VOLVO_LOOKUP_ANGLE_RATE_DOWN, volvo_speed) * DEG_TO_CAN_VOLVO_C1) + 1.0f;
      int delta_angle_down = (int)delta_angle_float;
      int highest_desired_angle = volvo_desired_angle_last + ((volvo_desired_angle_last > 0) ? delta_angle_up : delta_angle_down);
      int lowest_desired_angle = volvo_desired_angle_last - ((volvo_desired_angle_last >= 0) ? delta_angle_down : delta_angle_up);

      int hi_angle_req = SAFETY_MIN(desired_angle + VOLVO_MAX_DELTA_OFFSET_ANGLE, VOLVO_MAX_ANGLE_REQ);
      int lo_angle_req = SAFETY_MAX(desired_angle - VOLVO_MAX_DELTA_OFFSET_ANGLE, VOLVO_MIN_ANGLE_REQ);

      if (safety_max_limit_check(desired_angle, highest_desired_angle, lowest_desired_angle)) {
        tx = false;
      }
      if (safety_max_limit_check(desired_angle, hi_angle_req, lo_angle_req)) {
        tx = false;
      }
    }
    volvo_desired_angle_last = desired_angle;

    if (!controls_allowed) {
      if (((volvo_angle_meas.min - 1) >= VOLVO_MAX_ANGLE_REQ) &&
          ((volvo_angle_meas.max + 1) <= VOLVO_MIN_ANGLE_REQ) &&
          ((desired_angle < (volvo_angle_meas.min - 1)) || (desired_angle > (volvo_angle_meas.max + 1)))) {
        tx = false;
      }
      if (lka_active) {
        tx = false;
      }
    }
  }

  if (addr == MSG_BTNS_VOLVO_C1) {
    if (((msg->data[7] & 0xef) > 0) || (msg->data[6] > 0)) {
      tx = false;
    }
  }

  if (!tx) {
    controls_allowed = false;
  }

  return tx;
}

static bool volvo_eucd_tx_hook(const CANPacket_t *msg) {
  SAFETY_UNUSED(msg);
  bool tx = true;

  if (relay_malfunction) {
    tx = false;
  }

  return tx;
}

static bool volvo_c1_fwd_hook(int bus_num, int addr) {
  bool block_msg = true;

  if (!relay_malfunction && giraffe_forward_camera_volvo) {
    if (bus_num == 0) {
      block_msg = (addr == (int)MSG_PSCM1_VOLVO_C1);
    } else if (bus_num == 2) {
      block_msg = (addr == (int)MSG_FSM1_VOLVO_C1);
    }
  }
  return block_msg;
}

static bool volvo_eucd_fwd_hook(int bus_num, int addr) {
  bool block_msg = true;

  if (!relay_malfunction && giraffe_forward_camera_volvo) {
    if (bus_num == 0) {
      block_msg = (addr == (int)MSG_PSCM1_VOLVO_V60);
    } else if (bus_num == 2) {
      block_msg = (addr == (int)MSG_FSM2_VOLVO_V60);
    }
  }
  return block_msg;
}

static safety_config volvo_c1_init(uint16_t param) {
  SAFETY_UNUSED(param);
  controls_allowed = false;
  relay_malfunction = false;
  giraffe_forward_camera_volvo = 0;
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
