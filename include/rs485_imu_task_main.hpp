#ifndef RS485_IMU_TASK_MAIN_HPP_
#define RS485_IMU_TASK_MAIN_HPP_

namespace RSI {

enum MSG_ID {
  REQ_INIT  = 0x01,
  REQ_DEBUG = 0xFF,
};

// Message共通変数
struct MsgCommon {
  uint8_t MsgId;
  uint8_t Sender;
  uint8_t Recv0;
  uint8_t Recv1;
};

// DebugMessage
struct MsgDebug {
  MsgCommon cmn;
  char      dbg_cmd;
};

// Message共用体
union MSG_REQ {
  MsgCommon common;
  MsgDebug  debug;
};

void prepare_task();
void main(void *params);
void send_req_msg(MSG_REQ *_msg);

enum AxisXYZ {
  AX_X,
  AX_Y,
  AX_Z,
  AX_XYZ_NUM,
};

enum AxisRPY {
  AX_ROLL,
  AX_PITCH,
  AX_YAW,
  AX_RPY_NUM,
};

struct st_imu {
  float fl_acc[AX_XYZ_NUM];
  float fl_gyro[AX_XYZ_NUM];
  float fl_mag[AX_XYZ_NUM];
  float fl_ang[AX_RPY_NUM];
  float fl_quat[4];
};

void get_now_imudata(st_imu &imu_d);

}; // namespace RSI

#endif