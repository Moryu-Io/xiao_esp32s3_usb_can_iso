#include <Arduino.h>
#include <message_buffer.h>

#include "wit_c_sdk.h"

// ローカル
#include "global_config.hpp"
#include "rs485_imu_task_main.hpp"

namespace RSI {

// RTOS メッセージ
MessageBufferHandle_t p_MsgBufReq;
MSG_REQ               msgReq;

#define ACC_UPDATE 0x01
#define GYRO_UPDATE 0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE 0x08
#define READ_UPDATE 0x80
static volatile char s_cDataUpdate        = 0;
const uint32_t       c_uiBaud[8]          = {4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800};
constexpr uint8_t    CU8_IMU_NO_CONNECTED = 0xFF; // 接続されてない時にU8_SEL_BAUDに格納される
uint8_t              U8_SEL_BAUD          = CU8_IMU_NO_CONNECTED;
HardwareSerial      *P_SERIAL_IMU         = &Serial2;

constexpr uint8_t CU8_IMU_DATA_PAGE_NUM               = 3;
st_imu            IMU_DATA_BUF[CU8_IMU_DATA_PAGE_NUM] = {};
uint8_t           U8_IMU_DATA_WRITE_PAGE              = 0;

static void init_imu();
static void update_imu();
static bool check_imu_connection(uint8_t u8_retry);
static void subproc_debugcmd(char ch_cmd);
static void process_message();

void AutoScanSensor(void);
void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
void Delayms(uint16_t ucMs);

void prepare_task() {
}
void main(void *params) {
  uint32_t loop_tick = (int)configTICK_RATE_HZ / LOOP_RATE_RS485_HZ;

  p_MsgBufReq = xMessageBufferCreate(3 * sizeof(MSG_REQ));

  P_SERIAL_IMU->begin(230400, SERIAL_8N1, PIN::RS485_RX, PIN::RS485_TX);

  WitInit(WIT_PROTOCOL_MODBUS, 0x50);
  WitSerialWriteRegister(SensorUartSend);
  WitRegisterCallBack(CopeSensorData);
  WitDelayMsRegister(Delayms);

  //AutoScanSensor();
  if(check_imu_connection(2)) {
    U8_SEL_BAUD = 6;
	DEBUG_PRINT_STR_RS485("success IMU default connection\n");
  } else {
    U8_SEL_BAUD = CU8_IMU_NO_CONNECTED;
	DEBUG_PRINT_STR_RS485("failure IMU default connection\n");
  }

  auto xLastWakeTime = xTaskGetTickCount();
  while(true) {
    vTaskDelayUntil(&xLastWakeTime, loop_tick);
    DEBUG_PRINT_PRC_START(DBG_PRC_ID::RS485_MAIN); // 処理時間計測開始

    /* Message処理 */
    process_message();

    if(U8_SEL_BAUD != CU8_IMU_NO_CONNECTED) {
      update_imu();
    } else {
    }

    DEBUG_PRINT_PRC_FINISH(DBG_PRC_ID::RS485_MAIN); // 処理時間計測停止
  }
}

void send_req_msg(MSG_REQ *_msg) {
  xMessageBufferSend(p_MsgBufReq, (void *)_msg, sizeof(MSG_REQ), 0);
}

void get_now_imudata(st_imu &imu_d) {
  int16_t s16_write_page = U8_IMU_DATA_WRITE_PAGE;
  uint8_t u8_read_page   = (s16_write_page - 1 < 0) ? CU8_IMU_DATA_PAGE_NUM - 1 : s16_write_page - 1;

  imu_d = IMU_DATA_BUF[u8_read_page];
}

static void init_imu() {
  if(check_imu_connection(1)) {
    U8_SEL_BAUD = 6;
  } else {
    U8_SEL_BAUD = CU8_IMU_NO_CONNECTED;
  }
}

static void update_imu() {
  WitReadReg(AX, 12);

  while(P_SERIAL_IMU->available()) {
    WitSerialDataIn(P_SERIAL_IMU->read());
  }
  if(s_cDataUpdate) {
    U8_IMU_DATA_WRITE_PAGE++;
    if(U8_IMU_DATA_WRITE_PAGE >= CU8_IMU_DATA_PAGE_NUM) {
      U8_IMU_DATA_WRITE_PAGE = 0;
    }
    if(s_cDataUpdate & ACC_UPDATE) {
      for(int i = 0; i < AxisXYZ::AX_XYZ_NUM; i++) {
        IMU_DATA_BUF[U8_IMU_DATA_WRITE_PAGE].fl_acc[i] = sReg[AX + i] / 32768.0f * 16.0f;
      }
      DEBUG_PRINT_RS485("acc:%0.3f, %0.3f, %0.3f\r\n",
                        IMU_DATA_BUF[U8_IMU_DATA_WRITE_PAGE].fl_acc[AxisXYZ::AX_X],
                        IMU_DATA_BUF[U8_IMU_DATA_WRITE_PAGE].fl_acc[AxisXYZ::AX_Y],
                        IMU_DATA_BUF[U8_IMU_DATA_WRITE_PAGE].fl_acc[AxisXYZ::AX_Z]);
      s_cDataUpdate &= ~ACC_UPDATE;
    }
    if(s_cDataUpdate & GYRO_UPDATE) {
      for(int i = 0; i < AxisXYZ::AX_XYZ_NUM; i++) {
        IMU_DATA_BUF[U8_IMU_DATA_WRITE_PAGE].fl_gyro[i] = sReg[GX + i] / 32768.0f * 2000.0f;
      }
      DEBUG_PRINT_RS485("gyr:%0.3f, %0.3f, %0.3f\r\n",
                        IMU_DATA_BUF[U8_IMU_DATA_WRITE_PAGE].fl_gyro[AxisXYZ::AX_X],
                        IMU_DATA_BUF[U8_IMU_DATA_WRITE_PAGE].fl_gyro[AxisXYZ::AX_Y],
                        IMU_DATA_BUF[U8_IMU_DATA_WRITE_PAGE].fl_gyro[AxisXYZ::AX_Z]);
      s_cDataUpdate &= ~GYRO_UPDATE;
    }
    if(s_cDataUpdate & ANGLE_UPDATE) {
      for(int i = 0; i < AxisRPY::AX_RPY_NUM; i++) {
        IMU_DATA_BUF[U8_IMU_DATA_WRITE_PAGE].fl_ang[i] = sReg[Roll + i] / 32768.0f * 180.0f;
      }
      DEBUG_PRINT_RS485("ang:%0.3f, %0.3f, %0.3f\r\n",
                        IMU_DATA_BUF[U8_IMU_DATA_WRITE_PAGE].fl_ang[AxisRPY::AX_ROLL],
                        IMU_DATA_BUF[U8_IMU_DATA_WRITE_PAGE].fl_ang[AxisRPY::AX_PITCH],
                        IMU_DATA_BUF[U8_IMU_DATA_WRITE_PAGE].fl_ang[AxisRPY::AX_YAW]);
      s_cDataUpdate &= ~ANGLE_UPDATE;
    }
    s_cDataUpdate = 0;
  }
}

static bool check_imu_connection(uint8_t u8_retry) {
  P_SERIAL_IMU->flush();
  s_cDataUpdate = 0;
  do {
    WitReadReg(AX, 3);
    delay(200);
    while(P_SERIAL_IMU->available()) {
      WitSerialDataIn(P_SERIAL_IMU->read());
    }
    if(s_cDataUpdate != 0) {
      DEBUG_PRINT_RS485("%d baud find sensor\r\n\r\n", P_SERIAL_IMU->baudRate());
      return true;
    }
    u8_retry--;
  } while(u8_retry);
  return false;
}

static void process_message() {
  if(xMessageBufferReceive(p_MsgBufReq, (void *)&msgReq, sizeof(MSG_REQ), 0) == sizeof(MSG_REQ)) {
    switch(msgReq.common.MsgId) {
    case MSG_ID::REQ_INIT:
      /* INIT指示 */
      init_imu();
      break;
    case MSG_ID::REQ_DEBUG:
      /* INIT指示 */
      subproc_debugcmd(msgReq.debug.dbg_cmd);
      break;
    default:
      break;
    }
  }
}

static void ShowHelp(void) {
  DEBUG_PRINT_STR_RS485("\r\n************************	 WIT_SDK_DEMO	************************");
  DEBUG_PRINT_STR_RS485("\r\n************************          HELP           ************************\r\n");
  DEBUG_PRINT_STR_RS485("UART SEND:a   Acceleration calibration.\r\n");
  DEBUG_PRINT_STR_RS485("UART SEND:m   Magnetic field calibration,After calibration send:   e\\r\\n   to indicate the end\r\n");
  DEBUG_PRINT_STR_RS485("UART SEND:U   Bandwidth increase.\r\n");
  DEBUG_PRINT_STR_RS485("UART SEND:u   Bandwidth reduction.\r\n");
  DEBUG_PRINT_STR_RS485("UART SEND:B   Baud rate increased to 115200.\r\n");
  DEBUG_PRINT_STR_RS485("UART SEND:b   Baud rate reduction to 9600.\r\n");
  DEBUG_PRINT_STR_RS485("UART SEND:t   AutoScanSensor.\r\n");
  DEBUG_PRINT_STR_RS485("UART SEND:h   help.\r\n");
  DEBUG_PRINT_STR_RS485("******************************************************************************\r\n");
}

static void subproc_debugcmd(char ch_cmd) {
  switch(ch_cmd) {
  case 'a':
    if(WitStartAccCali() != WIT_HAL_OK) DEBUG_PRINT_STR_RS485("\r\nSet AccCali Error\r\n");
    break;
  case 'm':
    if(WitStartMagCali() != WIT_HAL_OK) DEBUG_PRINT_STR_RS485("\r\nSet MagCali Error\r\n");
    break;
  case 'e':
    if(WitStopMagCali() != WIT_HAL_OK) DEBUG_PRINT_STR_RS485("\r\nSet MagCali Error\r\n");
    break;
  case 'u':
    if(WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) DEBUG_PRINT_STR_RS485("\r\nSet Bandwidth Error\r\n");
    break;
  case 'U':
    if(WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) DEBUG_PRINT_STR_RS485("\r\nSet Bandwidth Error\r\n");
    break;
  case 'B':
    if(WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) {
      DEBUG_PRINT_STR_RS485("\r\nSet Baud Error\r\n");
    } else {
      P_SERIAL_IMU->begin(c_uiBaud[WIT_BAUD_115200]);
      DEBUG_PRINT_STR_RS485(" 115200 Baud rate modified successfully\r\n");
    }
    break;
  case 'b':
    if(WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK) {
      DEBUG_PRINT_STR_RS485("\r\nSet Baud Error\r\n");
    } else {
      P_SERIAL_IMU->begin(c_uiBaud[WIT_BAUD_9600]);
      DEBUG_PRINT_STR_RS485(" 9600 Baud rate modified successfully\r\n");
    }
    break;
  case 't':
    AutoScanSensor();
    break;
  case 'h':
    ShowHelp();
    break;
  default: return;
  }
}

void AutoScanSensor(void) {
  for(int i = 0; i < sizeof(c_uiBaud) / sizeof(c_uiBaud[0]); i++) {
    P_SERIAL_IMU->begin(c_uiBaud[i], SERIAL_8N1, PIN::RS485_RX, PIN::RS485_TX);
    P_SERIAL_IMU->flush();
    int iRetry = 2;
    if(check_imu_connection(2)) {
      U8_SEL_BAUD = i;
      return;
    }
  }
  DEBUG_PRINT_STR_RS485("can not find sensor\r\n");
  DEBUG_PRINT_STR_RS485("please check your connection\r\n");
}

void SensorUartSend(uint8_t *p_data, uint32_t uiSize) {
  digitalWrite(PIN::RS485_RE, HIGH);
  P_SERIAL_IMU->write(p_data, uiSize);
  P_SERIAL_IMU->flush();
  digitalWrite(PIN::RS485_RE, LOW);
}

void Delayms(uint16_t ucMs) {
  delay(ucMs);
}

void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum) {
  int i;
  for(i = 0; i < uiRegNum; i++) {
    switch(uiReg) {
    case AZ:
      s_cDataUpdate |= ACC_UPDATE;
      break;
    case GZ:
      s_cDataUpdate |= GYRO_UPDATE;
      break;
    case HZ:
      s_cDataUpdate |= MAG_UPDATE;
      break;
    case Yaw:
      s_cDataUpdate |= ANGLE_UPDATE;
      break;
    default:
      s_cDataUpdate |= READ_UPDATE;
      break;
    }
    uiReg++;
  }
}

} // namespace RSI