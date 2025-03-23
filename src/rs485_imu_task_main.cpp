#include <Arduino.h>

#include "wit_c_sdk.h"

// ローカル
#include "global_config.hpp"
#include "rs485_imu_task_main.hpp"

namespace RSI{

    
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
const uint32_t c_uiBaud[8] = { 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800};
//const uint32_t c_uiBaud[8] = { 9600, 9600, 9600, 9600, 9600, 9600, 9600, 9600};

void AutoScanSensor(void);
void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
void Delayms(uint16_t ucMs);

void prepare_task(){

}
void main(void* params){
    uint32_t loop_tick = (int)configTICK_RATE_HZ / LOOP_RATE_RS485_HZ;

    Serial2.begin(230400, SERIAL_8N1, PIN::RS485_RX, PIN::RS485_TX);
	WitInit(WIT_PROTOCOL_MODBUS, 0x50);
	WitSerialWriteRegister(SensorUartSend);
	WitRegisterCallBack(CopeSensorData);
    WitDelayMsRegister(Delayms);
	DEBUG_PRINT_STR_RS485("\r\n********************** wit-motion modbus example  ************************\r\n");
	//uint8_t test[2] = {0x55, 0x55};
	//while(1){
	//	SensorUartSend(test, 2);
	//	delay(1);
	//}
	AutoScanSensor();

    auto xLastWakeTime = xTaskGetTickCount();
    while (true) {
      vTaskDelayUntil(&xLastWakeTime, loop_tick);
      DEBUG_PRINT_PRC_START(DBG_PRC_ID::RS485_MAIN);  // 処理時間計測開始

      WitReadReg(AX, 12);
      delay(500);
      while (Serial2.available())
      {
        WitSerialDataIn(Serial2.read());
      }
      //while (Serial.available()) 
      //{
      //  CopeCmdData(Serial.read());
      //}
      //CmdProcess();
      if(s_cDataUpdate)
      {
        float fAcc[3], fGyro[3], fAngle[3];
            for(int i = 0; i < 3; i++)
            {
                fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
                fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
                fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
            }
            if(s_cDataUpdate & ACC_UPDATE)
            {
                DEBUG_PRINT_RS485("acc:%0.3f, %0.3f, %0.3f\r\n", fAcc[0], fAcc[1], fAcc[2]);
                s_cDataUpdate &= ~ACC_UPDATE;
            }
            if(s_cDataUpdate & GYRO_UPDATE)
            {
                DEBUG_PRINT_RS485("gyr:%0.3f, %0.3f, %0.3f\r\n", fGyro[0], fGyro[1], fGyro[2]);
                s_cDataUpdate &= ~GYRO_UPDATE;
            }
            if(s_cDataUpdate & ANGLE_UPDATE)
            {
                DEBUG_PRINT_RS485("ang:%0.3f, %0.3f, %0.3f\r\n", fAngle[0], fAngle[1], fAngle[2]);
                s_cDataUpdate &= ~ANGLE_UPDATE;
            }
            if(s_cDataUpdate & MAG_UPDATE)
            {
                DEBUG_PRINT_RS485("mag:%0.3f, %0.3f, %0.3f\r\n", sReg[HX], sReg[HY], sReg[HZ]);
                s_cDataUpdate &= ~MAG_UPDATE;
            }
        s_cDataUpdate = 0;
        }

      DEBUG_PRINT_PRC_FINISH(DBG_PRC_ID::RS485_MAIN); // 処理時間計測停止
    }
}



void CopeCmdData(unsigned char ucData)
{
	static unsigned char s_ucData[50], s_ucRxCnt = 0;
	
	s_ucData[s_ucRxCnt++] = ucData;
	if(s_ucRxCnt<3)return;										//Less than three data returned
	if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
	if(s_ucRxCnt >= 3)
	{
		if((s_ucData[1] == '\r') && (s_ucData[2] == '\n'))
		{
			s_cCmd = s_ucData[0];
			memset(s_ucData,0,50);
			s_ucRxCnt = 0;
		}
		else 
		{
			s_ucData[0] = s_ucData[1];
			s_ucData[1] = s_ucData[2];
			s_ucRxCnt = 2;
			
		}
	}
}
void ShowHelp(void)
{
	DEBUG_PRINT_STR_RS485("\r\n************************	 WIT_SDK_DEMO	************************");
	DEBUG_PRINT_STR_RS485("\r\n************************          HELP           ************************\r\n");
	DEBUG_PRINT_STR_RS485("UART SEND:a\\r\\n   Acceleration calibration.\r\n");
	DEBUG_PRINT_STR_RS485("UART SEND:m\\r\\n   Magnetic field calibration,After calibration send:   e\\r\\n   to indicate the end\r\n");
	DEBUG_PRINT_STR_RS485("UART SEND:U\\r\\n   Bandwidth increase.\r\n");
	DEBUG_PRINT_STR_RS485("UART SEND:u\\r\\n   Bandwidth reduction.\r\n");
	DEBUG_PRINT_STR_RS485("UART SEND:B\\r\\n   Baud rate increased to 115200.\r\n");
	DEBUG_PRINT_STR_RS485("UART SEND:b\\r\\n   Baud rate reduction to 9600.\r\n");
	DEBUG_PRINT_STR_RS485("UART SEND:h\\r\\n   help.\r\n");
	DEBUG_PRINT_STR_RS485("******************************************************************************\r\n");
}

void CmdProcess(void)
{
	switch(s_cCmd)
	{
		case 'a':	if(WitStartAccCali() != WIT_HAL_OK) DEBUG_PRINT_STR_RS485("\r\nSet AccCali Error\r\n");
			break;
		case 'm':	if(WitStartMagCali() != WIT_HAL_OK) DEBUG_PRINT_STR_RS485("\r\nSet MagCali Error\r\n");
			break;
		case 'e':	if(WitStopMagCali() != WIT_HAL_OK) DEBUG_PRINT_STR_RS485("\r\nSet MagCali Error\r\n");
			break;
		case 'u':	if(WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) DEBUG_PRINT_STR_RS485("\r\nSet Bandwidth Error\r\n");
			break;
		case 'U':	if(WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) DEBUG_PRINT_STR_RS485("\r\nSet Bandwidth Error\r\n");
			break;
		case 'B':	if(WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) DEBUG_PRINT_STR_RS485("\r\nSet Baud Error\r\n");
              else 
              {
                 Serial2.begin(c_uiBaud[WIT_BAUD_115200]);
                 DEBUG_PRINT_STR_RS485(" 115200 Baud rate modified successfully\r\n");
              }
			break;
		case 'b':	if(WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK) DEBUG_PRINT_STR_RS485("\r\nSet Baud Error\r\n");
              else 
              {
                Serial2.begin(c_uiBaud[WIT_BAUD_9600]);
                DEBUG_PRINT_STR_RS485(" 9600 Baud rate modified successfully\r\n");
              }
			break;
		case 'h':	ShowHelp();
			break;
		default :return;
	}
	s_cCmd = 0xff;
}
void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
  digitalWrite(PIN::RS485_RE, HIGH);
  Serial2.write(p_data, uiSize);
  Serial2.flush();
  digitalWrite(PIN::RS485_RE, LOW);
}

void Delayms(uint16_t ucMs)
{
  delay(ucMs);
}

void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
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

void AutoScanSensor(void)
{
	int i, iRetry;
	
	for(i = 0; i < sizeof(c_uiBaud)/sizeof(c_uiBaud[0]); i++)
	{
		Serial2.begin(c_uiBaud[i], SERIAL_8N1, PIN::RS485_RX, PIN::RS485_TX);
        Serial2.flush();
		iRetry = 2;
		s_cDataUpdate = 0;
		do
		{
			WitReadReg(AX, 3);
			delay(200);
			while (Serial2.available())
			{
				WitSerialDataIn(Serial2.read());
			}
			if(s_cDataUpdate != 0)
			{
				DEBUG_PRINT_RS485("%d baud find sensor\r\n\r\n",c_uiBaud[i]);
				ShowHelp();
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
	DEBUG_PRINT_STR_RS485("can not find sensor\r\n");
	DEBUG_PRINT_STR_RS485("please check your connection\r\n");
}


}