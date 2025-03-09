#include "i2c.h"
#include "wit_c_sdk.h"
#include <stdio.h>

#define TIMEOUT	3
#define RETRY	3

static int fd;
//char *i2c_dev = "/dev/i2c-1";



#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
static volatile char s_cDataUpdate = 0;

static void AutoScanSensor(void);
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
static int i2c_read(u8 addr, u8 reg, u8 *data, u32 len);
static int i2c_write(u8 addr, u8 reg, u8 *data, u32 len);

void getparam(float fAcc[3], float fGyro[3],float  fAngle[3]);