#include "getparam.h"

void getparam(float fAcc[3], float fGyro[3], float fAngle[3]){
	int i;
	char *argv="/dev/i2c-1";
	/*if(argc < 2)
	{
		printf("please input dev name\n");
		return 0;
	}
	
	;
	int i;*/
	fd = i2c_open(argv, 3, 3);
	if( fd < 0)printf("open %s fail\n", argv);
	else printf("open %s success\n", argv);
	
	WitInit(WIT_PROTOCOL_I2C, 0x50);
	WitI2cFuncRegister(i2c_write, i2c_read);
	WitRegisterCallBack(CopeSensorData);
	WitDelayMsRegister(Delayms);
	printf("\r\n********************** wit-motion IIC example  ************************\r\n");
	AutoScanSensor();
	
		WitReadReg(AX, 12);
		//usleep(500000);
		if(s_cDataUpdate)
		{
			//printf("\r\n");
			for(i = 0; i < 3; i++)
			{
				fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
				fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
				fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
			}
			if(s_cDataUpdate & ACC_UPDATE)
			{
				
				s_cDataUpdate &= ~ACC_UPDATE;
			}
			if(s_cDataUpdate & GYRO_UPDATE)
			{
				
				s_cDataUpdate &= ~GYRO_UPDATE;
			}
			if(s_cDataUpdate & ANGLE_UPDATE)
			{
				
				s_cDataUpdate &= ~ANGLE_UPDATE;
			}
			if(s_cDataUpdate & MAG_UPDATE)
			{
				
				s_cDataUpdate &= ~MAG_UPDATE;
			}
		}
	
	
	
		
	//close(fd);
}

static int i2c_read(u8 addr, u8 reg, u8 *data, u32 len)
{
	if(i2c_read_data(addr>>1, reg, data, len) < 0)return 0;
	return 1;
}
static int i2c_write(u8 addr, u8 reg, u8 *data, u32 len)
{
	if(i2c_write_data(addr>>1, reg, data, len) < 0)return 0;
	return 1;
}
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
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

static void Delayms(uint16_t ucMs)
{
	usleep(ucMs*1000);
}

static void AutoScanSensor(void)
{
	int i, iRetry;
	
	for(i = 0; i < 0x7F; i++)
	{
		WitInit(WIT_PROTOCOL_I2C, i);
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			usleep(5);
			if(s_cDataUpdate != 0)
			{
				printf("find %02X addr sensor\r\n", i);
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");
}

