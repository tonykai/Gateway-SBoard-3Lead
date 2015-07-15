/*===================================Library==================================*/
#include "stm32f401_discovery.h"
#include "stm32f4xx.h"
#include "stdio.h"
#include "math.h"
#include "defines.h"

#include "tm_stm32f4_usart.h"
#include "tm_stm32f4_adc.h"
#include "tm_stm32f4_usb_vcp.h"
#include "misc.h"
#include "ff.h"
#include "diskio.h"
#include "SPI_MSD0_Driver.h"
/*=============================================================================*/

/*====================================Define===================================*/
#define NULL 0
#define BUFLN  128	/* must be a power of 2, see ltsamp() */
#define EYECLS 0.25
#define MaxQRSw 0.13
#define v1Rawdata_N 128
#define v2Rawdata_N 128
#define v3Rawdata_N 128
#define HRV_N	800
#define NN50_Pcount 6 /* 50ms / (1000ms/sample rate) */

FATFS fs;           
FRESULT res;                 
DIR dirs;
FIL file;
FILINFO finfo;
UINT br;
unsigned char buffer[2000];
unsigned char ECGv1RawData[1024];
unsigned char ECGv2RawData[1024];
unsigned char ECGv3RawData[1024];
char spfline[30];
unsigned int ECGv1Data;
unsigned int ECGv2Data;
unsigned int ECGv3Data;
unsigned int TempData;
unsigned int GsensorX;
unsigned int GsensorY;
unsigned int GsensorZ;
const int gain = 200;
const int sps = 128;
const int PWFreq = 60;
int LPn = 2;				//(int)sps/PWFreq;
int LP2n =4;				//(int)2*sps/PWFreq;
double lfsc = 390.625;	//(double)1.25*gain*gain/sps;
int LTwindow = 16;			//(int)sps*MaxQRSw;
int ebuf[BUFLN]={0};
int lbuf[BUFLN]={0};
int HR_count=0;
int HR_Value=60;
int ECGv1_Rawdata[v1Rawdata_N]={0};
int ECGv2_Rawdata[v2Rawdata_N]={0};
int ECGv3_Rawdata[v3Rawdata_N]={0};
unsigned int ADC_Value[5]={0,0,0,0,0};
int count_t;
static long int v1LT_count=0;
static long int v2LT_count=0;
static long int v3LT_count=0;
int v1QRS_result;
int v2QRS_result;
int v3QRS_result;
int TempCount = 0;
int StateCounter=128;
int HRV_Count=0;

unsigned char Gdata_X_H,Gdata_X_L;
unsigned char Gdata_Y_H,Gdata_Y_L;
unsigned char Gdata_Z_H,Gdata_Z_L;

unsigned int G_data_All_X;
unsigned int G_data_All_Y;
unsigned int G_data_All_Z;


static void TIM_Config(uint16_t period,uint16_t prec);
static void Delay(__IO uint32_t nCount);
int qrs_detect(long int t_count,int *inputdata);
int ltsamp(long int t,int *inputdata);
int getsample(long int tt,int *inputdata);

uint16_t HRV_Data[HRV_N];
uint16_t HRV_SDNN; 
uint16_t HRV_RMSSD;
uint16_t HRV_NN50;
uint16_t HRV_PNN50;
static uint16_t HRV_Store_PT =0;

void Gsensor_Write(unsigned char REG_Address,unsigned char REG_data);
/*==============================================================================*/



/*=================================GsensorDefine================================*/
#define	SlaveAddress 	0x32
#define WHO_AM_I		0x0F
#define CTRL_REG1		0x20
#define CTRL_REG2		0x21
#define CTRL_REG3		0x22
#define CTRL_REG4		0x23
#define CTRL_REG5		0x24
#define CTRL_REG1_Data  0X2F

#define REFERENCE		0x26
#define STATUS_REG		0x27
#define OUT_X_ALL		0x28
#define OUT_X_L			0x28
#define OUT_X_H			0x29
#define OUT_Y_ALL		0x2A
#define OUT_Y_L			0x2A
#define OUT_Y_H			0x2B
#define OUT_Z_ALL		0x2C
#define OUT_Z_L			0x2C
#define OUT_Z_H			0x2D

#define INT1_CFG		0x30
#define INT1_SOURCE		0x31
#define INT1_THS		0x32
#define INT1_DURATION	0x33
#define INT2_CFG		0x34
#define INT2_SOURCE		0x35
#define INT2_THS		0x36
#define INT2_DURATION	0x37
/*==============================================================================*/

/*===================================SendData===================================*/
//void BT_Send_ECG(void){
//	unsigned char ECGLow;
//	unsigned char ECGHigh;
//	
//	ECGData=TM_ADC_Read(ADC1,ADC_Channel_3);
//	ECGLow = ECGData&0x00ff;
//	ECGHigh = (ECGData>>8)&0x00ff;
//	
//	TM_USART_Putc(USART2, ECGLow);
//	TM_USART_Putc(USART2, ECGHigh);
//}
void BT_Send_Temp(void){
	unsigned char TempLow;
	unsigned char TempHigh;
	
	TempData=TM_ADC_Read(ADC1,ADC_Channel_12);
	TempLow = TempData&0x00ff;
	TempHigh = (TempData>>8)&0x00ff;
	
	TM_USART_Putc(USART2, TempLow);  
	TM_USART_Putc(USART2, TempHigh);
}
/*==============================================================================*/


/*===================================I2C(I2C1)==================================*/
void I2C_GPIO_Init(void){
	//��l��GPIO��pin�}
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP; 
	
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);   //PB6:SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);   //PB7:SDA
	
	I2C_InitStruct.I2C_Mode=I2C_Mode_SMBusHost;
	I2C_InitStruct.I2C_DutyCycle=I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = 0X00;
	I2C_InitStruct.I2C_Ack=I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_ClockSpeed=300000;
	I2C_Init(I2C1, &I2C_InitStruct);
	
	I2C_Cmd(I2C1,ENABLE);
}
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction){
		
		/*
			�o�e�]�Ʀa�}+R/W bit
			
			Parameter:
			I2Cx-->�ϥΪ�I2C
			address-->7 bits address
			direction-->transmitter/receiver
			
		*/
		
    // ����I2C1�������ɭ�
    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
 
    //��I2C1�o�e�ҰʫH��
    I2C_GenerateSTART(I2Cx, ENABLE);
 
    // ����I2C1�q�]�ƽT�{�ҰʫH��
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
 
    // Send slave Address for write 
    I2C_Send7bitAddress(I2Cx, address, direction);
		
		//I2C_Direction_Tranmitter:�D�o�e�Ҧ�
		//I2C_Direction_Tranmitter:�D�����Ҧ�
		if(direction == I2C_Direction_Transmitter){
        while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    } else if(direction == I2C_Direction_Receiver){
        while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    }
}
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data){
	
		/*
			�o�e1byte��SlaveDevice
			
			Parameter:
			I2Cx-->�ϥΪ�I2C
			data-->�n�o�e�����
			
		*/
		
    I2C_SendData(I2Cx, data);
    // ����I2C1�o�e�@��byte
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx){
		
		/*
			�qSlaveDeviceŪ���@��byte�ýШD�᭱�@��byte
			
			Parameter:
			I2Cx-->�ϥΪ�I2C
			
		*/
	
		uint8_t data;
    // �ҥΨýT�{�]�w�������
    I2C_AcknowledgeConfig(I2Cx, ENABLE);
    // ���ݱ�����@��byte
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
    // �qI2C�Ȧs��Ū���@��byte�æ^��
    data = I2C_ReceiveData(I2Cx);
    return data;
}
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx){
		
		/*
			�qSlaveDeviceŪ���@��byte�����ШD�᭱�@��byte
			
			Parameter:
			I2Cx-->�ϥΪ�I2C
			
		*/
	
		uint8_t data;
    // disabe acknowledge of received data
    // nack also generates stop condition after last byte received
    // see reference manual for more info
    I2C_AcknowledgeConfig(I2Cx, DISABLE);
    I2C_GenerateSTOP(I2Cx, ENABLE);
    // ���ݱ�����@��byte
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
    // �qI2C�Ȧs��Ū���@��byte�æ^��
    data = I2C_ReceiveData(I2Cx);
    return data;
}
void I2C_stop(I2C_TypeDef* I2Cx){
    
		/*
			�o�e����T��������I2C�׬y��
			
			Parameter:
			I2Cx-->�ϥΪ�I2C
			
		*/
	
		// ����I2C1
    I2C_GenerateSTOP(I2Cx, ENABLE);
}
void writeReg(int reg, int value){
    
		/*
			�N�ȼg�J�Ȧs����
			
			Parameter:
			reg-->�Ȧs���a�}
			value-->�n�g�J�Ȧs������
	
		*/
	
		I2C_start(I2C1, SlaveAddress, I2C_Direction_Transmitter); // �btransmitter�Ҧ��U�ǳƶ}�l�o�e
    I2C_write(I2C1, reg); // ���w�n�g�J���Ȧs��
    I2C_write(I2C1, value); // �g�J�Ȧs������
    I2C_stop(I2C1); // ����o�e
}
int readReg(int reg){
    
		/*
			Ū���Ȧs��
			
			Parameter:
			reg-->�Ȧs���a�}
			
		*/
	
		int value;
    
    I2C_start(I2C1, SlaveAddress, I2C_Direction_Transmitter); // �btransmitter�Ҧ��U�ǳƶ}�l�o�e
    I2C_write(I2C1, reg); // ���w�n�g�J���Ȧs��  
    I2C_stop(I2C1); // �g�J�Ȧs������
       
    I2C_start(I2C1, SlaveAddress, I2C_Direction_Transmitter); // �btransmitter�Ҧ��U�ǳƶ}�l�o�e
    value = I2C_read_ack(I2C1); // �qLIS331DLHŪ���@��byte��ýШD�᭱�@��byte
    I2C_read_nack(I2C1); // Ū���@��byte��N���bŪ���U�@��byte(����ǿ�)
    
    return value;
}
void Gsensor_Init(void){
	I2C_GPIO_Init(); //����l��I2C��GPIO
	writeReg(CTRL_REG1, 0x2F); //�bREG1���g�J0x2F(00101111)�A�}��REG1(�ԲӽЬ�LIS331DLH datasheet)
}
void Gsensor_readValue(){
		I2C_start(I2C1, SlaveAddress, I2C_Direction_Transmitter); // �btransmitter�Ҧ��U�ǳƶ}�l�o�e
    I2C_write(I2C1, 0x28 | (1 << 7)); // �q0x28�}�lŪ���T�b�[�t�W��high�ȥH��low�Ȧ@���ӭȡA�h�d�@�ӭ��קK�ֶ�
    I2C_stop(I2C1); // ����o�e
    
    I2C_start(I2C1, SlaveAddress, I2C_Direction_Receiver); // �breceiver�Ҧ��U�ǳƱ���
    
    Gdata_X_L = I2C_read_ack(I2C1); // Ū��0x28�Ax�b��Low byte�A�ín�D�U�@��byte
    Gdata_X_H = I2C_read_ack(I2C1); // Ū��0x29�Ax�b��High byte�A�ín�D�U�@��byte
    Gdata_Y_L = I2C_read_ack(I2C1); // Ū��0x2A�Ay�b��Low byte�A�ín�D�U�@��byte
    Gdata_Y_H = I2C_read_ack(I2C1); // Ū��0x2B�Ay�b��High byte�A�ín�D�U�@��byte
    Gdata_Z_L = I2C_read_ack(I2C1); // Ū��0x2C�Az�b��Low byte�A�ín�D�U�@��byte
    Gdata_Z_H = I2C_read_ack(I2C1); // Ū��0x2D�Az�b��High byte�A�ín�D�U�@��byte
        
    I2C_read_nack(I2C1); // Ū��0x2E�A���O�d���w�]�ȡA�����n�D�U�@��byte
	
		//��high byte��low byte�[�_��
		G_data_All_X=(short)((Gdata_X_H<<8)+Gdata_X_L);
		G_data_All_Y=(short)((Gdata_Y_H<<8)+Gdata_Y_L);
		G_data_All_Z=(short)((Gdata_Z_H<<8)+Gdata_Z_L);
}
/*==============================================================================*/

/*===============================BlueTooth(USART2)==============================*/
void BT_GPIO_Init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	
  // GPIOD Periph clock enable 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  // Configure PD12, PD13, PD14 and PD15 in output pushpull mode 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 ;   //Pin0: Power   Pin1:RST   Pin2:Pairing   Pin3:SW_BTN   Pin4:Status
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   //setting out 
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);   //Initial GPIO
}
void BT_RST(int st){
 if(st==1)
	 GPIO_SetBits(GPIOD,GPIO_Pin_1);
 else
	 GPIO_ResetBits(GPIOD,GPIO_Pin_1);
}
void BT_Pairing(int st){
 if(st==1)
	 GPIO_SetBits(GPIOD,GPIO_Pin_2);
 else
	 GPIO_ResetBits(GPIOD,GPIO_Pin_2);
}

void BT_BTN(int st){
 if(st==1)
	 GPIO_SetBits(GPIOD,GPIO_Pin_3);
 else
	 GPIO_ResetBits(GPIOD,GPIO_Pin_3);
}

void BT_Status(int st){
 if(st==1)
	 GPIO_SetBits(GPIOD,GPIO_Pin_4);
 else
	 GPIO_ResetBits(GPIOD,GPIO_Pin_4);
}

void BT_USART_Init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	//�ҥ�USART2��RCC����
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 , ENABLE);
	
	//�ҥ�GPIOD��RCC����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD , ENABLE);
	
	//�NPD5�MPD6�s����USART2
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2);
	
	//�]�wPD5��Tx��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	//GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_USART1);
	
	//�]�wPD6��Rx��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	//USART�򥻰ѼƳ]�w
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_Init(USART2,&USART_InitStructure);
	
	//USART�ҥ�
	USART_Cmd(USART2,ENABLE);
	
	//USART_Puts(USART1, "Init complete!Hello World");
}
void BT_Start(void){
	BT_GPIO_Init();     //Initial GPIO
	
	BT_BTN(1);          //Give SE_BTN 'High' as open�A1 as high 
	Delay(500);
	BT_RST(1);          //Give RST 'High' as open�A1 as high 
	Delay(1000);
	BT_Pairing(0);      //Give Pairing 'Low' as close�A0 as low
	Delay(1000);
	BT_Pairing(1);      //Give Pairing 'High' as open�A1 as high
	
	BT_USART_Init();    //Initial USART
}

int fputc(int ch, FILE *f) {
    /* Do your stuff here */
    /* Send your custom byte */
    /* Send byte to USART */
    TM_USART_Putc(USART2, ch);
    
    /* If everything is OK, you have to return character written */
    return ch;
    /* If character is not correct, you can return EOF (-1) to stop writing */
    return -1;
}
/*==============================================================================*/

/* We need to implement own __FILE struct */
/* FILE struct is used from __FILE */
/* You need this if you want use printf */
/* Struct FILE is implemented in stdio.h */
//FILE __stdout;



/*===============================SDCard Function==================================*/
int Status;
char buff_d[512]={0};

struct __FILE {
    int dummy;
};
//void SDCard_Write(void){
////	int i;
////	for(i=0;i<8;i++)
////	{
////		temp[i]=i;
////	}
//	MSD0_SPI_Configuration();
//	Status=MSD0_Init();
//		
//	res=f_mount(0, &fs);
//	//Open file.If file exists ,opened;file doesn't exists ,create new file 
//	res = f_open(&file, "0:/RawData.txt",FA_OPEN_ALWAYS|FA_READ|FA_WRITE );
////	if(res!=FR_OK) { printf("f_open() fail \r\n"); }
////	else { printf("f_open() success \r\n"); }
//	 
//	//Write file
//	res = f_lseek(&file, file.fsize); 
//	br = f_write(&file,&ECGRawData,256,&br);
////	br = f_printf(&file,"%c",1);
//	//br = f_puts("1234567890\n", &file) ;  //Write "1234567890" into string
////	if(br<1) { printf("f_puts() fail \r\n"); }
////	else
//// { 
//// printf("f_puts() success \r\n");
//// }
//	
////	res = f_lseek(&file, 2);   
////	br = f_puts("--", &file) ;  //Write "--" into string
////	if(br<1) { printf("f_puts() fail \r\n"); }
////	else { printf("f_puts() success \r\n"); }
//	
//	//Read file
////	br = file.fsize;
////	printf("file size:%d\r\n",br);
////	res = f_read(&file, buffer, file.fsize, &br);     //Read a char repeatly until end of file
////	if(res == FR_OK ) { printf("text:%s\r\n", buffer); }
////	else { printf(" f_read() fail \r\n"); }

//	//close file
//	f_close(&file);
//}



/*================================================================================*/



/*===============================Temperature Function=================================*/
void Temp_Init(void){
		GPIO_InitTypeDef GPIO_InitStructure;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
		GPIO_SetBits(GPIOC,GPIO_Pin_3);
}
/*================================================================================*/



/*===============================ECG_ADC Function=================================*/
void ECG_ADC_U9_Init(void){ 
    GPIO_InitTypeDef GPIO_InitStructure; 

		/* �ҥ�C4_SDN�� RCC ���� */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //�ϥ�C4_SDN�}���X
    
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
		GPIO_SetBits(GPIOC,GPIO_Pin_0);
} 
void ECG_ADC_U8_Init(void){ 
    GPIO_InitTypeDef GPIO_InitStructure; 

		/* �ҥ�C3_SDN�� RCC ���� */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //�ϥ�C3_SDN�}���X
    
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
		GPIO_SetBits(GPIOC,GPIO_Pin_10);
} 
void ECG_ADC_U7_Init(void){ 
    GPIO_InitTypeDef GPIO_InitStructure; 

		/* �ҥ�C2_SDN�� RCC ���� */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //�ϥ�C2_SDN�}���X
    
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
		GPIO_SetBits(GPIOA,GPIO_Pin_6);
} 
/*================================================================================*/



/*=================================QRS_Detection==================================*/
int qrs_detect(long int t_count,int *inputdata){
	static int jump_state=0;
	static int jump_count=0;
	static int L_state=0;
//	static int LT_data=0;
	static long int T0=0;
	static int T1=0;
	static int Ta=0;
//	static int T1,Ta;
	static int LT_data=0;

	switch(L_state){
		case 0:
		{
			T0 +=ltsamp(t_count,inputdata);
			//360Hz-1440 128Hz-512
			if(t_count >= (512-1) ) {
				L_state=1;
				T0 = (long int)(T0/512);
				T1=(int)2*T0;
				Ta=3*T0;
			}
			break;
		}
		case 1:
		{
			LT_data=ltsamp(t_count,inputdata);
			if(LT_data>T1) {
				Ta +=(LT_data-Ta)/10;
				T1=Ta/3;
			}
			//360Hz-2880 128Hz-1024
			if(t_count>=(1024-1)){
			L_state=2;
			}
			break;
		}
		case 2:
		{
			LT_data=ltsamp(t_count,inputdata);
			if(jump_state==0){
				if(LT_data>T1) {
//					tqrs=t_count-4;	//360Hz-12 128Hz-4
					jump_state=1;
					jump_count=0;
					return 1;
				}
			}
			else {
				jump_count++;
				//360Hz-80 128Hz-40
				if(jump_count>=40) {
				jump_count=0;
				jump_state=0;
				}
				if(LT_data>T1) {
					Ta +=(LT_data-Ta)/10;
					T1=Ta/3;
				}
			}
			break;
		}
		default: L_state=0;break;
		}
	return 0;
}


int ltsamp(long int t,int *inputdata){
	int dy;
	static long int tt = -1;
	static int Yn, Yn1, Yn2;
	static int buf_state=0;

//	if (lbuf == NULL)
//	{
//		lbuf = (int *)malloc((unsigned)BUFLN * sizeof(int));
//		ebuf = (int *)malloc((unsigned)BUFLN * sizeof(int));
		if (buf_state==0)
		{
			for (ebuf[0] = (int)sqrt(lfsc), tt = 1L; tt < BUFLN; tt++)
				ebuf[tt] = ebuf[0];
			if (t > BUFLN) tt = (t - BUFLN);
			else tt = -1;
			Yn = Yn1 = Yn2 = 0;
			buf_state=1;
		}
//		else memory_error();
//
//		if(t<tt-BUFLN) memory_error();
//	}

	while (t > tt) 
	{
		int v0=0,v1=0,v2=0;
		static int aet = 0,et;

		Yn2 = Yn1;
		Yn1 = Yn;
		v0 = getsample(tt,inputdata);
		v1 = getsample(tt-LPn,inputdata);
		v2 = getsample(tt-LP2n,inputdata);
//		Yn = getsample(tt);by tg
//		Yn1 = getsample(tt-1);by tg
		Yn = 2*Yn1 - Yn2 + v0 - 2*v1 + v2;
		dy = (Yn - Yn1) / LP2n;		/* lowpass derivative of input */
//		dy = (Yn - Yn1);		/* lowpass derivative of input */ by tg
		et = ebuf[(++tt)&(BUFLN-1)] = (int)sqrt(lfsc +dy*dy); /* length transform */
		lbuf[(tt)&(BUFLN-1)] = aet += et - ebuf[(tt-LTwindow)&(BUFLN-1)];
	}
	return lbuf[t&(BUFLN-1)];
}
int getsample(long int tt,int *inputdata){
	if(tt<0)
		tt=0;
	return inputdata[tt&(v1Rawdata_N-1)];
//	return input_testdata[tt&(INBUF)];//????
}
/*================================================================================*/

int main(void){	
	TM_USB_VCP_Init(); //USB VCP Initialize
	BT_Start(); //Enable BlueTooth(USART2)
	Gsensor_Init(); //Enable Gsensor(I2C)
	ECG_ADC_U9_Init(); //Enable ECG Module U9
	ECG_ADC_U8_Init(); //Enable ECG Module U8
	ECG_ADC_U7_Init(); //Enable ECG Module U7
	TM_ADC_Init(ADC1,ADC_Channel_3); //Enable ADC1_Channel_3,ECGv1
	TM_ADC_Init(ADC1,ADC_Channel_2); //Enable ADC1_Channel_2,ECGv2
	TM_ADC_Init(ADC1,ADC_Channel_1); //Enable ADC1_Channel_1,ECGv3
	TM_ADC_Init(ADC1,ADC_Channel_13); //Enable ADC1_Channel_12,Temperature
	TIM_Config(525 - 1 , 1250 - 1); //400Hz,Clock=84MHz, (84000000/period)/prescaler=Freq
	
	while(1){
		
	}
	
}



/*===================================Interrupt==================================*/
//Timer Setting
void TIM_Config(uint16_t period,uint16_t prec){
  NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
  // TIM3 clock enable 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  // Enable the TIM3 gloabal Interrupt 
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;   //���_�ɰ��檺�禡
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //�u������
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //���u������
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = period;     //�۰ʭ��˭�

  TIM_TimeBaseStructure.TIM_Prescaler = prec;    //Timer���W
 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //�V�W�p�ƼҦ�
	
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //��l��Timer
	
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); //���\��s���_
   
  TIM_Cmd(TIM3, ENABLE); //Enable Timer
}
//�w�ɾ�3���_�禡
void TIM3_IRQHandler(){
	unsigned char ECGv1Low;
	unsigned char ECGv1High;
	unsigned char ECGv2Low;
	unsigned char ECGv2High;
	unsigned char ECGv3Low;
	unsigned char ECGv3High;
	unsigned char TempLow;
	unsigned char TempHigh;
	unsigned char GsensorXLow;
	unsigned char GsensorXHigh;
	unsigned char GsensorYLow;
	unsigned char GsensorYHigh;
	unsigned char GsensorZLow;
	unsigned char GsensorZHigh;
	
	
	if(TIM_GetITStatus(TIM3 , TIM_IT_Update) == SET) //�T�{�O�_�����椤�_
	{
		Gsensor_readValue();
		ECGv1Data = TM_ADC_Read(ADC1,ADC_Channel_3); //�]�wData���Ȭ�ADC1��Channel3�ഫ�X�Ӫ���,U9
		ECGv2Data = TM_ADC_Read(ADC1,ADC_Channel_2); //�]�wData���Ȭ�ADC1��Channel2�ഫ�X�Ӫ���,U8
		ECGv3Data = TM_ADC_Read(ADC1,ADC_Channel_1); //�]�wData���Ȭ�ADC1��Channel1�ഫ�X�Ӫ���,U7
//		ECGRawData[128-StateCounter]=ECGData;
//		SDCard_Write();
	
		
		if(TempCount==0)
		{
			TempData = TM_ADC_Read(ADC1,ADC_Channel_13);
			TempCount=128;
		}
		
		
		//�վ�T�b�[�t�W���Ȩ�{���i�H��ܪ��d��
		GsensorX = ((G_data_All_X)+32768)>>4;
		GsensorY = ((G_data_All_Y)+32768)>>4;
		GsensorZ = ((G_data_All_Z)+32768)>>4;
		
		ECGv1Low = ECGv1Data&0x00ff;   //Low�Ȩ�Data���C8bits
		ECGv1High = (ECGv1Data>>8)&0x00ff; //High�ȥ��NData Shift 8bits��A���ȡA���L�]��ADC��12bits�B�C8bits�w�g�ϥιL�A�ҥHHigh�ȥu��12-8=4bits
		ECGv2Low = ECGv2Data&0x00ff;   //Low�Ȩ�Data���C8bits
		ECGv2High = (ECGv2Data>>8)&0x00ff; //High�ȥ��NData Shift 8bits��A���ȡA���L�]��ADC��12bits�B�C8bits�w�g�ϥιL�A�ҥHHigh�ȥu��12-8=4bits
		ECGv3Low = ECGv3Data&0x00ff;   //Low�Ȩ�Data���C8bits
		ECGv3High = (ECGv3Data>>8)&0x00ff; //High�ȥ��NData Shift 8bits��A���ȡA���L�]��ADC��12bits�B�C8bits�w�g�ϥιL�A�ҥHHigh�ȥu��12-8=4bits
		TempLow = TempData&0x00ff;
		TempHigh = (TempData>>8)&0x00ff;
		GsensorXLow = GsensorX&0x00ff;
		GsensorXHigh = (GsensorX>>8)&0x00ff;
		GsensorYLow = GsensorY&0x00ff;
		GsensorYHigh = (GsensorY>>8)&0x00ff;
		GsensorZLow = GsensorZ&0x00ff;
		GsensorZHigh = (GsensorZ>>8)&0x00ff;
		
		ECGv1_Rawdata[v1LT_count&(v1Rawdata_N-1)]=ECGv1Data-2048;
		ECGv2_Rawdata[v2LT_count&(v2Rawdata_N-1)]=ECGv2Data-2048;
		ECGv3_Rawdata[v3LT_count&(v3Rawdata_N-1)]=ECGv3Data-2048;
		v1QRS_result=qrs_detect(v1LT_count,ECGv1_Rawdata);
		v2QRS_result=qrs_detect(v2LT_count,ECGv2_Rawdata);
		v3QRS_result=qrs_detect(v3LT_count,ECGv3_Rawdata);
		
		if(v1QRS_result>0) {
			HR_Value=(int)(7680.0/HR_count);//60*128/HR_count +0.5 for int convert
//			HRV_Data[HRV_Store_PT] = HR_Value;
//			HRV_Store_PT++;
			HR_count=0;
		}
		else {
			HR_count++;
		}
		
//		if(HRV_Count==108000)	//(360Hz*60)*5   5mins
//		{
//			uint16_t i=0;
//							int HRV_Sum=0;
//							int HRV_Mean =0;
//							int DNN=0; 
//							HRV_NN50 = 0;
//							HRV_PNN50 =0;
//							
//							
//							for(i=0; i<HRV_Store_PT; i++) {
//								HRV_Sum += HRV_Data[i]; // calculate HRV total sum from [0]~[HRV_Store_PT]
//							}
//							if(HRV_Store_PT != 0)
//								HRV_Mean = HRV_Sum/HRV_Store_PT; // calculate HRV mean value.
//							for(i=0; i<HRV_Store_PT; i++) {
//								DNN += ((HRV_Mean - HRV_Data[i])*(HRV_Mean-HRV_Data[i])); 
//							}
//							if(HRV_Store_PT != 0)
//								HRV_SDNN =(int)sqrt(DNN/HRV_Store_PT); // calculate HRV SDNN
//							for(i=0; i<HRV_Store_PT-1; i++) {      // calculate HRV NN50
//								if(HRV_Data[i] >= HRV_Data[i+1]){
//									if((HRV_Data[i] - HRV_Data[i+1]) > NN50_Pcount)
//										HRV_NN50++;
//								}
//								else
//									if((HRV_Data[i+1] - HRV_Data[i]) > NN50_Pcount)
//										HRV_NN50++;																			
//							}		
//							HRV_Store_PT =0;
//							HRV_Count=0;
//		}
		
		//�z�L�Ū޶ǰe�߹q�T��
//		TM_USART_Putc(USART2,ECGLow);
//		TM_USART_Putc(USART2,ECGHigh);
		
		
		//if USB is connected,send out all data via USB
		if(TM_USB_VCP_GetStatus()==TM_USB_VCP_CONNECTED){
			TM_USB_VCP_Putc(0xA5);	//Header
			TM_USB_VCP_Putc(0xA5);	//Header
			TM_USB_VCP_Putc(GsensorXLow);	//Gsensor_X,Low,Graph1
			TM_USB_VCP_Putc(GsensorXHigh);	//Gsensor_X,High,Graph1
			TM_USB_VCP_Putc((ECGv1_Rawdata[(v1LT_count)&(v1Rawdata_N-1)]+2048)&0xFF);	//ECG,Low,Graph2
			TM_USB_VCP_Putc(((ECGv1_Rawdata[(v1LT_count)&(v1Rawdata_N-1)]+2048)>>8)&0xFF);	//ECG,High,Graph2
			TM_USB_VCP_Putc((ECGv2_Rawdata[(v2LT_count)&(v2Rawdata_N-1)]+2048)&0xFF);	//ECG,Low,Graph2
			TM_USB_VCP_Putc(((ECGv2_Rawdata[(v2LT_count)&(v2Rawdata_N-1)]+2048)>>8)&0xFF);	//ECG,High,Graph2
			TM_USB_VCP_Putc((ECGv3_Rawdata[(v3LT_count)&(v3Rawdata_N-1)]+2048)&0xFF);	//ECG,Low,Graph2
			TM_USB_VCP_Putc(((ECGv3_Rawdata[(v3LT_count)&(v3Rawdata_N-1)]+2048)>>8)&0xFF);	//ECG,High,Graph2
//			TM_USB_VCP_Putc(ECGv2Low);	//ECG,Low,Graph3
//			TM_USB_VCP_Putc(ECGv2High);	//ECG,High,Graph3
//			TM_USB_VCP_Putc(ECGv3Low);	//ECG,Low,Graph4
//			TM_USB_VCP_Putc(ECGv3High);	//ECG,High,Graph4
//			TM_USB_VCP_Putc(GsensorYLow);	//Gsensor_Y,Low,Graph3
//			TM_USB_VCP_Putc(GsensorYHigh);	//Gsensor_Y,High,Graph3
//			TM_USB_VCP_Putc(GsensorZLow);	//Gsensor_Z,Low,Graph4
//			TM_USB_VCP_Putc(GsensorZHigh);	//Gsensor_Z,High,Graph4
			TM_USB_VCP_Putc(TempLow);	//Temprature,Low
			TM_USB_VCP_Putc(TempHigh);	//Temprature,High
			TM_USB_VCP_Putc((unsigned char)v1QRS_result);	//Reddot on ECG
			TM_USB_VCP_Putc((unsigned char)v2QRS_result);	//Reddot on ECG
			TM_USB_VCP_Putc((unsigned char)v3QRS_result);	//Reddot on ECG
			TM_USB_VCP_Putc((unsigned char)HR_Value);	//HeartRate on ECG
		}
		
		count_t++;
		v1LT_count++;
		v2LT_count++;
		v3LT_count++;
		TempCount--;
		
		TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);  //�M�����_���Х�
	}
}
/*==============================================================================*/

void Delay(__IO uint32_t nCount){
  while(nCount--)
  {
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
static void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
