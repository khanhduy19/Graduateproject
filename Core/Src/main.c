/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "driver.h"
#include "lora.h"
#include "stdio.h"
#include "stdlib.h"
#include <stdbool.h>
#include <string.h>
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char *str = {0};
//////////////TIMER////////////
int k=0,c=0,q=0,e=0,w=0,v=0;
float XB=15,YB=2,ZB=74,XLB=13,YLB=1,ZLB=77;
//////////////WIND/////////////
int 	count=0;
char 	control1[6];
float 	wind_count,
		circumference_cm,
		rotations,
		distance_km,
		km_per_hour,
		speed;
///////////////LORA///////////////
char data[30]={0};
char data1[30]={0};
char receive_data[50]={0};
char receive_data1[50]={0};
uint8_t p=0,m=0;
////////////	DAT	/////////////
uint16_t adc_value=0;
float	GTao,b;
int GTthuc;
char sensor[10],sensor1[100];
char buffergn[100];
char buffergn1[100];
char l;

/////////////	GPS	////////////////
#define sentenceSize 80
char sentence[sentenceSize];
char datagps[50] = {0};
char datagps1[50] = {0};
char KD[100]= {0};
char VD[100]= {0};
/////////////////////////	pin 	//////////////////
int pnguyen,pthapphan;
int giatri;
char Phantrampin[100];
char *datagps1B = {0};
int warning;
///////////////////////////////////////////////////

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/////////////////////////  PIN  //////////////////////////////////
#define MAX17043_ADDRESS 0x36<<1
uint8_t data_max[20];
void write (uint8_t reg,uint8_t value)
{
uint8_t data[2];
		data[0] = reg;
	  	data[1] = value;
	  	HAL_I2C_Master_Transmit (&hi2c1, MAX17043_ADDRESS, data, 2, 10);
}
void read (uint8_t reg )
{
	HAL_I2C_Mem_Read(&hi2c1, MAX17043_ADDRESS, reg, 1, (uint8_t *)data_max,3, 100);

}
void max17043_init(void)
{
write(0xFE,0x00);
write(0xFE,0x54);
HAL_Delay(100);
write(0x06, 0x00);
}
void pin()
{
	max17043_init();
	HAL_Delay(100);
	read(0x04);
	pnguyen = data_max[0];
    pthapphan = data_max[1];
	giatri = pnguyen + (pthapphan/256);
	HAL_Delay(500);
   	sprintf(Phantrampin,"\n phan tram pin = %d%%  \r\n", giatri);
   	HAL_UART_Transmit(&huart1, (uint8_t*)Phantrampin, strlen(Phantrampin), 100);
	  HAL_Delay(100);
	  }
/////////////////////////	GOC NGHIENG 1 	//////////////////
#define adxl_address 0x53<<1

int8_t data_rec1[6];
int16_t xl,yl,zl;
float xgl,ygl,zgl,
	  XL,YL,ZL,xxL,yyL,zzL;

void adxl_write1 (uint8_t reg,uint8_t value)
{
	uint8_t data1[2];
	data1[0] = reg;
	data1[1] = value;
	HAL_I2C_Master_Transmit (&hi2c2, adxl_address, data1, 2, 10);
}
void adxl_read1 (uint8_t reg, uint8_t numberofbytes)
{
	for(int i = 0; i < sizeof(data_rec1); i++)
		{
			data_rec1[i] = 0;
		}
	HAL_I2C_Mem_Read(&hi2c2, adxl_address, reg, 1, (uint8_t *)data_rec1,numberofbytes, 100);
}
void adxl_init1(void)
{
	adxl_read1(0x00,1);
	adxl_write1(0x2d,0);
	adxl_write1(0x2d,0x08);
	adxl_write1(0x31,0x01);

}

void kq_adxl1()
{
	adxl_read1(0x32,6);
	xl=data_rec1[1]<<8| data_rec1[0];
	yl=data_rec1[3]<<8| data_rec1[2];
	zl=data_rec1[5]<<8| data_rec1[4];
	xgl = xl* .0078;
	ygl = yl* .0078;
	zgl = zl* .0078;
//	fXg = Xg * alpha + (fXg * (1.0 - alpha));
	if(xgl!=0 || ygl!=0 || zgl!=0)
	{
//		fxg=xg* 0.5 +(fxg*0.5);
//		fyg=yg* 0.5 +(fyg*0.5);
//		fzg=zg* 0.5 +(fzg*0.5);

		//roll  = (atan2(-fyg, fzg)*180.0)/3.14;
		XL = (atan2(xgl, sqrt(ygl*ygl + zgl*zgl))*180.0)/3.14;
		YL = (atan2(ygl, sqrt(xgl*xgl + zgl*zgl))*180.0)/3.14;
		ZL = (atan2(zgl, sqrt(xgl*xgl + ygl*ygl))*180.0)/3.14;
//		sprintf(buffergn1,"\r\n XL=%.4f YL=%.4f  ZL=%.4f \r\n ",XL, YL,ZL );
//		HAL_UART_Transmit(&huart1 , (uint8_t *)buffergn1, strlen(buffergn1), 1000);
//
//		HAL_Delay(1000);
	}
	else
	{
		XL=0;
		YL=0;
		ZL=0;
//		sprintf(buffergn1,"\r\n XL=%.4f YL=%.4f  ZL=%.4f \r\n ",XL, YL,ZL );
//      HAL_UART_Transmit(&huart1 , (uint8_t *)buffergn1, strlen(buffergn1), 1000);
//		HAL_Delay(500);

	}
}
void kq_adxl_timer1()
{
	adxl_read1(0x32,6);
	xl=data_rec1[1]<<8| data_rec1[0];
	yl=data_rec1[3]<<8| data_rec1[2];
	zl=data_rec1[5]<<8| data_rec1[4];
	xgl = xl* .0078;
	ygl = yl* .0078;
	zgl = zl* .0078;
//	fXg = Xg * alpha + (fXg * (1.0 - alpha));
	if(xgl!=0 || ygl!=0 || zgl!=0)
	{
//		fxg=xg* 0.5 +(fxg*0.5);
//		fyg=yg* 0.5 +(fyg*0.5);
//		fzg=zg* 0.5 +(fzg*0.5);

		//roll  = (atan2(-fyg, fzg)*180.0)/3.14;
		XL = (atan2(xgl, sqrt(ygl*ygl + zgl*zgl))*180.0)/3.14;
		YL = (atan2(ygl, sqrt(xgl*xgl + zgl*zgl))*180.0)/3.14;
		ZL = (atan2(zgl, sqrt(xgl*xgl + ygl*ygl))*180.0)/3.14;
		sprintf(buffergn1,"\r\n XL=%.4f YL=%.4f  ZL=%.4f \r\n ",XL, YL,ZL );
		HAL_UART_Transmit(&huart1 , (uint8_t *)buffergn1, strlen(buffergn1), 100);
//
//		HAL_Delay(1000);
	}
	else
	{
		XL=0;
		YL=0;
		ZL=0;
//		sprintf(buffergn1,"\r\n XL=%.4f YL=%.4f  ZL=%.4f \r\n ",XL, YL,ZL );
//      HAL_UART_Transmit(&huart1 , (uint8_t *)buffergn1, strlen(buffergn1), 1000);
//		HAL_Delay(500);

	}
}

////////////////////////////// goc nghieng 2 ///////////////////////////
#define adxl_address 0x53<<1

int8_t data_rec[6];
int16_t x,y,z;
float xg,yg,zg,
	  X,Y,Z,xx,yy,zz;

void adxl_write (uint8_t reg,uint8_t value)
{
	uint8_t data[2];
	data[0] = reg;
	data[1] = value;
	HAL_I2C_Master_Transmit (&hi2c1, adxl_address, data, 2, 10);
}
void adxl_read (uint8_t reg, uint8_t numberofbytes)
{
	for(int i = 0; i < sizeof(data_rec); i++)
		{
			data_rec[i] = 0;
		}
	HAL_I2C_Mem_Read(&hi2c1, adxl_address, reg, 1, (uint8_t *)data_rec,numberofbytes, 100);
}
void adxl_init(void)
{
	adxl_read(0x00,1);
	adxl_write(0x2d,0);
	adxl_write(0x2d,0x08);
	adxl_write(0x31,0x01);

}

void kq_adxl()
{
	adxl_read(0x32,6);
	x=data_rec[1]<<8| data_rec[0];
	y=data_rec[3]<<8| data_rec[2];
	z=data_rec[5]<<8| data_rec[4];
	xg = x* .0078;
	yg = y* .0078;
	zg = z* .0078;
//	fXg = Xg * alpha + (fXg * (1.0 - alpha));
	if(xg!=0 || yg!=0 || zg!=0)
	{
//		fxg=xg* 0.5 +(fxg*0.5);
//		fyg=yg* 0.5 +(fyg*0.5);
//		fzg=zg* 0.5 +(fzg*0.5);

		//roll  = (atan2(-fyg, fzg)*180.0)/3.14;
		X = (atan2(xg, sqrt(yg*yg + zg*zg))*180.0)/3.14;
		Y = (atan2(yg, sqrt(xg*xg + zg*zg))*180.0)/3.14;
		Z = (atan2(zg, sqrt(xg*xg + yg*yg))*180.0)/3.14;
//		sprintf(buffergn,"\r\n XH=%.4f YH=%.4f  ZH=%.4f \r\n ",X, Y,Z );
//		HAL_UART_Transmit(&huart1 , (uint8_t *)buffergn, strlen(buffergn), 1000);
//
//		HAL_Delay(1000);
	}
	else
	{
		X=0;
		Y=0;
		Z=0;
//		sprintf(buffergn,"\r\n XH=%.4f YH=%.4f  ZH=%.4f \r\n ",X, Y,Z );
//        HAL_UART_Transmit(&huart1 , (uint8_t *)buffergn, strlen(buffergn), 1000);
//		HAL_Delay(500);

	}
}


void kq_adxl_timer()
{
	adxl_read(0x32,6);
	x=data_rec[1]<<8| data_rec[0];
	y=data_rec[3]<<8| data_rec[2];
	z=data_rec[5]<<8| data_rec[4];
	xg = x* .0078;
	yg = y* .0078;
	zg = z* .0078;
	//fXg = Xg * alpha + (fXg * (1.0 - alpha));
	if(xg!=0 || yg!=0 || zg!=0)
	{
//		fxg=xg* 0.5 +(fxg*0.5);
//		fyg=yg* 0.5 +(fyg*0.5);
//		fzg=zg* 0.5 +(fzg*0.5);

		//roll  = (atan2(-fyg, fzg)*180.0)/3.14;
		X = (atan2(xg, sqrt(yg*yg + zg*zg))*180.0)/3.14;
		Y = (atan2(yg, sqrt(xg*xg + zg*zg))*180.0)/3.14;
		Z = (atan2(zg, sqrt(xg*xg + yg*yg))*180.0)/3.14;
		sprintf(buffergn,"\r\n X=%.4f Y=%.4f  Z=%.4f \r\n ",X, Y,Z );
		HAL_UART_Transmit(&huart1 , (uint8_t *)buffergn, strlen(buffergn), 1000);

	//HAL_Delay(500);
	}
	else
	{
		X=0;
		Y=0;
		Z=0;
		sprintf(buffergn,"\r\n X=%.4f Y=%.4f  Z=%.4f \r\n ",X, Y,Z );
        HAL_UART_Transmit(&huart1 , (uint8_t *)buffergn, strlen(buffergn), 1000);
		HAL_Delay(1000);

	}
}

//////////////lora////////

void LoRa_txMode(void)
{
    LoRa_idle(); // set standby mode
    LoRa_enableInvertIQ(); // active invert I and Q signals
}

void LoRa_rxMode(void)
{
	LoRa_disableInvertIQ(); // normal mode
    LoRa_receive(0); // set receive mode
}

void send_id(uint8_t id)
{
    char buffer[30];
    sprintf(buffer, "%d", id);
    LoRa_txMode();
    LoRa_sendPacket(buffer);

}

uint8_t get_id(void)
{
    char buffer[30]={0};

    LoRa_rxMode();
    LoRa_receivePacket(buffer, 7000);
    uint8_t id = atoi(buffer);
    for(uint8_t i = 0; i < strlen(buffer); i++)
        {
            buffer[i] = 0;
        }
    return id;
}

void clear_buffer(char* buffer)
{
    uint8_t i;
    for(i = 0; i < sizeof(buffer); i++)
    {
        buffer[i] = 0;
    }
}

void Response_Gateway(void)
{
    uint8_t ID,t=0,k=0,i=0,q=0;


    char send_data[120]={0};

    	for( i = 0; i < sizeof(data); i++)
    	{
    		data[i] = 0;
    	}
        LoRa_rxMode();
    	LoRa_receivePacket(data, 40000);
    	 HAL_UART_Transmit(&huart1, (uint8_t *)data ,sizeof(data), 5000);
        if(!strcmp(data,"hello"))
        {
        	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        	HAL_UART_Transmit(&huart1, (uint8_t *)"\n\rSend ID to Gateway:" ,21,10000);
        	LoRa_txMode();
         	HAL_Delay(1000);
            send_id(ID_NODE);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
            HAL_Delay(500);
            sprintf(data1, " %d",ID_NODE );
            HAL_UART_Transmit(&huart1, (uint8_t *)data1 ,strlen(data1), 5000);
            HAL_UART_Transmit(&huart1, (uint8_t *)"\n\rGet ID from Gateway:" ,22, 10000);
            ID = get_id();
            sprintf(data1, " %d", ID);
            HAL_UART_Transmit(&huart1, (uint8_t *)data1 ,strlen(data1), 5000);
            LoRa_rxMode();
            t=0;
            k=0;
            if(ID == ID_NODE)
            {
            	while(1)
            	{

            		for( q = 0; q < sizeof(receive_data); q++)
            		{
            			receive_data[q] = 0;
            		}
            		LoRa_receivePacket(receive_data, 20000);
            		k++;
            		if(k==10) ///////////////////// thêm nút thì tăng k
            		{
            			break;
            		}
            		if(!strcmp(receive_data,"send_data"))
            		{
            			for( k=0;k<5;k++)
            			{
            				if(t==8) //////////////////khoảng 4 nút
            				{
            					break;
            				}
            				while(1)

            				{
            					for( i = 0; i < sizeof(receive_data1); i++)
								{
									 receive_data1[i] = 0;
								}
            					LoRa_rxMode();
            					LoRa_receivePacket(receive_data1, 12000);
            					if(!strcmp(receive_data1,"1"))
            					{
            						HAL_ADC_Start_IT(&hadc1);
            						KqTocdogio();
            						pin();
            						kq_adxl();
            						kq_adxl1();
									//itoa(speed,control1,10);///////gio
									itoa(GTthuc,sensor,10);///////dat
									HAL_Delay(400);
									sprintf(send_data,"*%s!%s#%s$%.f&%.f %.f^%s@%d<%.f&%.f %.f^%d@",receive_data1,datagps1B,sensor,X,Y,Z,control1,giatri,XL,YL,ZL,warning );
									HAL_UART_Transmit(&huart1, (uint8_t *)"\n\rSend data node2:" ,20,10000);
									HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
									HAL_Delay(1000);
									LoRa_txMode();
									LoRa_sendPacket(send_data);
									if( X-xx>5 || Y-yy>5 )
								    {
									  p++;
									}
									if( X-xx<-5 || Y-yy<-5 )
									{
									  m++;
									}
									HAL_UART_Transmit(&huart1, (uint8_t *)"\n\r", 3,1000);
									HAL_UART_Transmit(&huart1, (uint8_t *)send_data,strlen(send_data),5000);
									for(i=0;i<sizeof(send_data);i++)
									{
										send_data[i]=0;
									}
									for(i=0;i<sizeof(sensor);i++)
									{
										sensor[i]=0;
									}
									HAL_Delay(500);
									t=0;
									HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
									break;
            					}
            					t++;
            					if(t==8)
            					{
            						break;
            					}
            					for( i = 0; i < sizeof(receive_data1); i++)
            					{
            					     receive_data1[i] = 0;
            					}

            				}


            			}
            			LoRa_txMode();
            			HAL_TIM_Base_Stop_IT(&htim1);
            			HAL_NVIC_DisableIRQ(EXTI3_IRQn);
//            			RTC_SetTime(9, 9, 0, 20, 3, 21);
//            			sleep_1p();
            			HAL_NVIC_EnableIRQ(EXTI3_IRQn);
            			break;
            		}
            		for( q = 0; q < sizeof(receive_data); q++)
            		{
            			receive_data[q] = 0;
            		}


            	}
            }
        }

}
////////////////////	WIND	//////////////////
void KqTocdogio()
  {
	  HAL_UART_Transmit(&huart1,(uint8_t*)"toc do gio: ",14,10000);
	  itoa(speed,control1,10);
	  HAL_UART_Transmit(&huart1,(uint8_t*)control1,4,10000);
	  HAL_UART_Transmit(&huart1,(uint8_t*)"km/h ",6,10000);
	  HAL_UART_Transmit(&huart1,(uint8_t*)"\n\r ",2,10000);
	  HAL_Delay(999);
  }

//////////////////	GPS		//////////////////////
void getField(char* buff, int index)
{
  int sentencePos = 0;
  int fieldPos = 0;
  int commaCount = 0;
  while (sentencePos < sentenceSize)
  {
    if (sentence[sentencePos] == ',')
    {
      commaCount ++;
      sentencePos ++;
    }
    if (commaCount == index)
    {
      buff[fieldPos] = sentence[sentencePos];
      fieldPos ++;
    }
    sentencePos ++;
  }
  buff[fieldPos] = '\0';
}
void kqgps()
{
	  int i = 0;
	  char ch;
	  while (1)
	  {
		  if(HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, 100) == HAL_OK)
		  {
			  if (ch != '\n' && i < sentenceSize)
			  {
				sentence[i] = ch;
				i++;
			  }
			  else
			  {
				  sentence[i] = '\0';
				  i = 0;
				  char field[20];
				  char TIME[20];
				  char hour[3];
				  int hourValue;
				  char minute[3];
				  char second[3];
				  char LATITUDE[15]={0};
				  char northSouth[3];
				  char LONGITUDE[15]={0};
				  char westEast[3];
				  char ALTITUDE[10];

				  getField(field, 0);
				  if (strcmp(field, "$GPGGA") == 0)
				  {
					  getField(TIME, 1); // Time
					  getField(LATITUDE, 2); // Latitude
					  getField(northSouth, 3); // North/South
					  getField(LONGITUDE, 4); // Longitude
					  getField(westEast, 5); // West/East
					  getField(ALTITUDE, 9); // Altitude above mean sea level

					  strncpy(hour, TIME, 2);
					  hourValue = atoi(hour) + 7; // Add 2 to adjust for France zone from GMT
					  itoa(hourValue, hour, 10);
					  hour[2]= '\0';
					  strncpy(minute, &TIME[2], 2);
					  minute[2]= '\0';
					  strncpy(second, &TIME[4], 2);
					  second[2]= '\0';

//					  sprintf(datagps, "%s:%s:%s, %s%s, %s%s, %s ", hour, minute, second, LATITUDE, northSouth, LONGITUDE, westEast, ALTITUDE);
					  sprintf(datagps, "%s>%s", LONGITUDE,LATITUDE);

					  HAL_UART_Transmit(&huart1, (uint8_t*)"\n----->", 8, 1000);
					  HAL_UART_Transmit(&huart1, (uint8_t*)datagps, strlen(datagps), 5000);
					  if(ALTITUDE[0] !=48)
					  {
						 break;
					  }
					  for(uint8_t i=0;i<sizeof(datagps);i++)
					  {
						  datagps[i]=0;
					  }

				  }

			  }
		  }
	  }
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
 // kqgps();
  max17043_init();
  for(uint8_t i=0;i<strlen(datagps);i++)
  {
  	datagps1[i]=datagps[i];
  }
  adxl_init1();
  adxl_init();
  if(!LoRa_Init(434E6))
   {
 	  char str[] = "\n\rLoRa init failed\n\r";
 	  HAL_UART_Transmit(&huart1 , (uint8_t*)str, sizeof(str), 1000);
       while(1);
   }
	LoRa_setSignalBandwidth(125E3);
	LoRa_setSpreadingFactor(8);
	LoRa_setPreambleLength(8);
	LoRa_setCodingRate4(5);
	LoRa_enableCrc();
   char str[] = "\n\rLoRa Node init succeeded\n\r";
   HAL_UART_Transmit(&huart1 , (uint8_t*)str, sizeof(str), 1000);
   LoRa_rxMode();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */




   for(uint8_t a=0;a<3;a++)
   {
	   kq_adxl();
	   kq_adxl1();
	   xxL=XL;
	   yyL=YL;
	   zzL=ZL;
	   xx=X;
	   yy=Y;
	   zz=Z;
	   sprintf(buffergn,"\r\n XH=%.4f YH=%.4f  ZH=%.4f \r\n ",X, Y,Z );
	   sprintf(buffergn1,"\r\n XL=%.4f YL=%.4f  ZL=%.4f \r\n ",XL, YL,ZL );
	   HAL_UART_Transmit(&huart1 , (uint8_t *)buffergn, strlen(buffergn), 100);
	   HAL_UART_Transmit(&huart1 , (uint8_t *)buffergn1, strlen(buffergn1), 100);
	   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	   HAL_Delay(200);
	   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	   datagps1B = "10546.8284>1002.8274";
   }

  while (1)
  {
	  HAL_TIM_Base_Start_IT(&htim1);
	  Response_Gateway();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RESET_Pin|GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : NSS_Pin */
  GPIO_InitStruct.Pin = NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RESET_Pin PB13 PB4 */
  GPIO_InitStruct.Pin = RESET_Pin|GPIO_PIN_13|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == DIO0_Pin)
	 {
		LoRa_handleDio0Rise();
	 }
	if(GPIO_Pin==GPIO_PIN_3)
		count++;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
  {
	  if(hadc->Instance==ADC1)
	  {
		  adc_value=HAL_ADC_GetValue(&hadc1);
		  b=adc_value;
		  GTao=(b/4096)*100;
		  GTthuc=100-GTao;
		  if(GTthuc<5)
		  {
			  GTthuc=0;
		  }
		  HAL_UART_Transmit(&huart1,(uint8_t*)"HUMI:", 6,1000);
		  sprintf(sensor,"%d",GTthuc);
		  HAL_UART_Transmit(&huart1,(uint8_t*)sensor, 5,1000);
		  HAL_UART_Transmit(&huart1,(uint8_t*)"%", 1,1000);
		  HAL_UART_Transmit(&huart1,(uint8_t*)"\n\r", 2,1000);
		  for(uint8_t i=0;i<3;i++)
		  {
			  sensor[i]=0;
		  }
	  }
  }

void Counter()
{
	count++;
}

void Wind()
{
	uint8_t radius_cm = 9;

	circumference_cm = 2 * 3.14 * radius_cm;
	rotations = count / 2;
	distance_km = (circumference_cm * rotations) / 100000;
	km_per_hour = (distance_km / 1) * 3600;
	speed = km_per_hour * 1.18;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim1.Instance)
	{
		Wind();
		count=0;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		if (q==5)
		{
			q=0;
			k=0;
			c=0;
			w=0;
			v=0;
			//warning=0;
		}
		if(e==20)
		{
			e=0;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		}
		kq_adxl_timer1();
		kq_adxl_timer();
		if( XL-xxL>5 || YL-yyL>5 )
		{
			w++;
		}
		if( XL-xxL<-5 || YL-yyL<-5 )
		{
			v++;
		}
		if( X-xx>15 || Y-yy > 15 )
		{
			k++;
		}
		if( X-xx<-15 ||Y-yy<-15 )
		{
			c++;
		}
		if(X!=0 && Y!=0 && Z!=0)
		{	if(k>2 && w>2 )
			{
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
//			  sstr = "warning\r\n";
//			  HAL_UART_Transmit(&huart1, (uint8_t *)sstr, strlen (sstr), HAL_MAX_DELAY);
			  warning = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
			}
			if(c>2 && v>2)
			{
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
//			 sstr = "warning\r\n";
//			 HAL_UART_Transmit(&huart1, (uint8_t *)sstr, strlen (sstr), HAL_MAX_DELAY);
			  warning = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
			}
			if(k>2 && v>2 )
			{
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		//	 sstr = "warning\r\n";
		//	 HAL_UART_Transmit(&huart1, (uint8_t *)sstr, strlen (sstr), HAL_MAX_DELAY);
			 warning = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
			}
			if(c>2 && w>2)
			{
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		//	 sstr = "warning\r\n";
		//	 HAL_UART_Transmit(&huart1, (uint8_t *)sstr, strlen (sstr), HAL_MAX_DELAY);
			 warning = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
					}
		    }
		q++;
		if(HAL_GPIO_ReadPin(GPIOB,  GPIO_PIN_4)==1)
		{
			e++;
		}
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
