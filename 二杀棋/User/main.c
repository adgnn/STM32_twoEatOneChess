#include <stm32f2xx.h>
/* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N */
#define PLL_M      25
#define PLL_N      240
/* SYSCLK = PLL_VCO / PLL_P */
#define PLL_P      2
/* USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ */
#define PLL_Q      5
#define FSMC_LCD_DATA ((uint32_t)0x60020000)
#define FSMC_LCD_REG ((uint32_t)0x60000000)

// ���״̬����
#define chooseChess 1
#define chooseDir 2
#define Move 3
#define Rest 4

//������
#define UP 0
#define DOWN 1
#define LEFT 2
#define RIGHT 3

//������ɫ����
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000
#define BLUE         	 0x001F
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40 //��ɫ
#define BRRED 			 0XFC07 //�غ�ɫ
#define GRAY  			 0X8430 //��ɫ

struct Player{    //��ʾ��ҵġ��������ӵ�״̬�����꡿�����״̬��
	int chessState[6];
	// ��(chessX[0], chessY[0])��ʾ����1��λ��
	int chessX[6];
	int chessY[6];
	int playerState; //���״̬
	//1��chooseChess��ѡ����
	//2��chooseDirѡ�з���
	//3��Move�ƶ�����
	//4:Judge�ж��Ƿ��ܳԵ��������ӣ��ж��Ƿ��ʤ
	//5��Rest����
};

enum gameState
{ /* �ɸ���gameState�л�����ܵ���ʾ */
	player1Turn = 1, //���1�غ� 
	player2Turn,
	player1Win, //���1��ʤ
	player2Win,
};

struct direction{
	int dx;
	int dy;
}DIR[4] = { {1,0}, {-1,0}, {0,1}, {0,-1} };

struct Coord{
	int LCDX;
	int LCDY;
}coord[5][4] = { 
{{20,200},{20,160},{20,120},{20,80}},
{{60,200},{60,160},{60,120},{60,80}},
{{100,200},{100,160},{100,120},{100,80}},
{{140,200},{140,160},{140,120},{140,80}},
{{180,200},{180,160},{180,120},{180,80}},
};


/***********************************��������*************************************/
vu16 Delay_ms(vu16 count);
void LCD_IO_WriteReg(uint8_t Reg) ;
void LCD_IO_WriteData(uint16_t Data) ;
void LCD_DrawPoint(u16 x,u16 y,uint16_t GRB);
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);
void LCD_Color_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t GRB);
void DrawCBGrid();
void LCD_DrawRect(u16 ox, u16 oy, u16 ex, u16 ey, uint16_t GRB);
void clearAll();
void LCD_DrawAChess(int x, int y, int num, uint16_t GRB);
void DrawCB();
/********************************************************************************/

/*********************************ȫ�ֱ�������************************************/
__IO uint32_t StartUpCounter = 0, HSEStatus = 0;
GPIO_InitTypeDef GPIO_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
FSMC_NORSRAMTimingInitTypeDef  p;
u16 key_button1,key_button2,key_button3,key_button4;
u8 key_value = 100;

int cx; //ѡ�����ӵ�x����
int cy; //ѡ�����ӵ�y����
int gocx; //���Ӵ�������x
int gocy; //���Ӵ�������y
int whichChess; //ѡ�����Ӻ���

int CB[5][4] ={ {1,1,1,1}, {1,0,0,1}, {0,0,0,0}, {1,0,0,1}, {1,1,1,1} };
int i;
	
struct Player player1 = {
{1,1,1,1,1,1}, 
{0,0,0,0,1,1},
{0,1,2,3,0,3},
1};
struct Player player2 = {
{1,1,1,1,1,1}, 
{4,4,4,4,3,3},
{0,1,2,3,0,3},
1};
struct Player* Player1 = &player1;
struct Player* Player2 = &player2;

enum gameState State = 1;
/********************************************************************************/

int main()
{    

	/*********************************�ڲ���������************************************/	


	/*********************************ϵͳ��ʼ��*************************************/
	/*            PLL (clocked by HSE) used as System clock source                */
	__IO uint32_t StartUpCounter = 0, HSEStatus = 0;
	RCC->CR |= (uint32_t)0x00000001;
	/* Reset CFGR register */
	RCC->CFGR = 0x00000000;
	/* Reset HSEON, CSSON and PLLON bits */
	RCC->CR &= (uint32_t)0xFEF6FFFF;
	/* Reset PLLCFGR register */
	RCC->PLLCFGR = 0x24003010;
	/* Reset HSEBYP bit */
	RCC->CR &= (uint32_t)0xFFFBFFFF;
	/* Disable all interrupts */
	RCC->CIR = 0x00000000;
	/* Enable HSE */
	RCC->CR |= ((uint32_t)RCC_CR_HSEON);
	/* Wait till HSE is ready and if Time out is reached exit */
	do
	{
	HSEStatus = RCC->CR & RCC_CR_HSERDY;
	StartUpCounter++;
	} while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

	if ((RCC->CR & RCC_CR_HSERDY) != RESET)
	{
	HSEStatus = (uint32_t)0x01;
	}
	else
	{
	HSEStatus = (uint32_t)0x00;
	}

	if (HSEStatus == (uint32_t)0x01)
	{
	/* HCLK = SYSCLK / 1*/
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

	/* PCLK2 = HCLK / 2*/
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

	/* PCLK1 = HCLK / 4*/
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

	/* Configure the main PLL */
	RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) |
						 (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);

	/* Enable the main PLL */
	RCC->CR |= RCC_CR_PLLON;

	/* Wait till the main PLL is ready */
	while((RCC->CR & RCC_CR_PLLRDY) == 0)
	{
	}

	/* Configure Flash prefetch, Instruction cache, Data cache and wait state */
	FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_3WS;

	/* Select the main PLL as system clock source */
	RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	/* Wait till the main PLL is used as system clock source */
	while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL);
	{
	}
	}
	else
	{ /* If HSE fails to start-up, the application will have wrong clock
	 configuration. User can add here some code to deal with this error */
	}

	/********************************************************************************/
	/*********************************�ڲ������жϳ�ʼ��**************************************/

	/* Set the Vector Table base address at 0x08000000 */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);
	/* Configure the Priority Group to 2 bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;     	//ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		//��ռʽ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//��Ӧʽ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;   
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;    
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/********************************************************************************/
	/*********************************�ⲿ�жϳ�ʼ��**************************************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG , ENABLE);
	/* Connect Button EXTI Line to Button GPIO Pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD,EXTI_PinSource3);
	/* Configure Button EXTI line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line3;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;   //�½��ش���
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line3);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD,EXTI_PinSource6);
	EXTI_InitStructure.EXTI_Line = EXTI_Line6;
	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line6);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource0);
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line0);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource1);
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line1);

	/********************************************************************************/

	/*********************************������ʼ��**************************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 |GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOE,&GPIO_InitStructure); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 |GPIO_Pin_6; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOD,&GPIO_InitStructure); 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOE,&GPIO_InitStructure); 

	GPIO_ResetBits(GPIOE, GPIO_Pin_2 |GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5);
	GPIO_SetBits(GPIOE, GPIO_Pin_0 |GPIO_Pin_1);
	GPIO_SetBits(GPIOD, GPIO_Pin_6 |GPIO_Pin_3);
	//������Ϊ����ˣ�������Ϊ�����
	//������û�а���ʱ�����е�������õ͵�ƽ�����е�����˶��Ǹߵ�ƽ

	/********************************************************************************/

	
	/*********************************LCD��ʼ��**************************************/
	/* Enable GPIOs clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOC, ENABLE);

	/* Enable FSMC clock */
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				 //PC0 ������� 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 //�������
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_SetBits (GPIOD,GPIO_Pin_12);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;				 //PC0 ������� ����
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 //�������
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits (GPIOC,GPIO_Pin_0);
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC); 

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7 |
	GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* GPIOE configuration */
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource15 , GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7 |	GPIO_Pin_8  | GPIO_Pin_9  | GPIO_Pin_10 | GPIO_Pin_11|
		GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;

	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/*-- FSMC Configuration ------------------------------------------------------*/
	p.FSMC_AddressSetupTime = 6;
	p.FSMC_AddressHoldTime = 1;
	p.FSMC_DataSetupTime = 9;
	p.FSMC_BusTurnAroundDuration = 1;
	p.FSMC_CLKDivision = 0;
	p.FSMC_DataLatency = 0;
	p.FSMC_AccessMode = FSMC_AccessMode_A;

	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
	FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;  
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;

	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 

	/*!< Enable FSMC Bank1_SRAM1 Bank */
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);
  Delay_ms(200);
  GPIO_ResetBits (GPIOD,GPIO_Pin_12);
  Delay_ms(500);
  GPIO_SetBits (GPIOD,GPIO_Pin_12);
  Delay_ms(500);
 	Delay_ms(50); 					// delay 50 ms 
 	LCD_IO_WriteReg(0xCF);  
	LCD_IO_WriteData(0x00); 
	LCD_IO_WriteData(0xC1); 
	LCD_IO_WriteData(0X30); 
	LCD_IO_WriteReg(0xED);  
	LCD_IO_WriteData(0x64); 
	LCD_IO_WriteData(0x03); 
	LCD_IO_WriteData(0X12); 
	LCD_IO_WriteData(0X81); 
	LCD_IO_WriteReg(0xE8);  
	LCD_IO_WriteData(0x85); 
	LCD_IO_WriteData(0x10); 
	LCD_IO_WriteData(0x7A); 
	LCD_IO_WriteReg(0xCB);  
	LCD_IO_WriteData(0x39); 
	LCD_IO_WriteData(0x2C); 
	LCD_IO_WriteData(0x00); 
	LCD_IO_WriteData(0x34); 
	LCD_IO_WriteData(0x02); 
	LCD_IO_WriteReg(0xF7);  
	LCD_IO_WriteData(0x20); 
	LCD_IO_WriteReg(0xEA);  
	LCD_IO_WriteData(0x00); 
	LCD_IO_WriteData(0x00); 
	LCD_IO_WriteReg(0xC0);    //Power control 
	LCD_IO_WriteData(0x1B);   //VRH[5:0] 
	LCD_IO_WriteReg(0xC1);    //Power control 
	LCD_IO_WriteData(0x01);   //SAP[2:0];BT[3:0] 
	LCD_IO_WriteReg(0xC5);    //VCM control 
	LCD_IO_WriteData(0x30); 	 //3F
	LCD_IO_WriteData(0x30); 	 //3C
	LCD_IO_WriteReg(0xC7);    //VCM control2 
	LCD_IO_WriteData(0XB7); 
	LCD_IO_WriteReg(0x36);    // Memory Access Control 
	LCD_IO_WriteData(0x48); 
	LCD_IO_WriteReg(0x3A);   
	LCD_IO_WriteData(0x55); 
	LCD_IO_WriteReg(0xB1);   
	LCD_IO_WriteData(0x00);   
	LCD_IO_WriteData(0x1A); 
	LCD_IO_WriteReg(0xB6);    // Display Function Control 
	LCD_IO_WriteData(0x0A); 
	LCD_IO_WriteData(0xA2); 
	LCD_IO_WriteReg(0xF2);    // 3Gamma Function Disable 
	LCD_IO_WriteData(0x00); 
	LCD_IO_WriteReg(0x26);    //Gamma curve selected 
	LCD_IO_WriteData(0x01); 
	LCD_IO_WriteReg(0xE0);    //Set Gamma 
	LCD_IO_WriteData(0x0F); 
	LCD_IO_WriteData(0x2A); 
	LCD_IO_WriteData(0x28); 
	LCD_IO_WriteData(0x08); 
	LCD_IO_WriteData(0x0E); 
	LCD_IO_WriteData(0x08); 
	LCD_IO_WriteData(0x54); 
	LCD_IO_WriteData(0XA9); 
	LCD_IO_WriteData(0x43); 
	LCD_IO_WriteData(0x0A); 
	LCD_IO_WriteData(0x0F); 
	LCD_IO_WriteData(0x00); 
	LCD_IO_WriteData(0x00); 
	LCD_IO_WriteData(0x00); 
	LCD_IO_WriteData(0x00); 		 
	LCD_IO_WriteReg(0XE1);    //Set Gamma 
	LCD_IO_WriteData(0x00); 
	LCD_IO_WriteData(0x15); 
	LCD_IO_WriteData(0x17); 
	LCD_IO_WriteData(0x07); 
	LCD_IO_WriteData(0x11); 
	LCD_IO_WriteData(0x06); 
	LCD_IO_WriteData(0x2B); 
	LCD_IO_WriteData(0x56); 
	LCD_IO_WriteData(0x3C); 
	LCD_IO_WriteData(0x05); 
	LCD_IO_WriteData(0x10); 
	LCD_IO_WriteData(0x0F); 
	LCD_IO_WriteData(0x3F); 
	LCD_IO_WriteData(0x3F); 
	LCD_IO_WriteData(0x0F); 
	LCD_IO_WriteReg(0x2B); 
	LCD_IO_WriteData(0x00);
	LCD_IO_WriteData(0x00);
	LCD_IO_WriteData(0x01);
	LCD_IO_WriteData(0x3f);
	LCD_IO_WriteReg(0x2A); 
	LCD_IO_WriteData(0x00);
	LCD_IO_WriteData(0x00);
	LCD_IO_WriteData(0x00);
	LCD_IO_WriteData(0xef);	 
	LCD_IO_WriteReg(0x11); //Exit Sleep
	Delay_ms(120);
	LCD_IO_WriteReg(0x29); //display on
	/********************************************************************************/
   	
	DrawCB();


	while(1)
	{
		GPIOE->BSRRH =(uint16_t)0x0020;//����KEY8�õ͵�ƽ
		GPIOE->BSRRL =0x001C;//����KEY5 6 7�øߵ�ƽ
		key_button1=(((GPIOD->IDR) & 0x0048)|((GPIOE->IDR) & 0x0003));
		switch(key_button1)
		{ 
			case  0x004A:   key_value=     4;                 break;
			case  0x0049:    key_value=     5;                 break;
			case  0x0043:  key_value=    2;                break;
			case  0x000B:  key_value=   3;               break;
		}			
		GPIOE->BSRRH =(uint16_t)0x0010;
		GPIOE->BSRRL =0x002C;
		key_button2=(((GPIOD->IDR) & 0x0048)|((GPIOE->IDR) & 0x0003));
		switch(key_button2)
		{ 
			case  0x004A:   key_value=     8;                 break;
			case  0x0049:    key_value=     9;                 break;
			case  0x0043:  key_value=    6;                break;
			case  0x000B:  key_value=   7;               break;
		}
		GPIOE->BSRRH =(uint16_t)0x0008;
		GPIOE->BSRRL =0x0034;
		key_button3=(((GPIOD->IDR) & 0x0048)|((GPIOE->IDR) & 0x0003));
		switch(key_button3)
		{ 
			case  0x004A : key_value=     12;                 break;
			case   0x0049 : key_value=     13;                 break;
			case 0x0043 :  key_value=    10 ;                break;
			case   0x000B :  key_value=   11;               break;
		}			
		GPIOE->BSRRH =(uint16_t)0x0004;
		GPIOE->BSRRL =0x0038;
		key_button4=(((GPIOD->IDR) & 0x0048)|((GPIOE->IDR) & 0x0003));
		switch(key_button4)
		{ 
			case  0x004A:  key_value=     16;                 break;
			case  0x0049:    key_value=     17;                 break;
			case  0x0043:  key_value=    14 ;                break;
			case   0x000B:  key_value=   15;               break;
		}	

	}
}


// ѭ�����ô�㺯�����Ѹ�����������
void LCD_DrawRect(u16 ox, u16 oy, u16 ex, u16 ey, uint16_t GRB)
{
    /*
    ox��oy����ʼ��
    ex��ey���յ�
    GRB����ɫ
    */
    u16 i, j;
    for(i=ox; i<=ex; i++){
        for(j=oy; j<=ey; j++){
            LCD_DrawPoint(i, j, GRB);
        }
    }
}

// ȫLCD���ð�ɫ
// ���������κ�������
void clearAll()
{
    LCD_DrawRect(0,0,239,319,WHITE);
}

/* ��LCD��Ļ�������� */
// ���������κ���������
void DrawCBGrid()
{
    /* ��� */

    LCD_DrawRect(19,80,19,240,RED);//�·�����
    LCD_DrawRect(20,80,219,80,RED);//�Ҳ�����
    LCD_DrawRect(20,240,221,240,RED);//�������
    LCD_DrawRect(220,80,221,240,RED);//�Ϸ�����
    /* ���� */
    // ����

    LCD_DrawRect(60,80,61,240,RED);
    LCD_DrawRect(100,80,101,240,RED);
    LCD_DrawRect(140,80,141,240,RED);
    LCD_DrawRect(180,80,181,240,RED);

    // ����
    LCD_DrawRect(20,120,220,121,RED);
    LCD_DrawRect(20,160,220,161,RED);
    LCD_DrawRect(20,200,220,201,RED);
}

/* �����Ӻ��� */
void LCD_DrawAChess(int x, int y, int num, uint16_t GRB)
{
    /*
    x��������CB[5][4]��������
    y��������CB[5][4]��������
    num����ʾ�ڼ������ӣ����ݲ�ͬ���ӻ���ͬ��ͼ�񣬱��ڹ۲죩
    */
    int LCDX = coord[x][y].LCDX;
    int LCDY = coord[x][y].LCDY;
    switch(num){
        case 1:
            // ��1������Ϊ��
            // ��LCD��Ӧ���껭һ����һ����
            // ��LCD��������ȡ���������Ͻ���������
            LCD_DrawRect(LCDX+20, LCDY+10, LCDX+21, LCDY+30, GRB);
            break;
        case 2:
            // 2������
            // ��LCD��Ӧ���껭һ����+����
            LCD_DrawRect(LCDX+20, LCDY+10, LCDX+21, LCDY+30, GRB);
            LCD_DrawRect(LCDX+10, LCDY+20, LCDX+30, LCDY+21, GRB);
            break;
        case 3:
            // 3������
            // ��һ����ܳ����
            LCD_DrawRect(LCDX+20, LCDY+10, LCDX+21, LCDY+30, GRB);
            LCD_DrawRect(LCDX+10, LCDY+15, LCDX+30, LCDY+16, GRB);
            LCD_DrawRect(LCDX+10, LCDY+25, LCDX+30, LCDY+26, GRB);
            break;
        case 4:
            // 4������
            // ��һ����4����
            LCD_DrawRect(LCDX+20, LCDY+30, LCDX+30, LCDY+31, GRB);
            LCD_DrawRect(LCDX+20, LCDY+15, LCDX+21, LCDY+30, GRB);
            LCD_DrawRect(LCDX+10, LCDY+20, LCDX+30, LCDY+21, GRB);
            break;
        case 5:
            // 5������
            // ��һ����5����
            LCD_DrawRect(LCDX+10, LCDY+10, LCDX+11, LCDY+30, GRB);
            LCD_DrawRect(LCDX+10, LCDY+10, LCDX+20, LCDY+11, GRB);
            LCD_DrawRect(LCDX+20, LCDY+10, LCDX+21, LCDY+30, GRB);
            LCD_DrawRect(LCDX+20, LCDY+30, LCDX+30, LCDY+31, GRB);
            LCD_DrawRect(LCDX+30, LCDY+10, LCDX+31, LCDY+30, GRB);
            break;
        case 6:
            // 6������
            // ��һ����b��
            LCD_DrawRect(LCDX+10, LCDY+15, LCDX+20, LCDY+16, GRB);
            LCD_DrawRect(LCDX+20, LCDY+15, LCDX+21, LCDY+25, GRB);
            LCD_DrawRect(LCDX+10, LCDY+25, LCDX+30, LCDY+26, GRB);
            LCD_DrawRect(LCDX+10, LCDY+15, LCDX+11, LCDY+25, GRB);
            break;
        default:
            break;
    }
}

	void DrawCB()
{
    clearAll();
    DrawCBGrid();
    for(i=0; i<6; i++){
        //����PlayerX->chessState[i]��������û���Ե������ú���������ͼ��
        // ���1��������ɫ�����2��������ɫ
        if(Player1->chessState[i] == 1){
            LCD_DrawAChess(Player1->chessX[i], Player1->chessY[i], i+1, BLUE);
        }
        if(Player2->chessState[i] == 1){
            LCD_DrawAChess(Player2->chessX[i], Player2->chessY[i], i+1, GREEN);
        }
    }
}

/* ѡ���� */
void choosechess(int number){
	switch(State){
			case 1: //���1�Ļغ�
					if(Player1->playerState == chooseChess){
							if(Player1->chessState[number-1] == 1){
									// �����1�ĵ�1�����ӻ�û����
									whichChess = number;//����ѡ�����ӱ��
									cx = Player1 -> chessX[number-1];
									cy = Player1 -> chessY[number-1]; //����ȫ�ֱ���cx��cyΪ���1�ĵ�1�����ӵ�����
									Player1 -> playerState = chooseDir; //�����1��״̬����Ϊ��ѡ����
							}
					}
					else{
							//���1�Ѿ����ԣ�����ѡ����
							
					}
					break;
					
			case 2: //���2�Ļغ�
					if(Player2->playerState == chooseChess){
							if(Player2->chessState[number-1] == 1){
									// �����2�ĵ�1�����ӻ�û����
									whichChess = number;//����ѡ�����ӱ��
									cx = Player2 -> chessX[number-1];
									cy = Player2 -> chessY[number-1]; //����ȫ�ֱ���cx��cyΪ���1�ĵ�1�����ӵ�����
									Player2 -> playerState = chooseDir; //�����1��״̬����Ϊ��ѡ����
							}
					}
					else{
							//���2�Ѿ����ԣ�����ѡ����
							
					}
					break;	
					
			case 3:
					//���1��ʤ
					break;
			case 4:
					//���2��ʤ
					break;
			default:
					break;
	}
}


/* ѡ���� ��Ҫ�ж�cx��cy����һ���Ƿ�Խ�硢�Ƿ�����������*/
void choosedir(int direction){
	int tempX,tempY;
	switch(State){
			case 1:
					if(Player1->playerState == chooseDir){
							if(Player1->chessState[whichChess-1] == 1){
									tempX = cx ;
									tempY = cy ;

									gocx = tempX + DIR[direction].dx;
									gocy = tempY + DIR[direction].dy;
									if( CB[gocx][gocy] == 0 && (0<=gocx<=3) && (0<=gocy<=4) ){
										//����Ӧλ��û�����ӣ����������ڣ�������ƶ�
										Player1->playerState = Move;//ѡ�������ӵ����꣬���״̬����Move
									}
									else 
										//�����ƶ�
										Player1->playerState = chooseDir;
							}
					}
					
			case 2:
					if(Player2->playerState == chooseDir){
							if(Player2->chessState[whichChess-1] == 1){
									tempX = cx ;
									tempY = cy ;

									gocx = tempX + DIR[direction].dx;
									gocy = tempY + DIR[direction].dy;
									if( CB[gocx][gocy] == 0 && (0<=gocx<=3) && (0<=gocy<=4) ){
										//����Ӧλ��û�����ӣ����������ڣ�������ƶ�
										Player2->playerState = Move;//ѡ�������ӵ����꣬���״̬����Move
									}
									else 
										//�����ƶ�
										Player2->playerState = chooseDir;
							}
					}


			case 3:

					break;
			
			case 4:

					break;
			default:
					break;
	}
}


/* ȷ������ */
	void acknowledge(){
		switch(State){
			case 1:
				if(Player1->playerState == Move){
					//�ƶ�player1��ĳ����
					LCD_DrawAChess(cx, cy, whichChess, WHITE);
					LCD_DrawAChess(gocx, gocy, whichChess, BLUE);
					//����CB[5][4]
					CB[cx][cy] = 0;//�ƶ�ǰλ�ø�Ϊ0
					CB[gocx][gocy] = 1;//�ƶ���λ�ø�Ϊ1
					//����Player1��Ӧ��chessX��chessY
					Player1 -> chessX[whichChess-1] = gocx;
					Player1 -> chessY[whichChess-1] = gocy;
					//ת��״̬
					State = 2;
					Player1->playerState = chooseChess;
				}
					
			case 2:
				if(Player2->playerState == Move){
					//�ƶ�player1��ĳ����
					LCD_DrawAChess(cx, cy, whichChess, WHITE);
					LCD_DrawAChess(gocx, gocy, whichChess, GREEN);
					//����CB[5][4]
					CB[cx][cy] = 0;//�ƶ�ǰλ�ø�Ϊ0
					CB[gocx][gocy] = 1;//�ƶ���λ�ø�Ϊ1
					//����Player1��Ӧ��chessX��chessY
					Player2 -> chessX[whichChess-1] = gocx;
					Player2 -> chessY[whichChess-1] = gocy;
					//ת��״̬
					State = 1;
					Player2->playerState = chooseChess;
				}
					
			case 3:

					break;
			case 4:

					break;
			default:
					break;
		}
	}

/***********************************����ʱ����*************************************/
vu16 Delay_ms(vu16 Count)			                           			        
{																		
	u16 i=0;
	while(Count--)
	{
		for(i=12000;i>0;i--);
	}
	return Count;
}
/***********************************�ⲿ�жϺ���*************************************/
void EXTI3_IRQHandler (void)
{  
	Delay_ms(20); //��ȥ����
	if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3)==0)
	{
		if(key_value == 2)
		{
			LCD_DrawRect(10, 15, 20, 16, GREEN);
			if(State == 1)
				choosechess(1);
			else
				choosechess(1);
		}
		else if(key_value == 6)
		{
			LCD_DrawRect(25, 15, 35, 16, BLUE);
			if(State == 1)
				choosechess(5);
			else
				choosechess(5);
		}
		else if(key_value == 10)
		{
			LCD_DrawRect(40, 15, 50, 16, RED);
			if(State == 1)
				choosedir(UP);
			else
				choosedir(UP);
		}
		else if(key_value == 14)
		{
			LCD_DrawRect(55, 15, 65, 16, GREEN);
			if(State == 1)
				choosedir(DOWN);
			else
				choosedir(DOWN);
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line3);
}

void EXTI9_5_IRQHandler (void)
{  
	Delay_ms(20); //��ȥ����
	if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6)==0)
	{
		if(key_value == 3)
		{
			LCD_DrawRect(10, 25, 20, 26, BLUE);
			if(State == 1)
				choosechess(2);
			else
				choosechess(2);
		}
		else if(key_value == 7)
		{
			LCD_DrawRect(20, 25, 30, 26, RED);
			if(State == 1)
				choosechess(6);
			else
				choosechess(6);
		}
		else if(key_value == 11)
		{
			LCD_DrawRect(30, 25, 40, 26, BLUE);
			if(State == 1)
				choosedir(LEFT);
			else
				choosedir(LEFT);
		}
		else if(key_value == 15)
		{
			LCD_DrawRect(40, 25, 50, 26, RED);
			if(State == 1)
				choosedir(RIGHT);
			else
				choosedir(RIGHT);
		}

	}
	EXTI_ClearITPendingBit(EXTI_Line6);
}


void EXTI0_IRQHandler (void)
{  
	Delay_ms(20); //��ȥ����
	if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0)==0)
	{
		if(key_value == 4)
		{
			LCD_DrawRect(10, 35, 20, 36, BLUE);
			if(State == 1)
				choosechess(3);
			else
				choosechess(3);
		}
		else if(key_value == 8)
		{
			LCD_DrawRect(70,35,80,36,RED);
			if(State == 1)
				acknowledge();
		}

		
	}
	EXTI_ClearITPendingBit(EXTI_Line0);
}

void EXTI1_IRQHandler (void)
{  
	Delay_ms(20); //��ȥ����
	if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1)==0)
	{
		if(key_value == 5)
		{
			LCD_DrawRect(10, 45, 20, 46, BLUE);
			if(State == 1)
				choosechess(4);
			else
				choosechess(4);
		}
		else if(key_value == 9){
			// ��ʾ״̬
			DrawCB();
			if(State == 1){
				LCD_DrawAChess(180,20,1,BLUE);
				if(Player1->playerState == chooseChess){
					LCD_DrawAChess(140,20,1,BLUE);
				}
				else if(Player1->playerState == chooseDir){
					LCD_DrawAChess(140,20,1,BLUE);
				}
			}
			else if(State == 2){
				LCD_DrawAChess(180,20,2,RED);
				if(Player1->playerState == chooseChess){
					LCD_DrawAChess(140,20,1,RED);
				}
				else if(Player1->playerState == chooseDir){
					LCD_DrawAChess(140,20,1,RED);
				}
			}
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line1);
}

/***********************************дָ���*************************************/
void LCD_IO_WriteReg(uint8_t Reg) 
{
  /* Write 16-bit Index, then Write Reg */
   *(__IO uint16_t *)FSMC_LCD_REG = Reg;
}
/***********************************д���ݺ���*************************************/
void LCD_IO_WriteData(uint16_t Data) 
{
  /* Write 16-bit Reg */
  *(__IO uint16_t *)FSMC_LCD_DATA = Data;
}
/***********************************���ù�꺯��*************************************/
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
	LCD_IO_WriteReg(0x2A);
	LCD_IO_WriteData(Xpos>>8);
	LCD_IO_WriteData(Xpos&0XFF);
	
	LCD_IO_WriteReg(0x2B);
	LCD_IO_WriteData(Ypos>>8);
	LCD_IO_WriteData(Ypos&0XFF);
}
/***********************************��㺯��*************************************/
void LCD_DrawPoint(u16 x,u16 y,uint16_t GRB)
	{
        LCD_SetCursor(x,y);
			  LCD_IO_WriteReg(0x2C);
			 	LCD_IO_WriteData(GRB);
	}
/***********************************ȫ����ɫ����*************************************/

	void LCD_Color_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t GRB)
{  
	uint16_t height,width;
	uint16_t i,j;
	width=ex-sx+1; 			
	height=ey-sy+1;			
 	for(i=0;i<height;i++)
	{
 		LCD_SetCursor(sx,sy+i);
   	
		LCD_IO_WriteReg(0x2C);
		for(j=0;j<width;j++)LCD_IO_WriteData(GRB);
	}		  
}
