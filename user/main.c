#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include "lcd.h"
#include "touch.h"
#include <stdio.h>
#include <math.h>

#define LCD_TEAM_NAME_X 20
#define LCD_TEAM_NAME_Y 50
#define LCD_SENSOR1_Y   100
#define LCD_SENSOR2_Y   120
#define LCD_SENSOR3_Y   140
#define LCD_SENSOR4_Y   160
#define LCD_POWER_Y     200 
#define LCD_VAL_X       100

// 태양광 추적 설정
#define THRESHOLD 50    
#define STEP_ANGLE 30   
#define SERVO_MIN 500   
#define SERVO_MAX 2500  

// 발전량 계산 상수
#define PANEL_EFFICIENCY 0.18f  // 효율 eta = 0.18
#define PANEL_AREA       0.01f  // 패널 면적 A (0.01m^2 = 10cm x 10cm 가정)

// 센서 매핑
#define ADC_EAST  ADC_Values[0] // PC0
#define ADC_WEST  ADC_Values[1] // PC1
#define ADC_SOUTH ADC_Values[2] // PC2
#define ADC_NORTH ADC_Values[3] // PC3
#define ADC_SOLAR ADC_Values[4] // PC4 

// 함수 원형 선언
void Init(void);
void RCC_Configure(void);
void GPIO_Configure(void);
void DMA_Configure(void);
void ADC_Configure(void);
void TIM_Configure(void);
void USART1_Init(void);
void USART2_Init(void);
void UART_SendString(USART_TypeDef* USARTx, char* str);
void Servo_SetAngle(uint16_t pulseX, uint16_t pulseY);
void Delay(void);

// S1~S4(방향), S5(발전량)
volatile uint16_t ADC_Values[5] = {0, 0, 0, 0, 0}; 
char msg_buf[150]; 

// 현재 모터 위치
int servo_pos_x = 1500; 
int servo_pos_y = 1500;

int main(){
    uint16_t pos_x, pos_y;
    uint16_t pix_x, pix_y;
    
    int delta_x = 0;
    int delta_y = 0;
    char dir_x[10] = "Hold";
    char dir_y[10] = "Hold";

    // 전력 계산 변수
    float voltage = 0.0f;
    float lux = 0.0f;
    float g_est = 0.0f;     // 일사량
    float p_est = 0.0f;     // 추정 전력
    float p_fixed = 0.0f;   // 고정식 추정 전력
    float gain = 0.0f;      // 효율 이득 (%)

    Init(); 
    Servo_SetAngle(servo_pos_x, servo_pos_y);

    LCD_Clear(WHITE);
    LCD_ShowString(LCD_TEAM_NAME_X, LCD_TEAM_NAME_Y, "THU_TEAM06", BLUE, WHITE);
    
    LCD_ShowString(20, LCD_SENSOR1_Y, "E(S1):", BLACK, WHITE);
    LCD_ShowString(20, LCD_SENSOR2_Y, "W(S2):", BLACK, WHITE);
    LCD_ShowString(20, LCD_SENSOR3_Y, "S(S3):", BLACK, WHITE);
    LCD_ShowString(20, LCD_SENSOR4_Y, "N(S4):", BLACK, WHITE);
    
    // 전력 정보 라벨
    LCD_ShowString(20, LCD_POWER_Y,     "P_est:", BLACK, WHITE);
    LCD_ShowString(20, LCD_POWER_Y + 20, "Gain :", BLACK, WHITE);

    while(1){
        // 1) 태양광 추적 (기존 로직)
        delta_x = (int)ADC_EAST - (int)ADC_WEST;
        delta_y = (int)ADC_SOUTH - (int)ADC_NORTH;

        // X축 제어
        if (delta_x < -THRESHOLD) { 
            sprintf(dir_x, "Go East");
            servo_pos_x -= STEP_ANGLE; 
        } else if (delta_x > THRESHOLD) {
            sprintf(dir_x, "Go West");
            servo_pos_x += STEP_ANGLE;
        } else {
            sprintf(dir_x, "Hold   ");
        }

        // Y축 제어
        if (delta_y < -THRESHOLD) {
            sprintf(dir_y, "Go South");
            servo_pos_y -= STEP_ANGLE; 
        } else if (delta_y > THRESHOLD) {
            sprintf(dir_y, "Go North");
            servo_pos_y += STEP_ANGLE;
        } else {
            sprintf(dir_y, "Hold    ");
        }

        // 모터 범위 제한 및 구동
        if (servo_pos_x < SERVO_MIN) servo_pos_x = SERVO_MIN;
        if (servo_pos_x > SERVO_MAX) servo_pos_x = SERVO_MAX;
        if (servo_pos_y < SERVO_MIN) servo_pos_y = SERVO_MIN;
        if (servo_pos_y > SERVO_MAX) servo_pos_y = SERVO_MAX;
        Servo_SetAngle(servo_pos_x, servo_pos_y);

        // 2) 발전량 및 효율 계산         
        // 1. ADC -> 전압 변환 
        voltage = (float)ADC_SOLAR * (3.3f / 4095.0f);

        // 2. 전압 -> Lux 변환 (3.3V일 때 100,000 Lux라고 가정하여 맵핑)
        // 실제 센서 데이터시트에 맞춰 조정 필요
        lux = voltage * (100000.0f / 3.3f); 

        // 3. Lux -> 일사량 (G_est) 변환
        // G_est = Lux / 100  (100,000 lux = 1000 W/m^2)
        g_est = lux / 100.0f;

        // 4. 추정 전력 (P_est) 계산
        // P_est = eta * A * G_POA (여기서는 G_est를 사용)
        p_est = PANEL_EFFICIENCY * PANEL_AREA * g_est;

        // 5. 이득 (Gain%) 계산
        // Gain% = ((Tracking - Fixed) / Fixed) * 100
        // 실제 고정형 패널이 없으므로, 현재 트래킹 값의 70%가 고정형 효율이라고 가정하여 계산
        p_fixed = p_est * 0.7f; 
        
        if (p_fixed > 0) {
            gain = ((p_est - p_fixed) / p_fixed) * 100.0f;
        } else {
            gain = 0.0f;
        }

        // 3) LCD 및 UART 출력
        // 센서값 출력
        LCD_ShowNum(LCD_VAL_X, LCD_SENSOR1_Y, ADC_EAST, 4, YELLOW, BLACK);
        LCD_ShowNum(LCD_VAL_X, LCD_SENSOR2_Y, ADC_WEST, 4, YELLOW, BLACK);
        LCD_ShowNum(LCD_VAL_X, LCD_SENSOR3_Y, ADC_SOUTH, 4, YELLOW, BLACK);
        LCD_ShowNum(LCD_VAL_X, LCD_SENSOR4_Y, ADC_NORTH, 4, YELLOW, BLACK);
        
        // 전력값 출력 (소수점 표현을 위해 정수부/소수부 분리 표시)
        LCD_ShowNum(LCD_VAL_X, LCD_POWER_Y, (int)p_est, 1, RED, WHITE);
        LCD_ShowString(LCD_VAL_X + 10, LCD_POWER_Y, ".", RED, WHITE);
        LCD_ShowNum(LCD_VAL_X + 20, LCD_POWER_Y, (int)((p_est - (int)p_est)*100), 2, RED, WHITE);
        LCD_ShowString(LCD_VAL_X + 50, LCD_POWER_Y, "W", RED, WHITE);

        // Gain 출력
        LCD_ShowNum(LCD_VAL_X, LCD_POWER_Y + 20, (int)gain, 3, RED, WHITE);
        LCD_ShowString(LCD_VAL_X + 40, LCD_POWER_Y + 20, "%", RED, WHITE);
        
        // UART(Putty Bluetooth) 전송
        sprintf(msg_buf, "Power: %d.%03d W | Gain: %d.%d %% | X:%s Y:%s\r\n", 
                (int)p_est, (int)((p_est - (int)p_est)*1000), 
                (int)gain, (int)((gain - (int)gain)*10),
                dir_x, dir_y);
        
        UART_SendString(USART2, msg_buf); 
        UART_SendString(USART1, msg_buf); 

         // 4) 터치 처리
        Touch_GetXY(&pos_x, &pos_y, 1);
        Convert_Pos(pos_x, pos_y, &pix_x, &pix_y);
        if(pix_x > 0 && pix_y > 0) LCD_DrawCircle(pix_x, pix_y, 3);
        
        Delay(); 
    }
}

void Init(void) {
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    DMA_Configure();   
    ADC_Configure();
    TIM_Configure(); 
    USART1_Init();
    USART2_Init();
    
    LCD_Init();
    Touch_Configuration();
    Touch_Adjust();
}

void RCC_Configure(void) {
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
}

void GPIO_Configure(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    // 1. 센서 5개 (PC0~PC4) -> Analog Input
    // PC0~PC3 방향 센서, PC4 태양광 발전 센서
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // 2. 모터 PWM (PB0, PB1)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 3. USART1 (PA9, PA10)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 4. USART2 (PA2, PA3)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void DMA_Configure(void) {
    DMA_InitTypeDef DMA_Instructure;
    DMA_DeInit(DMA1_Channel1);
    DMA_Instructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    DMA_Instructure.DMA_MemoryBaseAddr = (uint32_t)ADC_Values;
    DMA_Instructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    // 센서 5개이므로 BufferSize = 5
    DMA_Instructure.DMA_BufferSize = 5; 
    DMA_Instructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_Instructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_Instructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_Instructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_Instructure.DMA_Mode = DMA_Mode_Circular;
    DMA_Instructure.DMA_Priority = DMA_Priority_High;
    DMA_Instructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_Instructure);
    DMA_Cmd(DMA1_Channel1, ENABLE);
}

void ADC_Configure(void) {
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    // 채널 5개
    ADC_InitStructure.ADC_NbrOfChannel = 5; 
    ADC_Init(ADC1, &ADC_InitStructure);

    // PC0->PC1->PC2->PC3->PC4 순서
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_55Cycles5); // PC0
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_55Cycles5); // PC1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_55Cycles5); // PC2
    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4, ADC_SampleTime_55Cycles5); // PC3
    ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 5, ADC_SampleTime_55Cycles5); // PC4 

    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void TIM_Configure(void) {
    TIM_TimeBaseInitTypeDef TIM3_InitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    uint16_t prescale = (uint16_t) (SystemCoreClock / 1000000) - 1; 

    TIM3_InitStructure.TIM_Period = 20000 - 1; 
    TIM3_InitStructure.TIM_Prescaler = prescale;
    TIM3_InitStructure.TIM_ClockDivision = 0;
    TIM3_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM3_InitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1500; 

    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

void Servo_SetAngle(uint16_t pulseX, uint16_t pulseY) {
    TIM_SetCompare3(TIM3, pulseX);
    TIM_SetCompare4(TIM3, pulseY);
}

void USART1_Init(void) {
    USART_InitTypeDef USART1_InitStructure;
    USART1_InitStructure.USART_BaudRate = 9600; 
    USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART1_InitStructure.USART_Parity = USART_Parity_No;
    USART1_InitStructure.USART_StopBits = USART_StopBits_1;
    USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART1_InitStructure);
    USART_Cmd(USART1, ENABLE);
}

void USART2_Init(void) {
    USART_InitTypeDef USART2_InitStructure;
    USART2_InitStructure.USART_BaudRate = 9600; 
    USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART2_InitStructure.USART_Parity = USART_Parity_No;
    USART2_InitStructure.USART_StopBits = USART_StopBits_1;
    USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART2, &USART2_InitStructure);
    USART_Cmd(USART2, ENABLE);
}

void UART_SendString(USART_TypeDef* USARTx, char* str) {
    while(*str) {
        while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
        USART_SendData(USARTx, *str++);
    }
}

void Delay(void) {
    volatile int i;
    for (i = 0; i < 1000000; i++); 
}