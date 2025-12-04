#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "lcd.h"
#include "touch.h"
#include <stdio.h>

#define LCD_TEAM_NAME_X 20
#define LCD_TEAM_NAME_Y 20
#define LCD_LUX_VAL_X   10
#define LCD_LUX_VAL_Y   50
#define LCD_STATUS_X    10
#define LCD_STATUS_Y    150

#define LCD_BTN_X       200
#define LCD_BTN_Y       20
#define LCD_BTN_W       100
#define LCD_BTN_H       40

#define ADC_SAMPLE_COUNT 8
#define SENSOR_THRESHOLD 100
#define SERVO_STEP_US    40
#define SERVO_MIN_US     600
#define SERVO_MAX_US     2400
#define SERVO_CENTER_US  1500
#define SERVO_PERIOD_US  20000
#define BT_BAUDRATE      9600

enum {
    LDR_NORTH = 0,
    LDR_SOUTH,
    LDR_WEST,
    LDR_EAST,
    LDR_COUNT
};

static const uint8_t adc_channel_map[LDR_COUNT] = {
    ADC_Channel_0, // North  -> PA0
    ADC_Channel_1, // South  -> PA1
    ADC_Channel_2, // West   -> PA2
    ADC_Channel_3  // East   -> PA3
};

void Init(void);
void RccInit(void);
void GpioInit(void);
void AdcInit(void);
void ServoPwmInit(void);
void UsartInit(void);
void DisplayLuxValues(const uint16_t *lux_values);
void TrackSun(const uint16_t *lux_values);
uint16_t ReadAdcChannel(uint8_t channel);
void ReadAllSensors(uint16_t *lux_values);
void DelayMs(uint32_t ms);
void DrawStartButton(uint8_t tracking_on);
uint8_t HandleButtonTouch(void);
void DisplayAverage(uint16_t average_lux, int16_t dx, int16_t dy, uint8_t balanced);
void DisplayIdle(void);
void SendTrackingData(uint16_t average_lux, int16_t dx, int16_t dy, uint8_t balanced);
void UsartSendString(const char *str);

static uint16_t servo_az_pulse = SERVO_CENTER_US;
static uint16_t servo_el_pulse = SERVO_CENTER_US;

int main() {
    uint16_t lux_values[LDR_COUNT];
    uint8_t tracking_on = 0;

    Init();

    LCD_Clear(WHITE);
    LCD_ShowString(LCD_TEAM_NAME_X, LCD_TEAM_NAME_Y, "THU_TEAM06", BLUE, WHITE);
    LCD_ShowString(LCD_LUX_VAL_X, LCD_LUX_VAL_Y - 20, "N  S  W  E", BLUE, WHITE);
    DrawStartButton(tracking_on);

    while (1) {
        tracking_on ^= HandleButtonTouch();
        DrawStartButton(tracking_on);

        if (tracking_on) {
            ReadAllSensors(lux_values);
            DisplayLuxValues(lux_values);
            TrackSun(lux_values);
        } else {
            DisplayIdle();
        }

        DelayMs(500);
    }
}

void Init(void) {
    SystemInit();
    RccInit();
    GpioInit();
    AdcInit();
    ServoPwmInit();
    UsartInit();

    LCD_Init();
    Touch_Configuration();
    Touch_Adjust();
}

void RccInit(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
}

void GpioInit(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    // LDR sensors: PA0~PA3 as analog inputs
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Servo outputs: PB6 (azimuth) / PB7 (elevation) as PWM outputs
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // USART1 TX (PA9) / RX (PA10) for HC-06
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void AdcInit(void) {
    ADC_InitTypeDef ADC_InitStructure;

    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;

    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1)) {
    }

    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1)) {
    }
}

void UsartInit(void) {
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = BT_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}

void ServoPwmInit(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / 1000000) - 1; // 1 MHz clock
    TIM_TimeBaseStructure.TIM_Period = SERVO_PERIOD_US - 1;                // 20 ms period
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    // Azimuth - TIM4 Channel 1 (PB6)
    TIM_OCInitStructure.TIM_Pulse = SERVO_CENTER_US;
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

    // Elevation - TIM4 Channel 2 (PB7)
    TIM_OCInitStructure.TIM_Pulse = SERVO_CENTER_US;
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

uint16_t ReadAdcChannel(uint8_t channel) {
    uint32_t total = 0;

    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_239Cycles5);

    for (int sample = 0; sample < ADC_SAMPLE_COUNT; sample++) {
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {
        }
        total += ADC_GetConversionValue(ADC1);
    }

    return (uint16_t)(total / ADC_SAMPLE_COUNT);
}

void ReadAllSensors(uint16_t *lux_values) {
    for (int idx = 0; idx < LDR_COUNT; idx++) {
        lux_values[idx] = ReadAdcChannel(adc_channel_map[idx]);
    }
}

static uint16_t ClampPulse(uint16_t pulse) {
    if (pulse < SERVO_MIN_US) {
        return SERVO_MIN_US;
    }
    if (pulse > SERVO_MAX_US) {
        return SERVO_MAX_US;
    }
    return pulse;
}

void TrackSun(const uint16_t *lux_values) {
    int16_t dx = (int16_t)lux_values[LDR_EAST] - (int16_t)lux_values[LDR_WEST];
    int16_t dy = (int16_t)lux_values[LDR_SOUTH] - (int16_t)lux_values[LDR_NORTH];
    uint16_t average = (uint16_t)((lux_values[LDR_EAST] + lux_values[LDR_WEST] + lux_values[LDR_NORTH] + lux_values[LDR_SOUTH]) / 4);

    if (dx > SENSOR_THRESHOLD) {
        servo_az_pulse += SERVO_STEP_US;
    } else if (dx < -SENSOR_THRESHOLD) {
        servo_az_pulse -= SERVO_STEP_US;
    }

    if (dy > SENSOR_THRESHOLD) {
        // South brighter than North -> tilt north (decrease elevation pulse)
        servo_el_pulse -= SERVO_STEP_US;
    } else if (dy < -SENSOR_THRESHOLD) {
        servo_el_pulse += SERVO_STEP_US;
    }

    servo_az_pulse = ClampPulse(servo_az_pulse);
    servo_el_pulse = ClampPulse(servo_el_pulse);

    TIM_SetCompare1(TIM4, servo_az_pulse);
    TIM_SetCompare2(TIM4, servo_el_pulse);

    uint8_t balanced = (dx > -SENSOR_THRESHOLD && dx < SENSOR_THRESHOLD) && (dy > -SENSOR_THRESHOLD && dy < SENSOR_THRESHOLD);
    DisplayAverage(average, dx, dy, balanced);
    SendTrackingData(average, dx, dy, balanced);
}

void DisplayLuxValues(const uint16_t *lux_values) {
    LCD_ShowNum(LCD_LUX_VAL_X, LCD_LUX_VAL_Y, lux_values[LDR_NORTH], 4, BLACK, WHITE);
    LCD_ShowNum(LCD_LUX_VAL_X + 40, LCD_LUX_VAL_Y, lux_values[LDR_SOUTH], 4, BLACK, WHITE);
    LCD_ShowNum(LCD_LUX_VAL_X + 80, LCD_LUX_VAL_Y, lux_values[LDR_WEST], 4, BLACK, WHITE);
    LCD_ShowNum(LCD_LUX_VAL_X + 120, LCD_LUX_VAL_Y, lux_values[LDR_EAST], 4, BLACK, WHITE);
}

void DrawStartButton(uint8_t tracking_on) {
    uint16_t color = tracking_on ? GREEN : GRAY;
    LCD_DrawRectangle(LCD_BTN_X, LCD_BTN_Y, LCD_BTN_X + LCD_BTN_W, LCD_BTN_Y + LCD_BTN_H);
    LCD_ShowString(LCD_BTN_X + 10, LCD_BTN_Y + 12, tracking_on ? "STOP" : "START", color, WHITE);
}

uint8_t HandleButtonTouch(void) {
    uint16_t pos_x, pos_y;
    uint16_t pix_x, pix_y;
    static uint8_t pressed_last = 0;

    Touch_GetXY(&pos_x, &pos_y, 1);
    Convert_Pos(pos_x, pos_y, &pix_x, &pix_y);

    uint8_t within = (pix_x >= LCD_BTN_X && pix_x <= LCD_BTN_X + LCD_BTN_W && pix_y >= LCD_BTN_Y && pix_y <= LCD_BTN_Y + LCD_BTN_H);

    if (within && !pressed_last) {
        pressed_last = 1;
        return 1;
    }

    pressed_last = within;
    return 0;
}

void DisplayAverage(uint16_t average_lux, int16_t dx, int16_t dy, uint8_t balanced) {
    LCD_ShowString(LCD_STATUS_X, LCD_STATUS_Y - 30, "AVG:", BLUE, WHITE);
    LCD_ShowNum(LCD_STATUS_X + 40, LCD_STATUS_Y - 30, average_lux, 4, BLACK, WHITE);

    LCD_ShowString(LCD_STATUS_X, LCD_STATUS_Y, "dX:", BLUE, WHITE);
    LCD_ShowSignedNum(LCD_STATUS_X + 40, LCD_STATUS_Y, dx, 4, BLACK, WHITE);
    LCD_ShowString(LCD_STATUS_X + 90, LCD_STATUS_Y, "dY:", BLUE, WHITE);
    LCD_ShowSignedNum(LCD_STATUS_X + 130, LCD_STATUS_Y, dy, 4, BLACK, WHITE);

    if (balanced) {
        LCD_ShowString(LCD_STATUS_X, LCD_STATUS_Y + 30, "Balanced: Stop ", GREEN, WHITE);
    } else {
        LCD_ShowString(LCD_STATUS_X, LCD_STATUS_Y + 30, "Adjusting...   ", BLUE, WHITE);
    }
}

void DisplayIdle(void) {
    LCD_ShowString(LCD_STATUS_X, LCD_STATUS_Y - 30, "AVG:", BLUE, WHITE);
    LCD_ShowNum(LCD_STATUS_X + 40, LCD_STATUS_Y - 30, 0, 4, BLACK, WHITE);
    LCD_ShowString(LCD_STATUS_X, LCD_STATUS_Y, "dX:", BLUE, WHITE);
    LCD_ShowSignedNum(LCD_STATUS_X + 40, LCD_STATUS_Y, 0, 4, BLACK, WHITE);
    LCD_ShowString(LCD_STATUS_X + 90, LCD_STATUS_Y, "dY:", BLUE, WHITE);
    LCD_ShowSignedNum(LCD_STATUS_X + 130, LCD_STATUS_Y, 0, 4, BLACK, WHITE);
    LCD_ShowString(LCD_STATUS_X, LCD_STATUS_Y + 30, "Waiting start ", GRAY, WHITE);
}

void SendTrackingData(uint16_t average_lux, int16_t dx, int16_t dy, uint8_t balanced) {
    char msg[96];
    int len = snprintf(msg, sizeof(msg), "AVG:%4u dX:%4d dY:%4d AZ:%4u EL:%4u %s\r\n", average_lux, dx, dy, servo_az_pulse, servo_el_pulse, balanced ? "BAL" : "MOVE");

    if (len > 0) {
        UsartSendString(msg);
    }
}

void UsartSendString(const char *str) {
    while (*str) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
        }
        USART_SendData(USART1, (uint16_t)(*str));
        str++;
    }
}

void DelayMs(uint32_t ms) {
    while (ms--) {
        SysTick->LOAD = (SystemCoreClock / 1000) - 1; // 1 ms tick
        SysTick->VAL = 0;
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) {
        }
        SysTick->CTRL = 0;
    }
}
