
#include "stm32f10x_adc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "misc.h"
#include "lcd.h"
#include "touch.h"

#define LCD_TEAM_NAME_X 20
#define LCD_TEAM_NAME_Y 50
#define LCD_COORD_X_X	40
#define LCD_COORD_X_Y	70
#define LCD_COORD_Y_X	40
#define LCD_COORD_Y_Y	90
#define LCD_LUX_VAL_X	20
#define LCD_LUX_VAL_Y	110

void Init(void);
void RccInit(void);
void GpioInit(void);
void AdcInit(void);
void NvicInit(void);

const int color[12] = {WHITE,CYAN,BLUE,RED,MAGENTA,LGRAY,GREEN,YELLOW,BROWN,BRRED,GRAY};

uint16_t ADC_Value = 0;

int main(){
	uint16_t pos_x, pos_y;
	uint16_t pix_x, pix_y;

	Init();

    LCD_Clear(WHITE);

    // team name
    LCD_ShowString(LCD_TEAM_NAME_X,LCD_TEAM_NAME_Y, "THU_TEAM06", BLUE, WHITE);

    while(1){
		// Todo
        // sensor value

    	// get touch coordinate
        Touch_GetXY(&pos_x, &pos_y, 1);
        Convert_Pos(pos_x, pos_y, &pix_x, &pix_y);
        LCD_DrawCircle(pix_x, pix_y, 3);
    	// show touch coordinate
        LCD_ShowNum(LCD_LUX_VAL_X, LCD_LUX_VAL_Y, ADC_Value, 3, YELLOW, BLACK); //value
        LCD_ShowNum(LCD_COORD_X_X, LCD_COORD_X_Y, pix_x, 3, YELLOW, BLACK);
        LCD_ShowNum(LCD_COORD_Y_X, LCD_COORD_Y_Y, pix_y, 3, YELLOW, BLACK);
		// Todo

    }
}

void Init(void) {
	SystemInit();
	RccInit();
	GpioInit();
	AdcInit();
	NvicInit();

	LCD_Init();
	Touch_Configuration();
	Touch_Adjust();
}

void RccInit(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
}

void GpioInit(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void AdcInit(void) {
    ADC_InitTypeDef ADC_InitStructure;

    // Todo: ADC1 Configuration
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
	
    ADC_Init(ADC1, &ADC_InitStructure);
	// Todo

    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5);
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE); // interrupt enable
    ADC_Cmd(ADC1, ENABLE); // ADC1 enable
    ADC_ResetCalibration(ADC1);

	while(ADC_GetResetCalibrationStatus(ADC1));

	ADC_StartCalibration(ADC1);

	while(ADC_GetCalibrationStatus(ADC1));

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void NvicInit(void){
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


/*
 * ISR
 */

void ADC1_2_IRQHandler() {
	if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET) {
		ADC_Value = ADC_GetConversionValue(ADC1);
		//--- Clear ADC1 AWD pending interrupt bit
		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
	}
}
