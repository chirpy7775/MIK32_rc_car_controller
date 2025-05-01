#include "uart_lib.h"
#include "mik32_hal_gpio.h"
#include "mik32_hal_timer16.h"
#include "mik32_hal_timer32.h"
#include "mik32_hal_pcc.h"

// Настройки сервопривода и ESC
uint16_t SERVO_MIN_US = 500;
uint16_t SERVO_MAX_US = 2500;
uint16_t ESC_MIN_US = 1000;
uint16_t ESC_MAX_US = 2000;

// Структуры таймеров
Timer16_HandleTypeDef htimer16_0 = {.Instance = TIMER16_0};
Timer16_HandleTypeDef htimer16_1 = {.Instance = TIMER16_1};
TIMER32_HandleTypeDef htimer32_1, htimer32_2;
TIMER32_CHANNEL_HandleTypeDef htimer32_1_channel0, htimer32_2_channel0, htimer32_2_channel1;

// Переменные для датчика расстояния
#define ECHO_ALERT_THRESHOLD 2000
#define ECHO_ALERT_FILTER 2
int32_t last_echo = -1;
int echo_alert_cnt = 0;

// Таймаут команд
uint32_t last_command_time;

void SystemClock_Config(void) {
    PCC_InitTypeDef PCC_OscInit = {0};
    PCC_OscInit.OscillatorEnable = PCC_OSCILLATORTYPE_ALL;
    PCC_OscInit.FreqMon.OscillatorSystem = PCC_OSCILLATORTYPE_OSC32M;
    PCC_OscInit.FreqMon.ForceOscSys = PCC_FORCE_OSC_SYS_UNFIXED;
    PCC_OscInit.FreqMon.Force32KClk = PCC_FREQ_MONITOR_SOURCE_OSC32K;
    PCC_OscInit.AHBDivider = 0;
    PCC_OscInit.APBMDivider = 0;
    PCC_OscInit.APBPDivider = 0;
    PCC_OscInit.HSI32MCalibrationValue = 128;
    PCC_OscInit.LSI32KCalibrationValue = 8;
    HAL_PCC_Config(&PCC_OscInit);
}

void GPIO_Init() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_PCC_GPIO_0_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_OUTPUT;
    GPIO_InitStruct.Pull = HAL_GPIO_PULL_NONE;
    HAL_GPIO_Init(GPIO_0, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIO_0, GPIO_PIN_9, GPIO_PIN_LOW);
}

void Timer16_Init(Timer16_HandleTypeDef* h) {
    h->Clock.Source = TIMER16_SOURCE_INTERNAL_SYSTEM;
    h->CountMode = TIMER16_COUNTMODE_INTERNAL;
    h->Clock.Prescaler = TIMER16_PRESCALER_32;
    h->Preload = TIMER16_PRELOAD_AFTERWRITE;
    h->Trigger.ActiveEdge = TIMER16_TRIGGER_ACTIVEEDGE_SOFTWARE;
    h->Trigger.TimeOut = TIMER16_TIMEOUT_DISABLE;
    h->Filter.ExternalClock = TIMER16_FILTER_NONE;
    h->Filter.Trigger = TIMER16_FILTER_NONE;
    h->EncoderMode = TIMER16_ENCODER_DISABLE;
    h->Waveform.Enable = TIMER16_WAVEFORM_GENERATION_ENABLE;
    h->Waveform.Polarity = TIMER16_WAVEFORM_POLARITY_INVERTED;
    HAL_Timer16_Init(h);
}

void Timer32_1_Init(void) {
    htimer32_1.Instance = TIMER32_1;
    htimer32_1.Top = 19999;
    htimer32_1.Clock.Source = TIMER32_SOURCE_PRESCALER;
    htimer32_1.Clock.Prescaler = 31;
    htimer32_1.CountMode = TIMER32_COUNTMODE_FORWARD;
    HAL_Timer32_Init(&htimer32_1);

    htimer32_1_channel0.TimerInstance = htimer32_1.Instance;
    htimer32_1_channel0.ChannelIndex = TIMER32_CHANNEL_0;
    htimer32_1_channel0.PWM_Invert = TIMER32_CHANNEL_NON_INVERTED_PWM;
    htimer32_1_channel0.Mode = TIMER32_CHANNEL_MODE_PWM;
    htimer32_1_channel0.OCR = 10000;
    HAL_Timer32_Channel_Init(&htimer32_1_channel0);
}

void Timer32_2_Init(void) {
    htimer32_2.Instance = TIMER32_2;
    htimer32_2.Top = ~0U;
    htimer32_2.Clock.Source = TIMER32_SOURCE_PRESCALER;
    htimer32_2.Clock.Prescaler = 31;
    htimer32_2.CountMode = TIMER32_COUNTMODE_FORWARD;
    HAL_Timer32_Init(&htimer32_2);

    htimer32_2_channel0.TimerInstance = htimer32_2.Instance;
    htimer32_2_channel0.ChannelIndex = TIMER32_CHANNEL_0;
    htimer32_2_channel0.Mode = TIMER32_CHANNEL_MODE_CAPTURE;
    htimer32_2_channel0.CaptureEdge = TIMER32_CHANNEL_CAPTUREEDGE_RISING;
    HAL_Timer32_Channel_Init(&htimer32_2_channel0);

    htimer32_2_channel1.TimerInstance = htimer32_2.Instance;
    htimer32_2_channel1.ChannelIndex = TIMER32_CHANNEL_1;
    htimer32_2_channel1.Mode = TIMER32_CHANNEL_MODE_CAPTURE;
    htimer32_2_channel1.CaptureEdge = TIMER32_CHANNEL_CAPTUREEDGE_FALLING;
    HAL_Timer32_Channel_Init(&htimer32_2_channel1);
}

void Servo_SetAngle(uint16_t angle_deg) {
    angle_deg = angle_deg > 180 ? 180 : angle_deg;
    uint16_t pulse_us = SERVO_MIN_US + (angle_deg * (SERVO_MAX_US - SERVO_MIN_US) / 180);
    htimer16_0.Instance->CMP = pulse_us;
}

void ESC_SetThrottle(uint16_t throttle_percent) {
    throttle_percent = throttle_percent > 100 ? 100 : throttle_percent;
    uint16_t pulse_us = ESC_MIN_US + (throttle_percent * (ESC_MAX_US - ESC_MIN_US) / 100);
    htimer16_1.Instance->CMP = pulse_us;
}

uint16_t simple_atoi(uint8_t *buf, uint8_t len) {
    uint16_t res = 0;
    while(len--) {
        if(*buf >= '0' && *buf <= '9')
            res = res * 10 + (*buf - '0');
        buf++;
    }
    return res;
}

int main(void) {
    uint8_t uart_buf[16], idx = 0;
    uint32_t cmd_value;

    SystemClock_Config();
    GPIO_Init();
    HAL_Time_TIM32_Init(TIMER32_0);
    last_command_time = HAL_Time_TIM32_Millis();

    if(!UART_Init(UART_1, 3333, 
                 UART_CONTROL1_TE_M | UART_CONTROL1_RE_M | UART_CONTROL1_M_8BIT_M, 
                 0, 0)) {
        while(1);
    }

    Timer16_Init(&htimer16_0);
    HAL_Timer16_StartPWM(&htimer16_0, 20000, 1500);
    htimer16_0.Instance->CFGR |= TIMER16_CFGR_PRELOAD_M;

    Timer16_Init(&htimer16_1);
    HAL_Timer16_StartPWM(&htimer16_1, 20000, 1000);
    htimer16_1.Instance->CFGR |= TIMER16_CFGR_PRELOAD_M;

    Timer32_1_Init();
    Timer32_2_Init();
    HAL_Timer32_Channel_Enable(&htimer32_1_channel0);
    HAL_Timer32_Start(&htimer32_1);
    HAL_Timer32_Channel_Enable(&htimer32_2_channel0);
    HAL_Timer32_Channel_Enable(&htimer32_2_channel1);
    HAL_Timer32_Start(&htimer32_2);

    while(1) {
        // Таймаут управления
        if(HAL_Time_TIM32_Millis() - last_command_time > 1500) {
            ESC_SetThrottle(0);
        }

        // Обработка UART
        if(!UART_IsRxFifoEmpty(UART_1)) {
            uint16_t c = UART_ReadByte(UART_1);
            UART_WriteByte(UART_1, c);

            if(c == '\r' || c == '\n') {
                if(idx > 0) {
                    switch(uart_buf[0]) {
                        case 's': // Установка угла сервы
                            cmd_value = simple_atoi(uart_buf+1, idx-1);
                            Servo_SetAngle(cmd_value);
                            break;
                            
                        case 't': // Установка тяги ESC
                            cmd_value = simple_atoi(uart_buf+1, idx-1);
                            ESC_SetThrottle(cmd_value);
                            break;
                            
                        case 'm': // Настройка SERVO_MIN
                            SERVO_MIN_US = simple_atoi(uart_buf+1, idx-1);
                            break;
                            
                        case 'M': // Настройка SERVO_MAX
                            SERVO_MAX_US = simple_atoi(uart_buf+1, idx-1);
                            break;
                            
                        case 'e': // Настройка ESC_MIN
                            ESC_MIN_US = simple_atoi(uart_buf+1, idx-1);
                            break;
                            
                        case 'E': // Настройка ESC_MAX
                            ESC_MAX_US = simple_atoi(uart_buf+1, idx-1);
                            break;
                    }
                    last_command_time = HAL_Time_TIM32_Millis();
                }
                idx = 0;
                continue;
            }
            
            if(idx < sizeof(uart_buf)-1) {
                uart_buf[idx++] = c;
            }
        }

        // Обработка эхо-сигнала
        if(last_echo < 0) {
            if(htimer32_2.Instance->INT_FLAGS & TIMER32_INT_IC_M(1)) {
                last_echo = 0;
                htimer32_2.Instance->INT_FLAGS = TIMER32_INT_IC_M(1);
            }
        }
        else if((htimer32_2.Instance->INT_FLAGS & TIMER32_INT_IC_M(0)) &&
                (htimer32_2.Instance->INT_FLAGS & TIMER32_INT_IC_M(1))) {
            last_echo = htimer32_2.Instance->CHANNELS[1].ICR - 
                        htimer32_2.Instance->CHANNELS[0].ICR;
            htimer32_2.Instance->INT_FLAGS = TIMER32_INT_IC_M(0) | TIMER32_INT_IC_M(1);

            if(last_echo > 0 && last_echo < ECHO_ALERT_THRESHOLD) {
                if(++echo_alert_cnt >= ECHO_ALERT_FILTER) {
                    ESC_SetThrottle(0);
                    HAL_GPIO_WritePin(GPIO_0, GPIO_PIN_9, GPIO_PIN_HIGH);
                }
            } 
            else {
                echo_alert_cnt = 0;
                HAL_GPIO_WritePin(GPIO_0, GPIO_PIN_9, GPIO_PIN_LOW);
            }
        }
    }
}