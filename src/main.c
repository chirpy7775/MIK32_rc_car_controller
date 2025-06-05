#include "uart_lib.h"
#include "xprintf.h"

#include "mik32_hal_gpio.h"
#include "mik32_hal_timer16.h"
#include "mik32_hal_timer32.h"
#include "mik32_hal_pcc.h"
#include "mik32_hal_adc.h"
#include "mik32_hal_irq.h"

#include <stdint.h>
#include <string.h>

/* ================= НАСТРОЙКИ ================= */
static struct {
    uint16_t servo_min_us;
    uint16_t servo_max_us;
    uint8_t  esc_reverse_enabled;
    uint16_t esc_reverse_min_us;
    uint16_t esc_neutral_min_us;
    uint16_t esc_neutral_max_us;
    uint16_t esc_forward_max_us;
    uint16_t esc_neutral_us;
    uint32_t echo_threshold_us;
    uint8_t  echo_filter;
    uint32_t command_timeout_ms;
} cfg = {
    .servo_min_us         = 500,
    .servo_max_us         = 2500,
    .esc_reverse_enabled  = 1,
    .esc_reverse_min_us   = 1000,
    .esc_neutral_min_us   = 1465,
    .esc_neutral_max_us   = 1483,
    .esc_forward_max_us   = 2000,
    .esc_neutral_us       = (1465 + 1483) / 2,
    .echo_threshold_us    = 2000,
    .echo_filter          = 2,
    .command_timeout_ms   = 1500,
};

Timer16_HandleTypeDef         htimer16_0 = { .Instance = TIMER16_0 };
Timer16_HandleTypeDef         htimer16_1 = { .Instance = TIMER16_1 };
TIMER32_HandleTypeDef         htimer32_1, htimer32_2;
TIMER32_CHANNEL_HandleTypeDef htimer32_1_ch0, htimer32_2_ch0, htimer32_2_ch1;
ADC_HandleTypeDef             hadc;

/* ================= КОЛЬЦЕВОЙ БУФЕР ПРИЁМА ================= */
#define RX_BUF_SZ 128
static volatile uint8_t  rx_buf[RX_BUF_SZ];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
static volatile uint32_t rx_overruns = 0;

/* ================= КОЛЬЦЕВОЙ БУФЕР ПЕРЕДАЧИ =============== */
#define TX_BUF_SZ 128
static volatile uint8_t  tx_buf[TX_BUF_SZ];
static volatile uint16_t tx_head = 0;
static volatile uint16_t tx_tail = 0;
static volatile uint32_t tx_overruns = 0;

/* ================= ПРОЧИЕ ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ================= */
static int32_t  last_echo         = -1;
static int      echo_alert_cnt    =  0;
static uint32_t last_command_time =  0;
static int16_t  last_throttle_pct =  0;
static uint32_t last_adc_time     =  0;

/* ================= ПРОТОТИПЫ ================= */
void SystemClock_Config(void);
static void GPIO_Init(void);
static void Timer16_CommonInit(Timer16_HandleTypeDef *h);
static void Timer32_1_Init(void);
static void Timer32_2_Init(void);
static void ADC_Init(void);
uint16_t simple_atoi(uint8_t *buf, uint8_t len);
void Servo_SetAngle(uint16_t deg);
void ESC_SetThrottle(int16_t pct);
static void UART_TxTask(void);

/* ================= СИСТЕМНЫЕ НАСТРОЙКИ ================= */
void SystemClock_Config(void)
{
    PCC_InitTypeDef p = {0};
    p.OscillatorEnable         = PCC_OSCILLATORTYPE_ALL;
    p.FreqMon.OscillatorSystem = PCC_OSCILLATORTYPE_OSC32M;
    p.FreqMon.ForceOscSys      = PCC_FORCE_OSC_SYS_UNFIXED;
    p.FreqMon.Force32KClk      = PCC_FREQ_MONITOR_SOURCE_OSC32K;
    p.AHBDivider  = 0;
    p.APBMDivider = 0;
    p.APBPDivider = 0;
    p.HSI32MCalibrationValue   = 128;
    p.LSI32KCalibrationValue   = 8;
    HAL_PCC_Config(&p);
}

static void GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_PCC_GPIO_0_CLK_ENABLE();
    __HAL_PCC_GPIO_1_CLK_ENABLE();

    /* светодиод-индикатор */
    gpio.Pin  = GPIO_PIN_1;
    gpio.Mode = HAL_GPIO_MODE_GPIO_OUTPUT;
    HAL_GPIO_Init(GPIO_0, &gpio);
    HAL_GPIO_WritePin(GPIO_0, GPIO_PIN_1, GPIO_PIN_HIGH);

    /* пины эха и триггера ультразвука */
    gpio.Pin  = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Pull = HAL_GPIO_PULL_NONE;
    HAL_GPIO_Init(GPIO_1, &gpio);

    gpio.Pin  = GPIO_PIN_9;
    HAL_GPIO_Init(GPIO_0, &gpio);
    HAL_GPIO_WritePin(GPIO_0, GPIO_PIN_9, GPIO_PIN_LOW);
}

/* ================= ТАЙМЕРЫ ================= */
static void Timer16_CommonInit(Timer16_HandleTypeDef *h)
{
    h->Clock.Source         = TIMER16_SOURCE_INTERNAL_SYSTEM;
    h->Clock.Prescaler      = TIMER16_PRESCALER_32;
    h->CountMode            = TIMER16_COUNTMODE_INTERNAL;
    h->Preload              = TIMER16_PRELOAD_AFTERWRITE;
    h->Trigger.ActiveEdge   = TIMER16_TRIGGER_ACTIVEEDGE_SOFTWARE;
    h->Trigger.TimeOut      = TIMER16_TIMEOUT_DISABLE;
    h->Filter.ExternalClock = TIMER16_FILTER_NONE;
    h->Filter.Trigger       = TIMER16_FILTER_NONE;
    h->EncoderMode          = TIMER16_ENCODER_DISABLE;
    h->Waveform.Enable      = TIMER16_WAVEFORM_GENERATION_ENABLE;
    h->Waveform.Polarity    = TIMER16_WAVEFORM_POLARITY_INVERTED;
    HAL_Timer16_Init(h);
}

static void Timer32_1_Init(void)
{
    htimer32_1.Instance         = TIMER32_1;
    htimer32_1.Top              = 19999;
    htimer32_1.Clock.Source     = TIMER32_SOURCE_PRESCALER;
    htimer32_1.Clock.Prescaler  = 31;   /* 32 МГц / 32 / 2 кГц */
    htimer32_1.CountMode        = TIMER32_COUNTMODE_FORWARD;
    HAL_Timer32_Init(&htimer32_1);

    htimer32_1_ch0.TimerInstance = TIMER32_1;
    htimer32_1_ch0.ChannelIndex  = TIMER32_CHANNEL_0;
    htimer32_1_ch0.Mode          = TIMER32_CHANNEL_MODE_PWM;
    htimer32_1_ch0.PWM_Invert    = TIMER32_CHANNEL_NON_INVERTED_PWM;
    htimer32_1_ch0.OCR           = cfg.esc_neutral_us;
    HAL_Timer32_Channel_Init(&htimer32_1_ch0);
}

static void Timer32_2_Init(void)
{
    htimer32_2.Instance        = TIMER32_2;
    htimer32_2.Top             = ~0U;
    htimer32_2.Clock.Source    = TIMER32_SOURCE_PRESCALER;
    htimer32_2.Clock.Prescaler = 31;
    htimer32_2.CountMode       = TIMER32_COUNTMODE_FORWARD;
    HAL_Timer32_Init(&htimer32_2);

    htimer32_2_ch0.TimerInstance = TIMER32_2;
    htimer32_2_ch0.ChannelIndex  = TIMER32_CHANNEL_0;
    htimer32_2_ch0.Mode          = TIMER32_CHANNEL_MODE_CAPTURE;
    htimer32_2_ch0.CaptureEdge   = TIMER32_CHANNEL_CAPTUREEDGE_RISING;
    HAL_Timer32_Channel_Init(&htimer32_2_ch0);

    htimer32_2_ch1 = htimer32_2_ch0;
    htimer32_2_ch1.ChannelIndex = TIMER32_CHANNEL_1;
    htimer32_2_ch1.CaptureEdge  = TIMER32_CHANNEL_CAPTUREEDGE_FALLING;
    HAL_Timer32_Channel_Init(&htimer32_2_ch1);
}

/* ================= АЦП ================= */
static void ADC_Init(void)
{
    hadc.Instance     = ANALOG_REG;
    hadc.Init.Sel     = ADC_CHANNEL2;
    hadc.Init.EXTRef  = ADC_EXTREF_OFF;
    hadc.Init.EXTClb  = ADC_EXTCLB_ADCREF;
    HAL_ADC_Init(&hadc);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    GPIO_InitTypeDef g = {0};
    __HAL_PCC_ANALOG_REGS_CLK_ENABLE();
    g.Pin  = GPIO_PIN_2;
    g.Mode = HAL_GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIO_0, &g);
}

/* ================= ФУНКЦИИ ================= */
uint16_t simple_atoi(uint8_t *buf, uint8_t len)
{
    uint16_t r = 0;
    while (len--) {
        if (*buf >= '0' && *buf <= '9')
            r = r * 10 + (*buf - '0');
        buf++;
    }
    return r;
}

void Servo_SetAngle(uint16_t deg)
{
    if (deg > 180) deg = 180;
    uint32_t pulse = cfg.servo_min_us +
                     (uint32_t)deg * (cfg.servo_max_us - cfg.servo_min_us) / 180;
    htimer16_0.Instance->CMP = (uint16_t)pulse;
}

void ESC_SetThrottle(int16_t pct)
{
    if (pct >  100) pct =  100;
    if (pct < -100) pct = -100;
    if (!cfg.esc_reverse_enabled && pct < 0) pct = 0;
    last_throttle_pct = pct;

    uint16_t pulse;
    if (pct >= 0) {
        pulse = cfg.esc_neutral_us +
            (uint32_t)pct * (cfg.esc_forward_max_us - cfg.esc_neutral_us) / 100;
    } else {
        uint16_t r = (uint16_t)(-pct);
        pulse = cfg.esc_neutral_us -
            (uint32_t)r * (cfg.esc_neutral_us - cfg.esc_reverse_min_us) / 100;
    }
    htimer16_1.Instance->CMP = pulse;
}

void xputc(char c)
{
    uint16_t next = (tx_head + 1u) % TX_BUF_SZ;
    if (next == tx_tail) { ++tx_overruns; return; }
    tx_buf[tx_head] = (uint8_t)c;
    tx_head = next;
}

static void UART_TxTask(void)
{
    if (tx_head == tx_tail)           return;
    if (!UART_IsTxBufferFreed(UART_1)) return;

    UART_WriteByte(UART_1, tx_buf[tx_tail]);
    tx_tail = (tx_tail + 1u) % TX_BUF_SZ;
}

/* ================= ОБРАБОТКА ПРЕРЫВАНИЙ ================= */
void trap_handler(void)
{
    if (EPIC_CHECK_UART_1()) {
        if (UART_1->FLAGS & UART_FLAGS_RXNE_M) {
            uint8_t c = (uint8_t)UART_1->RXDATA;
            uint16_t next = (rx_head + 1u) % RX_BUF_SZ;
            if (next != rx_tail) {
                rx_buf[rx_head] = c;
                rx_head = next;
            } else {
                ++rx_overruns; 
            }
        }
    }
    HAL_EPIC_Clear(0xFFFFFFFF);
}

/* ==================== main ==================== */
int main(void)
{
    uint8_t  uart_cmd[16];
    uint8_t  cmd_idx = 0;
    uint32_t now, cmd_val;

    SystemClock_Config();
    GPIO_Init();
    HAL_Time_TIM32_Init(TIMER32_0);

    UART_Init(UART_1, 3333,
              UART_CONTROL1_TE_M | UART_CONTROL1_RE_M | UART_CONTROL1_M_8BIT_M,
              0, 0);

    /* Разрешаем прерывания по приёму */
    UART_1->CONTROL1 |= UART_CONTROL1_RXNEIE_M;
    __HAL_PCC_EPIC_CLK_ENABLE();
    HAL_EPIC_MaskLevelSet(HAL_EPIC_UART_1_MASK);
    HAL_IRQ_EnableInterrupts();

    ADC_Init();

    last_command_time = HAL_Time_TIM32_Millis();
    last_adc_time     = last_command_time;

    /* --- PWM для сервопривода --- */
    Timer16_CommonInit(&htimer16_0);
    HAL_Timer16_StartPWM(&htimer16_0, 20000,
                         (cfg.servo_min_us + cfg.servo_max_us) / 2);
    htimer16_0.Instance->CFGR |= TIMER16_CFGR_PRELOAD_M;

    /* --- PWM для ESC --- */
    Timer16_CommonInit(&htimer16_1);
    HAL_Timer16_StartPWM(&htimer16_1, 20000, cfg.esc_neutral_us);
    htimer16_1.Instance->CFGR |= TIMER16_CFGR_PRELOAD_M;

    /* --- Таймеры измерения эха --- */
    Timer32_1_Init();
    Timer32_2_Init();
    HAL_Timer32_Channel_Enable(&htimer32_1_ch0);
    HAL_Timer32_Channel_Enable(&htimer32_2_ch0);
    HAL_Timer32_Channel_Enable(&htimer32_2_ch1);
    HAL_Timer32_Start(&htimer32_1);
    HAL_Timer32_Start(&htimer32_2);

    while (1)
    {
        now = HAL_Time_TIM32_Millis();

        /* --- защита по тайм-ауту команды --- */
        if (now - last_command_time > cfg.command_timeout_ms)
            ESC_SetThrottle(0);

        /* --- телеметрия раз в секунду --- */
        if (now - last_adc_time >= 1000) {
            HAL_ADC_Single(&hadc);
            uint16_t val = HAL_ADC_WaitAndGetValue(&hadc);
            xprintf("ADC:%u\n", val);
            last_adc_time = now;
        }

        /* --- разбор буфера приёма --- */
        while (rx_tail != rx_head) {
            uint8_t c = rx_buf[rx_tail];
            rx_tail = (rx_tail + 1u) % RX_BUF_SZ;

            if (c == '\r' || c == '\n') {
                if (cmd_idx) {
                    switch (uart_cmd[0]) {
                        case 's': /* servo angle            */
                            cmd_val = simple_atoi(uart_cmd + 1, cmd_idx - 1);
                            Servo_SetAngle(cmd_val);
                            break;
                        case 't': /* throttle forward       */
                            cmd_val = simple_atoi(uart_cmd + 1, cmd_idx - 1);
                            ESC_SetThrottle((int16_t)cmd_val);
                            break;
                        case 'b': /* throttle backward      */
                            cmd_val = simple_atoi(uart_cmd + 1, cmd_idx - 1);
                            ESC_SetThrottle(-(int16_t)cmd_val);
                            break;
                        case 'm': /* SERVO_MIN              */
                            cfg.servo_min_us = simple_atoi(uart_cmd + 1, cmd_idx - 1);
                            xprintf("m set\n");
                            break;
                        case 'M': /* SERVO_MAX              */
                            cfg.servo_max_us = simple_atoi(uart_cmd + 1, cmd_idx - 1);
                            xprintf("M set\n");
                            break;
                        case 'e': /* echo threshold         */
                            cfg.echo_threshold_us = simple_atoi(uart_cmd + 1, cmd_idx - 1);
                            xprintf("e set\n");
                            break;
                        case 'c': /* command timeout        */
                            cfg.command_timeout_ms = simple_atoi(uart_cmd + 1, cmd_idx - 1);
                            xprintf("c set\n");
                            break;
                    }
                    last_command_time = now;
                }
                cmd_idx = 0;
            } else if (cmd_idx < sizeof(uart_cmd) - 1) {
                uart_cmd[cmd_idx++] = c;
            }
        }

        /* --- обработка ультразвука --- */
        if (last_echo < 0) {
            if (htimer32_2.Instance->INT_FLAGS & TIMER32_INT_IC_M(1)) {
                last_echo = 0;
                htimer32_2.Instance->INT_FLAGS = TIMER32_INT_IC_M(1);
            }
        } else if ((htimer32_2.Instance->INT_FLAGS & TIMER32_INT_IC_M(0)) &&
                   (htimer32_2.Instance->INT_FLAGS & TIMER32_INT_IC_M(1))) {
            last_echo = htimer32_2.Instance->CHANNELS[1].ICR -
                        htimer32_2.Instance->CHANNELS[0].ICR;
            htimer32_2.Instance->INT_FLAGS =
                TIMER32_INT_IC_M(0) | TIMER32_INT_IC_M(1);

            if (last_echo > 0 && last_echo < (int32_t)cfg.echo_threshold_us) {
                if (++echo_alert_cnt >= cfg.echo_filter) {
                    if (last_throttle_pct > 0)
                        ESC_SetThrottle(0);
                    HAL_GPIO_WritePin(GPIO_0, GPIO_PIN_9, GPIO_PIN_HIGH);
                }
            } else {
                echo_alert_cnt = 0;
                HAL_GPIO_WritePin(GPIO_0, GPIO_PIN_9, GPIO_PIN_LOW);
            }
        }
        /* --- отправка накопленных символов --- */
        UART_TxTask();
    }
}
