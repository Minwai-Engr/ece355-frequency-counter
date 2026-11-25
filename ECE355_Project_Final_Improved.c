/* ECE 355 - PWM Signal Generation and Monitoring System
 * STM32F051R8T6 / STM32F0 Discovery + PBMCUSLK SSD1306
 * Revised from the group's source code, 2026-11-23.
 * Hardware integration candidate: compile and validate on the actual lab boards.
 *
 * PA0: USER button (active-high on STM32F0 Discovery)
 * PA1: ADC channel 1, potentiometer wiper (0..3.3 V)
 * PA4: DAC channel 1, optocoupler drive circuit (requires series resistor)
 * PB2: function generator SYNC (0..3.3 V)
 * PB3: 555 output (MUST be level-shifted to 0..3.3 V if necessary)
 * PB8/PB9/PB11: OLED CS/DC/RESET
 * PB13/PB15: SPI2 SCK/MOSI, AF0
 * PC8/PC9: board LEDs, indicate selected input
 *
 * IMPORTANT: POT_MAX_OHMS and ADC_TO_DAC_DIRECT are design assumptions.
 * Verify the actual PBMCUSLK potentiometer wiring and 4N35 drive circuit.
 */

#include <stdint.h>
#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"

#define TIMER_CLOCK_HZ       48000000UL
#define POT_MAX_OHMS        5000UL /* CHANGE after checking actual POT and wiring */
#define DEBOUNCE_MS         50U
#define DISPLAY_INTERVAL_MS 100U
#define ADC_TO_DAC_DIRECT   1U    /* DAC output follows ADC; calibrate for 4N35 */
#define OLED_COLUMN_OFFSET  2U    /* PBMCUSLK sample uses column offset 2 */

volatile uint32_t f = 0;         /* measured frequency, Hz */
volatile uint32_t r = 0;         /* inferred resistance, ohms */
volatile uint8_t timTrig = 0;    /* 0: wait for first edge; 1: wait for second */
volatile uint8_t source555 = 1;  /* 1: PB3 555; 0: PB2 generator */
volatile uint8_t freqValid = 0;
volatile uint8_t freqOverflow = 0;

/* TIM3 is a free-running 1 kHz (1 ms/tick) 16-bit timer.
 * Subtraction in uint16_t works correctly for intervals < 65536 ms.
 */
static volatile uint16_t lastButtonTick = 0;
static volatile uint8_t buttonSeen = 0;

static void SystemClock48MHz(void);
static void myGPIOA_Init(void);
static void myGPIOB_Init(void);
static void myGPIOC_Init(void);
static void myADC_Init(void);
static void myDAC_Init(void);
static void myTIM2_Init(void);
static void myTIM3_Init(void);
static void myEXTI_Init(void);
static void oled_Init(void);
static void refresh_OLED(uint32_t freq, uint32_t res, uint8_t src, uint8_t valid, uint8_t overflow);
static void read_Pot(void);

/* ------------------ CLOCK ------------------ */
static void SystemClock48MHz(void)
{
    /* HSI/2 * 12 = 48 MHz, as in the course's Part 2 clock routine. */
    FLASH->ACR |= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) { }
    RCC->CFGR = 0x00280000U;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) { }
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL) { }
    SystemCoreClockUpdate();
}

/* ------------------ GPIO ------------------ */
static void myGPIOA_Init(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    /* PA0 input, PA1 ADC analog, PA4 DAC analog. */
    GPIOA->MODER &= ~(3U << (0U * 2U));
    GPIOA->MODER |=  (3U << (1U * 2U)) | (3U << (4U * 2U));
    GPIOA->PUPDR &= ~((3U << (0U * 2U)) | (3U << (1U * 2U)) | (3U << (4U * 2U)));
}

static void myGPIOB_Init(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    /* PB2, PB3 digital input, no internal pull-up/down. */
    GPIOB->MODER &= ~((3U << 4U) | (3U << 6U));
    GPIOB->PUPDR &= ~((3U << 4U) | (3U << 6U));
}

static void myGPIOC_Init(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER &= ~((3U << 16U) | (3U << 18U));
    GPIOC->MODER |=  (1U << 16U) | (1U << 18U);
    /* PC8 blue = GEN, PC9 green = 555. */
    GPIOC->BSRR = (1U << (8U + 16U)) | (1U << 9U);
}

static void update_Source_LEDs(void)
{
    if (source555) {
        GPIOC->BSRR = (1U << (8U + 16U)) | (1U << 9U);
    } else {
        GPIOC->BSRR = (1U << 8U) | (1U << (9U + 16U));
    }
}

/* ------------------ ADC/DAC ------------------ */
static void myADC_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
    /* Enable HSI14 for ADC asynchronous clock (CKMODE = 00). */
    RCC->CR2 |= RCC_CR2_HSI14ON;
    while (!(RCC->CR2 & RCC_CR2_HSI14RDY)) { }
    ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE;
    /* ADC must be disabled before calibration. */
    if (ADC1->CR & ADC_CR_ADEN) {
        ADC1->CR |= ADC_CR_ADDIS;
        while (ADC1->CR & ADC_CR_ADEN) { }
    }
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL) { }
    ADC1->CHSELR = ADC_CHSELR_CHSEL1;
    ADC1->SMPR = 7U;  /* longest sampling time */
    ADC1->ISR = ADC_ISR_ADRDY; /* clear stale ready flag (W1C) */
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY)) { }
}

static void myDAC_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
    DAC->DHR12R1 = 0U;
    DAC->CR |= DAC_CR_EN1;
}

static void read_Pot(void)
{
    uint32_t adc;
    ADC1->CR |= ADC_CR_ADSTART;
    while (!(ADC1->ISR & ADC_ISR_EOC)) { }
    adc = ADC1->DR & 0xFFFU;
    /* Assumes POT voltage is proportional to the desired resistance.
     * Check the PBMCUSLK circuit before treating this as an actual ohmic value.
     */
    r = (uint32_t)(((uint64_t)adc * POT_MAX_OHMS + 2047ULL) / 4095ULL);
#if ADC_TO_DAC_DIRECT
    DAC->DHR12R1 = adc;
#endif
}

/* ------------------ TIMERS ------------------ */
static void myTIM2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->CR1 = TIM_CR1_OPM | TIM_CR1_URS; /* up-count; stop at overflow */
    TIM2->PSC = 0U;
    TIM2->ARR = 0xFFFFFFFFUL;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0U;
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);
}

static void myTIM3_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->CR1 = 0U; /* up-count continuously, NOT one-pulse */
    TIM3->PSC = 47999U; /* 48 MHz / 48000 = 1000 Hz */
    TIM3->ARR = 0xFFFFU;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0U;
    TIM3->CNT = 0U;
    TIM3->CR1 |= TIM_CR1_CEN;
}

static uint16_t millis16(void)
{
    return (uint16_t)TIM3->CNT;
}

/* ------------------ EXTI ------------------ */
static void myEXTI_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    /* EXTICR1: EXTI0=PA0, EXTI2=PB2, EXTI3=PB3. */
    SYSCFG->EXTICR[0] &= ~((0xFU << 0U) | (0xFU << 8U) | (0xFU << 12U));
    SYSCFG->EXTICR[0] |= (1U << 8U) | (1U << 12U);

    /* Discovery USER button is active-high: press produces a rising edge. */
    EXTI->RTSR |= (1U << 0U) | (1U << 2U) | (1U << 3U);
    EXTI->FTSR &= ~((1U << 0U) | (1U << 2U) | (1U << 3U));
    EXTI->PR = (1U << 0U) | (1U << 2U) | (1U << 3U);
    EXTI->IMR &= ~((1U << 2U) | (1U << 3U));
    EXTI->IMR |= (1U << 0U) | (1U << 3U); /* start on 555 PB3 */

    NVIC_SetPriority(EXTI0_1_IRQn, 1);
    NVIC_SetPriority(EXTI2_3_IRQn, 0);
    NVIC_EnableIRQ(EXTI0_1_IRQn);
    NVIC_EnableIRQ(EXTI2_3_IRQn);
}

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR = 0U;
        TIM2->CR1 &= ~TIM_CR1_CEN;
        timTrig = 0;
        f = 0U;
        freqValid = 0;
        freqOverflow = 1;
    }
}

void EXTI0_1_IRQHandler(void)
{
    if (EXTI->PR & (1U << 0U)) {
        uint16_t now;
        EXTI->PR = (1U << 0U); /* W1C */
        now = millis16();
        if (buttonSeen && (uint16_t)(now - lastButtonTick) < DEBOUNCE_MS) {
            return; /* ignore contact bounce */
        }
        buttonSeen = 1;
        lastButtonTick = now;

        /* Stop measurement before changing selected input. */
        EXTI->IMR &= ~((1U << 2U) | (1U << 3U));
        TIM2->CR1 &= ~TIM_CR1_CEN;
        TIM2->SR = 0U;
        timTrig = 0;
        f = 0U;
        freqValid = 0;
        freqOverflow = 0;
        source555 ^= 1U;
        EXTI->PR = (1U << 2U) | (1U << 3U);
        EXTI->IMR |= source555 ? (1U << 3U) : (1U << 2U);
        update_Source_LEDs();
    }
}

void EXTI2_3_IRQHandler(void)
{
    uint32_t pending = EXTI->PR & ((1U << 2U) | (1U << 3U));
    uint32_t active = source555 ? (1U << 3U) : (1U << 2U);
    if (pending != 0U) {
        EXTI->PR = pending; /* W1C */
        if (!(pending & active)) return;

        if (!timTrig) {
            TIM2->CR1 &= ~TIM_CR1_CEN;
            TIM2->CNT = 0U;
            TIM2->SR = 0U;
            TIM2->CR1 |= TIM_CR1_CEN;
            timTrig = 1;
            freqOverflow = 0;
        } else {
            uint32_t count;
            TIM2->CR1 &= ~TIM_CR1_CEN;
            count = TIM2->CNT;
            if ((TIM2->SR & TIM_SR_UIF) || count == 0U) {
                f = 0U;
                freqValid = 0;
                freqOverflow = 1;
            } else {
                /* rounded integer Hz; counts are TIM2 ticks at 48 MHz */
                f = (uint32_t)(((uint64_t)TIMER_CLOCK_HZ + count / 2U) / count);
                freqValid = 1;
                freqOverflow = 0;
            }
            timTrig = 0;
        }
    }
}

/* ------------------ SSD1306 OLED (SPI2, 128x64) ------------------ */
static void spi2_Send(uint8_t data)
{
    while (!(SPI2->SR & SPI_SR_TXE)) { }
    *(__IO uint8_t *)&SPI2->DR = data;
    while (SPI2->SR & SPI_SR_BSY) { }
}

static void oled_Write_Cmd(uint8_t cmd)
{
    GPIOB->BSRR = (1U << (8U + 16U)); /* CS high */
    GPIOB->BSRR = (1U << (9U + 16U)); /* DC low */
    GPIOB->BSRR = (1U << (8U + 16U)); /* CS low */
    spi2_Send(cmd);
    GPIOB->BSRR = (1U << 8U);         /* CS high */
}

static void oled_Write_Data(uint8_t data)
{
    GPIOB->BSRR = (1U << 8U);         /* CS high */
    GPIOB->BSRR = (1U << 9U);         /* DC high */
    GPIOB->BSRR = (1U << (8U + 16U)); /* CS low */
    spi2_Send(data);
    GPIOB->BSRR = (1U << 8U);         /* CS high */
}

static void oled_SetPageColumn(uint8_t page, uint8_t col)
{
    oled_Write_Cmd(0xB0U | (page & 7U));
    oled_Write_Cmd((uint8_t)(0x00U | (col & 0x0FU)));
    oled_Write_Cmd((uint8_t)(0x10U | ((col >> 4U) & 0x0FU)));
}

/* Minimal 5x7 glyph set for display strings below. */
static void oled_Glyph(char ch, uint8_t glyph[5])
{
    uint8_t i;
    for (i = 0; i < 5; i++) glyph[i] = 0;
    switch (ch) {
    case '0': { const uint8_t a[5]={0x3E,0x51,0x49,0x45,0x3E}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case '1': { const uint8_t a[5]={0x00,0x42,0x7F,0x40,0x00}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case '2': { const uint8_t a[5]={0x42,0x61,0x51,0x49,0x46}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case '3': { const uint8_t a[5]={0x21,0x41,0x45,0x4B,0x31}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case '4': { const uint8_t a[5]={0x18,0x14,0x12,0x7F,0x10}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case '5': { const uint8_t a[5]={0x27,0x45,0x45,0x45,0x39}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case '6': { const uint8_t a[5]={0x3C,0x4A,0x49,0x49,0x30}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case '7': { const uint8_t a[5]={0x03,0x01,0x71,0x09,0x07}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case '8': { const uint8_t a[5]={0x36,0x49,0x49,0x49,0x36}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case '9': { const uint8_t a[5]={0x06,0x49,0x49,0x29,0x1E}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case ':': glyph[1]=0x36; glyph[2]=0x36; break;
    case 'F': { const uint8_t a[5]={0x7F,0x09,0x09,0x09,0x01}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case 'R': { const uint8_t a[5]={0x7F,0x09,0x19,0x29,0x46}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case 'G': { const uint8_t a[5]={0x3E,0x41,0x49,0x49,0x7A}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case 'E': { const uint8_t a[5]={0x7F,0x49,0x49,0x49,0x41}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case 'N': { const uint8_t a[5]={0x7F,0x02,0x0C,0x10,0x7F}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case 'H': { const uint8_t a[5]={0x7F,0x08,0x08,0x08,0x7F}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case 'z': { const uint8_t a[5]={0x44,0x64,0x54,0x4C,0x44}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case 'O': { const uint8_t a[5]={0x3E,0x41,0x41,0x41,0x3E}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case 'h': { const uint8_t a[5]={0x7F,0x08,0x04,0x04,0x78}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case 'm': { const uint8_t a[5]={0x7C,0x04,0x18,0x04,0x78}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case 's': { const uint8_t a[5]={0x48,0x54,0x54,0x54,0x20}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case 'V': { const uint8_t a[5]={0x1F,0x20,0x40,0x20,0x1F}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case 'A': { const uint8_t a[5]={0x7E,0x11,0x11,0x11,0x7E}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case 'L': { const uint8_t a[5]={0x7F,0x40,0x40,0x40,0x40}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case 'I': { const uint8_t a[5]={0x41,0x41,0x7F,0x41,0x41}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case 'D': { const uint8_t a[5]={0x7F,0x41,0x41,0x22,0x1C}; for(i=0;i<5;i++)glyph[i]=a[i]; break; }
    case '-': glyph[1]=glyph[2]=glyph[3]=0x08; break;
    default: break;
    }
}

static void write_Page(const char *text, uint8_t page)
{
    uint16_t col;
    uint8_t glyph[5];
    uint8_t k;
    /* Clear whole page first so shorter readings don't leave old digits. */
    oled_SetPageColumn(page, OLED_COLUMN_OFFSET);
    for (col = 0; col < 128U; col++) oled_Write_Data(0U);
    oled_SetPageColumn(page, OLED_COLUMN_OFFSET);
    for (col = 0; col < 16U && text[col] != '\0'; col++) {
        oled_Glyph(text[col], glyph);
        for (k = 0; k < 5U; k++) oled_Write_Data(glyph[k]);
        for (k = 0; k < 3U; k++) oled_Write_Data(0U);
    }
}

static void oled_Init(void)
{
    static const uint8_t cmds[] = {
        0xAE, 0xD5,0x80, 0xA8,0x3F, 0xD3,0x00, 0x40,
        0x8D,0x14, 0x20,0x02, /* charge pump, PAGE addressing mode */
        0xA1, 0xC8, 0xDA,0x12, 0x81,0x7F, 0xD9,0xF1,
        0xDB,0x40, 0xA4, 0xA6, 0xAF
    };
    uint32_t i;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    /* PB13/15 AF0 = SPI2, PB8/9/11 general-purpose outputs. */
    GPIOB->MODER &= ~((3U << 26U) | (3U << 30U) |
                      (3U << 16U) | (3U << 18U) | (3U << 22U));
    GPIOB->MODER |= (2U << 26U) | (2U << 30U) |
                    (1U << 16U) | (1U << 18U) | (1U << 22U);
    GPIOB->AFR[1] &= ~((0xFU << 20U) | (0xFU << 28U)); /* AF0 */
    GPIOB->BSRR = (1U << 8U) | (1U << 11U);

    SPI2->CR1 = 0U;
    SPI2->CR2 = (7U << 8U) | SPI_CR2_FRXTH; /* 8-bit frames */
    SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI |
                SPI_CR1_BR | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE;
    SPI2->CR1 |= SPI_CR1_SPE;

    GPIOB->BSRR = (1U << (11U + 16U)); /* reset low */
    for (volatile uint32_t d=0; d<30000U; d++) { }
    GPIOB->BSRR = (1U << 11U);         /* reset high */
    for (volatile uint32_t d=0; d<30000U; d++) { }

    for (i=0; i<sizeof(cmds); i++) oled_Write_Cmd(cmds[i]);
    for (i=0; i<8U; i++) write_Page("", (uint8_t)i);
}

static void refresh_OLED(uint32_t freq, uint32_t res, uint8_t src, uint8_t valid, uint8_t overflow)
{
    char line[17];
    write_Page(src ? "555" : "GEN", 0);
    snprintf(line, sizeof(line), "R:%lu Ohms", (unsigned long)res);
    write_Page(line, 2);
    if (overflow) {
        write_Page("F:OVER", 4); /* long-period overflow */
    } else if (!valid) {
        write_Page("F:----", 4); /* waiting for measurement */
    } else {
        snprintf(line, sizeof(line), "F:%lu Hz", (unsigned long)freq);
        write_Page(line, 4);
    }
}

int main(void)
{
    uint16_t lastDisplayTick;
    uint32_t displayF, displayR;
    uint8_t displaySrc, displayValid, displayOverflow;

    SystemClock48MHz();
    myGPIOA_Init();
    myGPIOB_Init();
    myGPIOC_Init();
    myADC_Init();
    myDAC_Init();
    myTIM2_Init();
    myTIM3_Init();
    myEXTI_Init();
    oled_Init();
    update_Source_LEDs();

    lastDisplayTick = (uint16_t)(millis16() - DISPLAY_INTERVAL_MS);
    while (1) {
        uint16_t now = millis16();
        if ((uint16_t)(now - lastDisplayTick) >= DISPLAY_INTERVAL_MS) {
            lastDisplayTick = now;
            read_Pot();
            /* 32-bit variables are atomic on this Cortex-M0; snapshot values. */
            displayF = f;
            displayR = r;
            displaySrc = source555;
            displayValid = freqValid;
            displayOverflow = freqOverflow;
            refresh_OLED(displayF, displayR, displaySrc, displayValid, displayOverflow);
        }
    }
}
