#include <stdint.h>
#include "stm32f30x.h"
#include "gpio.h"

#define DIV(a, b) (((a)+((b)/2))/(b))

#define ADC_LEN 360 /* 72 MHz (ADC freq) / 10 kHz (Resulting freq) / (12.5+7.5) cycles */
#define BUF_LEN (ADC_LEN<<1)
#define ADC_F   (3600000 / ADC_LEN)

#define KOEFF1 380
#define KOEFF2 29

uint32_t adcref;
int32_t ref1 = 10000;
int32_t ref2 = 12000;

uint16_t adc1[BUF_LEN];
uint16_t adc2[BUF_LEN];

// log
uint32_t  n1, n2;
uint32_t log_adc1[256], log_adc2[256];
int32_t  log_err1[256], log_err2[256];
uint32_t log_k1[256], log_k2[256];

void set1(uint16_t val) {
    
    // Phase 1
    TIM15->CCR1 = val;
    
    // Phase 2
    if (val<=480) {
        TIM1->CCR1 = 240;
        TIM1->CCR2 = 240+val;
        TIM1->CCMR1 = TIM_CCMR1_OC1M_3 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0
                    | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
    }
    else {
        TIM1->CCR1 = val-480;
        TIM1->CCR2 = 240;
        TIM1->CCMR1 = TIM_CCMR1_OC1M_3 | TIM_CCMR1_OC1M_2
                    | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0;
    }
    
    // Phase 3
    if (val<=240) {
        TIM1->CCMR2 = TIM_CCMR2_OC3M_3 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_0
                    | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
        TIM1->CCR3 = 480;
        TIM1->CCR4 = 480+val;
    }
    else {
        TIM1->CCMR2 = TIM_CCMR2_OC3M_3 | TIM_CCMR2_OC3M_2
                    | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_0 | TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
        TIM1->CCR3 = val-240;
        TIM1->CCR4 = 480;
    }
    
}

void set2(uint16_t val) {
    
    // Phase 1
    TIM8->CCR1 = val;
    
    // Phase 2
    TIM8->CCR4 = 360+val;
    
}
    uint16_t * ptr;
    uint16_t * end;
void DMA1_Channel1_IRQHandler() {
    
    static uint8_t first = 2;
    
    uint16_t i;

    uint16_t from;
    uint16_t to;
    int32_t sum = 0;
    int32_t neg = 0;
    static int32_t err = 0;
    
    // tmp
    
    static uint32_t blink;
    
    blink++;
    if (blink>=(ADC_F>>1)) {
        blink = 0;
        GPIOE->BSRR = (GPIOE->ODR & (1<<14)) ? (1<<30) : (1<<14);
    }
    
    // /tmp
    
    if (first) {
        if (first==2) {
            for (i=0; i<ADC_LEN; i++) { // Суммируем
                sum += adc1[i];
            }
            adcref = DIV(sum, ADC_LEN);
            ADC1->CR |= ADC_CR_ADSTP; // Start ADC conversion
            while (ADC1->CR & ADC_CR_ADSTP);
            ADC1->SQR1 = ADC_SQR1_SQ1_1 | ADC_SQR1_SQ1_2; // Select channel 1
            ADC1->CR |= ADC_CR_ADSTART; // Start ADC conversion
        }
        first--;
        DMA1->IFCR |= DMA_IFCR_CGIF1; // Clear interrupt flags
        return;
    }
    
    if (DMA1->ISR & DMA_IFCR_CHTIF1) { // Смотрим из первой половины буфера брать
        //ptr = adc1;
        //end = adc1 + ADC_LEN;
        from = 0;
        to = ADC_LEN;
    }
    else { // или из второй
        //ptr = adc1 + ADC_LEN;
        //end = adc1 + BUF_LEN;
        from = ADC_LEN;
        to = BUF_LEN;
    }
    //while (ptr<end) {
    //    sum += *ptr;
    //    ptr++;
    //}
    for (i=from; i<to; i++) { // Суммируем
        sum += adc1[i];
    }
    
    if (sum>(ADC_LEN<<11)) {
        sum = DIV(sum, ADC_LEN);
        sum = DIV(sum*192000, adcref); // Вычисляем значение в мА
    }
    else {
        sum = 0;
    }
    
    err += (ref1 - sum)*KOEFF1/ADC_F; // Складируем текущую ошибку с предыдущей. Значение ошибки в В*сек*F, то есть в 10 тыс раз больше
    if (err<0) { // Ограничиваем ошибку снизу
        err = 0;
    }
    else if (err>576000) { // и сверху
        err = 576000;
    }
    set1(DIV(err, 1000));
    
    // Log
    log_adc1[n1&0xFF] = sum;
    log_err1[n1&0xFF] = DIV(err, 1000);
    log_k1[n1&0xFF]   = DIV(err, 720);
    n1++;
    
    // Clear interrupt flags
    DMA1->IFCR |= DMA_IFCR_CGIF1;
    
}

void DMA2_Channel1_IRQHandler() {
    
    uint16_t i;
    uint16_t from;
    uint16_t to;
    int32_t sum = 0;
    static int32_t err = 0;
    
    if (DMA2->ISR & DMA_IFCR_CHTIF1) { // Смотрим из первой половины буфера брать
        from = 0;
        to = ADC_LEN;
    }
    else { // или из второй
        from = ADC_LEN;
        to = BUF_LEN;
    }
    for (i=from; i<to; i++) { // Суммируем
        sum += adc2[i];
    }
    sum = DIV(sum, ADC_LEN);
    sum = DIV(sum*11127, adcref); // Вычисляем значение в мВ
    
    err += (sum - ref2)*KOEFF2/ADC_F; // Складируем текущую ошибку с предыдущей. Значение ошибки в В*сек*F, то есть в 10 тыс раз больше
    if (err<0) { // Если ошибка отрицательная, то коэффициент заполнения 0
        err = 0;
    }
    else if (err>320000) { // если положительная, то задаем
        err = 320000;
    }
    set2(DIV(err, 1000));
    //set2(ref2);
    
    // Log
    log_adc2[n1&0xFF] = sum;
    log_err2[n1&0xFF] = DIV(err, 1000);
    log_k2[n1&0xFF]   = DIV(err, 360);
    n2++;
    
    // Clear interrupt flags
    DMA2->IFCR |= DMA_IFCR_CGIF1;
    
}

uint32_t ticks;

void SysTick_Handler() {
    
    ticks++;
    
}

void TIM1_UP_TIM16_IRQHandler() {
    
    TIM1->SR &= ~TIM_SR_UIF;
    
}

int main(void) {
    
    // Variables
    uint32_t sec = 0;
    
    // SysTick init
    SysTick_Config(SystemCoreClock/1000);
    
    // GPIO init
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
    gpio_config(GPIOA,  4, MODE_AN, OTYPE_PP, OSPEED_2, PULL_NO, AF_NO); // ADC2_IN1
    gpio_config(GPIOB,  6, MODE_AF, OTYPE_PP, OSPEED_2, PULL_NO, AF5);   // TIM8_CH1
    gpio_config(GPIOB,  9, MODE_AF, OTYPE_PP, OSPEED_2, PULL_NO, AF10);  // TIM8_CH3
    gpio_config(GPIOB, 14, MODE_AF, OTYPE_PP, OSPEED_2, PULL_NO, AF1);   // TIM15_CH1
    gpio_config(GPIOB, 15, MODE_AF, OTYPE_PP, OSPEED_2, PULL_NO, AF2);   // TIM15_CH1N
    gpio_config(GPIOC,  0, MODE_AN, OTYPE_PP, OSPEED_2, PULL_NO, AF_NO); // ADC1_IN6
    gpio_config(GPIOC,  1, MODE_AN, OTYPE_PP, OSPEED_2, PULL_NO, AF_NO); // ADC1_IN7
    gpio_config(GPIOE,  8, MODE_AF, OTYPE_PP, OSPEED_2, PULL_NO, AF2);   // TIM1_CH1N
    gpio_config(GPIOE,  9, MODE_AF, OTYPE_PP, OSPEED_2, PULL_NO, AF2);   // TIM1_CH1
    gpio_config(GPIOE, 12, MODE_AF, OTYPE_PP, OSPEED_2, PULL_NO, AF2);   // TIM1_CH3N
    gpio_config(GPIOE, 13, MODE_AF, OTYPE_PP, OSPEED_2, PULL_NO, AF2);   // TIM1_CH3
    gpio_config(GPIOE, 14, MODE_GP, OTYPE_PP, OSPEED_2, PULL_NO, AF_NO); // LED8
    gpio_config(GPIOE, 15, MODE_GP, OTYPE_PP, OSPEED_2, PULL_NO, AF_NO); // LED6
    
    // DMA1 init
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1_Channel1->CNDTR = BUF_LEN;
    DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);
    DMA1_Channel1->CMAR = (uint32_t)adc1;
    DMA1_Channel1->CCR = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_HTIE | DMA_CCR_TCIE | DMA_CCR_EN;
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    
    // DMA2 init
    RCC->AHBENR |= RCC_AHBENR_DMA2EN;
    DMA2_Channel1->CNDTR = BUF_LEN;
    DMA2_Channel1->CPAR = (uint32_t)&(ADC2->DR);
    DMA2_Channel1->CMAR = (uint32_t)adc2;
    DMA2_Channel1->CCR = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_HTIE | DMA_CCR_TCIE | DMA_CCR_EN;
    NVIC_EnableIRQ(DMA2_Channel1_IRQn);
    
    // ADC1 and ADC2 clock init
    RCC->AHBENR |= RCC_AHBENR_ADC12EN;
    ADC1_2->CCR = ADC12_CCR_CKMODE_0 | ADC12_CCR_VREFEN;
    
    // ADC1 init
    ADC1->SMPR1 = ADC_SMPR1_SMP6_0 | ADC_SMPR1_SMP6_1; // 7.5 clock cycles sample time on channel 6
    ADC1->SMPR2 = ADC_SMPR2_SMP18_1 | ADC_SMPR2_SMP18_2; // 181.5 clock cycles sample time on channel 18
    ADC1->DIFSEL = ADC_DIFSEL_DIFSEL_5; // Channel 1 is differential
    ADC1->SQR1 = ADC_SQR1_SQ1_1 | ADC_SQR1_SQ1_4; // Select channel 18
    ADC1->CFGR = ADC_CFGR_DMAEN | ADC_CFGR_DMACFG | ADC_CFGR_CONT; // DMA enable and continious mode enable
    ADC1->CR = 0x00000000;            // ADC voltage regulator
    ADC1->CR = ADC_CR_ADVREGEN_0;     // enable sequence
    sec = 100; while (sec--) __NOP(); // and 10 us wait
    ADC1->CR |= ADC_CR_ADCAL;         // ADC calibration sequence
    while (ADC1->CR & ADC_CR_ADCAL);  // for single-ended mode
    ADC1->CR |= ADC_CR_ADCALDIF;      // ADC calibration
    ADC1->CR |= ADC_CR_ADCAL;         // sequence
    while (ADC1->CR & ADC_CR_ADCAL);  // for differential mode
    ADC1->CR |= ADC_CR_ADEN; // Enable ADC
    while (!(ADC1->ISR & ADC_ISR_ADRD));
    ADC1->CR |= ADC_CR_ADSTART; // Start ADC conversion
    
    // ADC2 init
    ADC2->SMPR1 = ADC_SMPR1_SMP1_0 | ADC_SMPR1_SMP1_1; // 7.5 clock cycles sample time on channel 1
    ADC2->SQR1 = ADC_SQR1_SQ1_0; // Select channel 1
    ADC2->CFGR = ADC_CFGR_DMAEN | ADC_CFGR_DMACFG | ADC_CFGR_CONT; // DMA enable and continious mode enable
    ADC2->CR = 0x00000000;            // ADC voltage regulator
    ADC2->CR = ADC_CR_ADVREGEN_0;     // enable sequence
    sec = 100; while (sec--) __NOP(); // and 10 us wait
    ADC2->CR |= ADC_CR_ADCAL;         // ADC calibration
    while (ADC2->CR & ADC_CR_ADCAL);  // sequence
    ADC2->CR |= ADC_CR_ADEN; // Enable ADC
    while (!(ADC2->ISR & ADC_ISR_ADRD));
    ADC2->CR |= ADC_CR_ADSTART; // Start ADC conversion
    
    // Timer1 init (phases 2 and 3)
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->SMCR    = TIM_SMCR_SMS_2; // Slave reset mode, master timer 15
    TIM1->ARR     = 719;
    TIM1->CCER    = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
    TIM1->BDTR    = TIM_BDTR_MOE | 48;
    TIM1->CR1    |= TIM_CR1_CEN;
    
    // Timer15 init (phase 1)
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    TIM15->CR2    = TIM_CR2_MMS_1; // Master reset mode
    TIM15->ARR    = 719;
    TIM15->CCER   = TIM_CCER_CC1E | TIM_CCER_CC1NE;
    TIM15->CCMR1  = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
    TIM15->BDTR   = TIM_BDTR_MOE | 48;
    TIM15->CR1   |= TIM_CR1_CEN;
    
    // Timer8 init
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    TIM8->ARR     = 719;
    TIM8->CCER    = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
    TIM8->CCMR1   = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
    TIM8->CCMR2   = TIM_CCMR2_OC3M_3 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_0
                    | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
    TIM8->CCR3    = 360;
    TIM8->BDTR    = TIM_BDTR_MOE | 0;
    TIM8->CR1    |= TIM_CR1_CEN;
    
    // Interrupt
    TIM1->DIER = TIM_DIER_UIE; // tmp
    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn); // tmp
    
    set1(3*72); // 0..719
    set2(5*36); // 0..359
    
    while (1) {
        
        if (ticks) {
            ticks--;
            
            sec++;
            if (sec>=1000) {
                sec = 0;
                
                
                
            }
            
        }
        
    }
    
}
