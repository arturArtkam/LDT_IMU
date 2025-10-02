#ifndef DELAY_TIM_H
#define DELAY_TIM_H

#include <limits>
#include "stm32g4xx.h"

template <int TimerNum>
class DelayCMSIS;

template <>
class DelayCMSIS<2>
{
public:
    static void init(uint32_t periph_clock)
    {
        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
        TIM2->PSC = (periph_clock / 1000000 - 1);
        TIM2->ARR = 0xFFFF;
        TIM2->CNT = 0;
        TIM2->EGR = TIM_EGR_UG;
        TIM2->CR1 = TIM_CR1_CEN;
    }

    static void wait_us(uint16_t us)
    {
        uint16_t start = TIM2->CNT;
        while ((uint16_t)(TIM2->CNT - start) < us);
    }

    static void wait_ms(uint32_t ms)
    {
        while (ms--) wait_us(1000);
    }
};

template <>
class DelayCMSIS<3>
{
public:
    static void init(uint32_t periph_clock)
    {
        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
        TIM3->PSC = (periph_clock / 1000000 - 1);
        TIM3->ARR = 0xFFFF;
        TIM3->CNT = 0;
        TIM3->EGR = TIM_EGR_UG;
        TIM3->CR1 = TIM_CR1_CEN;
    }

    static void wait_us(uint16_t us)
    {
        uint16_t start = TIM3->CNT;
        while ((uint16_t)(TIM3->CNT - start) < us);
    }

    static void wait_ms(uint32_t ms)
    {
        while (ms--) wait_us(1000);
    }
};

template <>
class DelayCMSIS<4>
{
public:
    static void init(uint32_t periph_clock)
    {
        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
        TIM4->PSC = (periph_clock / 1000000 - 1);
        TIM4->ARR = 0xFFFF;
        TIM4->CNT = 0;
        TIM4->EGR = TIM_EGR_UG;
        TIM4->CR1 = TIM_CR1_CEN;
    }

    static void wait_us(uint16_t us)
    {
        uint16_t start = TIM4->CNT;
        while ((uint16_t)(TIM4->CNT - start) < us);
    }

    static void wait_ms(uint32_t ms)
    {
        while (ms--) wait_us(1000);
    }
};

template <>
class DelayCMSIS<5>
{
public:
    static void init(uint32_t periph_clock)
    {
        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;
        TIM5->PSC = (periph_clock / 1000000 - 1);
        TIM5->ARR = 0xFFFFFFFF;
        TIM5->CNT = 0;
        TIM5->EGR = TIM_EGR_UG;
        TIM5->CR1 = TIM_CR1_CEN;
    }

    static void wait_us(uint32_t us)
    {
        uint32_t start = TIM5->CNT;
        while ((uint32_t)(TIM5->CNT - start) < us);
    }

    static void wait_ms(uint32_t ms)
    {
        wait_us(ms * 1000);
    }
};

/* “ребуетс€ инициализаци€ в main() перед использованием */
typedef DelayCMSIS<5> g_Delay;

#endif /* DELAY_TIM_H */
