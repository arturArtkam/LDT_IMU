#ifndef DELAY_TIM_H
#define DELAY_TIM_H

#include <limits>
#include <stdlib.h>
#include "../../../Lib/stm32_tim.h"

template <TIM::TimerNum num>
class Delay_tim
{
    typedef TIM::Timer<num> _Tim;

public:
    ~Delay_tim() = default;
    Delay_tim()
    {}

    static void init(uint32_t periph_clock)
    {
        _Tim::EnableClocks();
        _Tim::TIMx->PSC = (periph_clock / 1000000 - 1);  //предделитель
        _Tim::TIMx->EGR |= TIM_EGR_UG;          //апдейт для прескелера
        _Tim::TIMx->ARR = 0xFFFF;

        _Tim::UpdateCount(0);
        _Tim::Enable();
    }

    static void deinit()
    {
        LL_TIM_DeInit(_Tim::TIMx);
        _Tim::DisableClocks();
    }

	__STATIC_FORCEINLINE void wait_us(uint16_t us)
	{
        uint16_t start_time = _Tim::TIMx->CNT;
        int32_t ticks = us;
        int32_t dif = 0;

        do
        {
            if (_Tim::TIMx->CNT >= start_time)
                dif = _Tim::TIMx->CNT - start_time;
            else
                dif = 0xFFFF - start_time + _Tim::TIMx->CNT;
        } while (dif < ticks);
	}

    __STATIC_FORCEINLINE void wait_ms(uint32_t ms)
	{
        uint32_t cnt = 0;

        do
        {
            wait_us(1000);
        } while (++cnt < ms);
	}
};

#endif /* DELAY_TIM_H */
