#include "main.h"
#include "prj_logger.h"

template<Uart_num uart_n, class io_pin>
class Uart_dbg : public TextStream
{
private:
    typedef uart_x<uart_n, io_pin> _Uart;

public:
    virtual ~Uart_dbg() = default;
    Uart_dbg() {}

    void init(LL_RCC_ClocksTypeDef* rcc_clocks);

    virtual int GetChar(int timeout = 0) override { return timeout; }
    virtual int Keypressed() override { return -1; }
    virtual int CanSend() override { return true; };
    virtual int TxEmpty() override { return true; };
    virtual void PutChar(char ch) override;
};

template<Uart_num uart_n, class io_pin>
void Uart_dbg<uart_n, io_pin>::init(LL_RCC_ClocksTypeDef* rcc_clocks)
{
    io_pin::init();
    io_pin::pullup();

    _Uart::enable_clocks();

    set_baud(_Uart::USARTn, rcc_clocks->PCLK2_Frequency, 230400);
    /* Переводим в полудуплексный режим до включения приемо-передатчика */
    SET_BIT(_Uart::USARTn->CR3, USART_CR3_HDSEL);
    /* Разрешена работа "на передачу" */
    SET_BIT(_Uart::USARTn->CR1, USART_CR1_TE);
    SET_BIT(_Uart::USARTn->CR1, USART_CR1_UE);
}

template<Uart_num uart_n, class io_pin>
void Uart_dbg<uart_n, io_pin>::PutChar(char ch)
{
    while (!LL_USART_IsActiveFlag_TXE_TXFNF(_Uart::USARTn));
    _Uart::USARTn->TDR = ch;
}

void SystemClock_Config_LL()
{
    /* 1. Настройка регулятора напряжения и включение тактирования PWR */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

    /* 2. Включение внешнего резонатора (HSE) */
    LL_RCC_HSE_Enable();
    // Ожидаем, пока HSE будет готов
    while (LL_RCC_HSE_IsReady() != 1)
    {
        // Здесь можно добавить таймаут и вызов Error_Handler()
    }
    /* 3. Настройка PLL */
    // Убедимся, что PLL выключен, перед его настройкой
    LL_RCC_PLL_Disable();
    while(LL_RCC_PLL_IsReady() != 0)
    {
        // Ждем отключения
    }

    // Источник PLL = HSE
    LL_RCC_PLL_SetMainSource(LL_RCC_PLLSOURCE_HSE);

    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_2, 80, LL_RCC_PLLR_DIV_2);

    LL_RCC_PLL_EnableDomain_SYS();

//    // Настройка делителей и множителя PLL
//    // PLLM = /2
//    LL_RCC_PLL_SetM(LL_RCC_PLLM_DIV_2);
//    // PLLN = x20
//    LL_RCC_PLL_SetN(20);
//    // PLLP = /2 (для некоторых семейств это выход для SYSCLK)
//    LL_RCC_PLL_SetP(LL_RCC_PLLP_DIV_2);
//    // PLLQ = /2 (для USB, SDMMC и др.)
//    LL_RCC_PLL_SetQ(LL_RCC_PLLQ_DIV_2);
//    // PLLR = /2 (основной выход для SYSCLK на многих современных STM32)
//    LL_RCC_PLL_SetR(LL_RCC_PLLR_DIV_2);
//
//    // Включаем основной выход PLL (PLLR для SYSCLK)
//    LL_RCC_PLL_EnableR();
    // Включаем PLL
    LL_RCC_PLL_Enable();

    // Ожидаем, пока PLL будет готов (заблокирован)
    while (LL_RCC_PLL_IsReady() != 1)
    {
        // Таймаут и Error_Handler()
    }

    /* 4. Увеличение задержки Flash перед увеличением частоты */
    // Это КРИТИЧЕСКИ ВАЖНО сделать до переключения на PLL на высокой частоте
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
    {
        // Таймаут и Error_Handler()
    }

    /* 5. Настройка делителей для шин AHB, APB1, APB2 */
    // AHB prescaler = /1
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    // APB1 prescaler = /1
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    // APB2 prescaler = /1
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

    /* 6. Выбор PLL как источника системных часов (SYSCLK) */
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    // Ожидаем, пока системные часы переключатся на PLL
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
    {
        // Таймаут и Error_Handler()
    }

    /* 7. Обновление переменной SystemCoreClock */
    // Эта переменная используется другими частями системы (например, для настройки SysTick)
    // LL_SetSystemCoreClock() устанавливает вычисленное значение в SystemCoreClock
    // Предполагается, что HSE_VALUE = 8000000 (8 МГц) в stm32xxxx_hal_conf.h
    // Расчет: (HSE / PLLM) * PLLN / PLLR = (8МГц / 2) * 20 / 2 = 40 МГц
    // Примечание: Для многих семейств расчет будет через PLLP, а не PLLR
    // SYSCLK = (HSE_VALUE / PLLM) * PLLN / PLLR = (8MHz / 2) * 20 / 2 = 40 MHz.
    // Это значение нужно будет использовать для настройки SysTick и других таймеров.
    // Можно вызвать функцию SystemCoreClockUpdate(), которая сама все рассчитает.
    SystemCoreClockUpdate();
}

class Dummy_pin{};
Uart_dbg<UART1, Af_pin<PORTA, 9, LL_GPIO_AF_7>> dbg_uart;
ProjectLogger g_logger(dbg_uart);
Kx132 g_axel;
L3gd20h g_gyro;
Mmc5983 g_mag;

float S_x[130][5] = {0.0f, 0.0f};
Mmc5983::Xyz_data g_res_m = {0,0,0};
extern "C" void EXTI2_IRQHandler(void)
{
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);
    DELAY_US(500);
//    g_res_m = g_mag.xyz();
    g_mag.Measure_XYZ_WithAutoSR();
//    g_mag.fetch_xyz(auto_sr_result);
//    g_mag.stat |= MEAS_M_DONE;
//    g_mag.write_reg(Mmc5983::addr::STATUS, g_mag.stat);
}

int main()
{
    SystemClock_Config_LL();

    LL_RCC_ClocksTypeDef clocks;
    LL_RCC_GetSystemClocksFreq(&clocks);

    G_red_led::init();
    G_green_led::init();
    G_delay::init(clocks.PCLK2_Frequency);
    dbg_uart.init(&clocks);

    /* Настройка прерываний со стороны mmc5983 */
    G_mmc_int::init();
    G_mmc_int::pulldown();

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE2);

    LL_EXTI_InitTypeDef exti;
    LL_EXTI_StructInit(&exti);

    exti.Line_0_31 = LL_EXTI_LINE_2;
    exti.LineCommand = ENABLE;
    exti.Mode = LL_EXTI_MODE_IT;
    exti.Trigger = LL_EXTI_TRIGGER_RISING;

    LL_EXTI_Init(&exti);
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_2);
//    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_2);
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_2);

    app_log::warning("Wrapper System ini ", clocks.SYSCLK_Frequency, "Hz ", clocks.HCLK_Frequency, "Hz");

    /* считаем коэф для аппрох для всех К!!!!!!!!!!! */
    for (int i = 0; i < 130; i++) {
        S_X(S_x[i], i);
    }

    g_axel.spibus_conf(LL_SPI_POLARITY_HIGH, LL_SPI_PHASE_2EDGE, LL_SPI_BAUDRATEPRESCALER_DIV64);
    if (!g_axel.pwr_up())
    {
        app_log::warning("Axel ini error!");
    }
    else
    {
        app_log::warning("Axel OK");
    }
    g_mag.reset_chip();
    DELAY_MS(10);
    g_mag.set_mode();
    NVIC_EnableIRQ(EXTI2_IRQn);
    if (!g_mag.check_whoiam())
            app_log::warning("Mag error!");
    else
        app_log::warning("MAg OK");

    g_axel.norm_mode_2g_50hz();
    if (!g_gyro.check_whoiam())
    {
        app_log::warning("Gyroscope ini error!");
    }
    else
    {
        app_log::warning("Gyroscope OK");
    }
//    (void) res;
//    Gyro::init();
//    Mag::init();

    while (1)
    {
//        Kx132::Xyz_data res = g_axel.read_xyz();
        Mmc5983::Xyz_data res_m = {
            ((int32_t)g_mag.auto_sr_result[0] - 0x20000) >> 2,
            ((int32_t)g_mag.auto_sr_result[1] - 0x20000) >> 2,
            ((int32_t)g_mag.auto_sr_result[2] - 0x20000) >> 2};// = g_res_m;//g_mag.read_xyz();
//        app_log::warning("A_X ", res.a_x, "A_Y ", res.a_y, "A_Z ", res.a_z);
//        app_log::warning("M_X ", res_m.m_x, "  M_Y ", res_m.m_y, "  M_Z ", res_m.m_z);
        app_log::warning("M_X ", (int32_t)(g_mag.auto_sr_result[0] - 0x20000), "  M_Y ", (int32_t)(g_mag.auto_sr_result[1] - 0x20000), "  M_Z ", (int32_t)(g_mag.auto_sr_result[2] - 0x20000));
        DELAY_MS(100);

//        run_periodic_scale_stabilization();
    }
}
