#include "main.h"
#include "prj_logger.h"

void SystemClock_Config_LL()
{
    // --- Настройка регулятора напряжения и включение тактирования PWR ---
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

    // --- Включение внешнего резонатора (HSE) ---
    LL_RCC_HSE_Enable();
    // Ожидаем, пока HSE будет готов
    while (LL_RCC_HSE_IsReady() != 1)
    {
        // Здесь можно добавить таймаут и вызов Error_Handler()
    }
    // --- Настройка PLL ---
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

    // --- Увеличение задержки Flash перед увеличением частоты ---
    // Это КРИТИЧЕСКИ ВАЖНО сделать до переключения на PLL на высокой частоте
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
    {
        // Таймаут и Error_Handler()
    }

    // --- Настройка делителей для шин AHB, APB1, APB2 ---
    // AHB prescaler = /1
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    // APB1 prescaler = /1
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    // APB2 prescaler = /1
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

    // --- Выбор PLL как источника системных часов (SYSCLK) ---
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

Dbg_uart g_dbg_uart;
ProjectLogger g_logger(g_dbg_uart);
Ais2ih g_axel;
L3gd20h g_gyro;
Ads ads131;

float S_x[130][5] = {0.0f, 0.0f};


/**
 * @brief Безопасно отключает модуль SPI, дожидаясь завершения всех операций.
 */
void spi_safe_disable(SPI_TypeDef* SPIx)
{
    // Ждем, пока последний байт не будет передан
    while (!LL_SPI_IsActiveFlag_TXE(SPIx));
    // Ждем, пока шина не освободится
    while (LL_SPI_IsActiveFlag_BSY(SPIx));
    // Отключаем SPI
    LL_SPI_Disable(SPIx);
}

/**
 * @brief Настраивает SPI для работы с акселерометром AIS2IH и гироскопом L3GD20H.
 */
void setup_spi_for_accelerometer()
{
    // 1. Безопасно отключаем SPI
    spi_safe_disable(SPI1);

    // 2. Меняем конфигурацию
    LL_SPI_SetClockPolarity(SPI1, LL_SPI_POLARITY_HIGH); // CPOL = 1
    LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_2EDGE);      // CPHA = 1

    // 3. Включаем SPI
    LL_SPI_Enable(SPI1);
}

/**
 * @brief Настраивает SPI для работы с АЦП ADS131M08.
 */
void setup_spi_for_adc()
{
    // 1. Безопасно отключаем SPI
    spi_safe_disable(SPI1);

    // 2. Меняем конфигурацию
    LL_SPI_SetClockPolarity(SPI1, LL_SPI_POLARITY_LOW);  // CPOL = 0
    LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_2EDGE);      // CPHA = 0

    // 3. Включаем SPI
    LL_SPI_Enable(SPI1);
}

auto aps_callback = [](uint8_t* data) {
};

typedef Pin<PORTB, 2, LL_GPIO_MODE_OUTPUT, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_OUTPUT_PUSHPULL> SR_pin;
volatile int32_t hmc_plus[3] = {0}, hmc_minus[3] = {0};
volatile int32_t hmc[3] = {0}, offset[3] = {0};
uint32_t cnt = 0;

int main()
{
    SystemClock_Config_LL();

    LL_RCC_ClocksTypeDef clocks;
    LL_RCC_GetSystemClocksFreq(&clocks);

    G_red_led::init();
    G_green_led::init();
    g_Delay::init(clocks.PCLK2_Frequency);
    g_dbg_uart.init(&clocks);

    SR_pin::init();

    app_log::warning("Wrapper System ini ", clocks.SYSCLK_Frequency, "Hz ", clocks.HCLK_Frequency, "Hz");

//    /* считаем коэф для аппрох для всех К!!!!!!!!!!! */
//    for (int i = 0; i < 130; i++) {
//        S_X(S_x[i], i);
//    }

    fill_aps_buff();

    // В методе init инициализируется SPI, на коротом в том числе, висят аксель и гироскоп
    ads131.init();
    ads131.wake_up();
    AddSyncFrameSetupADC(ads131);
    // для перенастройки SPI для акселерометра и гироскопа, следует вызвать функцию setup_spi_for_accelerometer()
    setup_spi_for_accelerometer();

    if (!g_axel.init())
    {
        while (1)
        {
            G_red_led::toggle();
            DELAY_MS(25);
        }
    }

    if (!g_gyro.init())
    {
        while (1)
        {
            G_red_led::toggle();
            DELAY_MS(25);
        }
    }

    // Период калибровки в сэмплах (должен быть степенью двойки)
    constexpr uint32_t CALIBRATION_PERIOD_SAMPLES = 512;
    constexpr uint32_t CALIBRATION_PERIOD_MASK = CALIBRATION_PERIOD_SAMPLES - 1;

    // Индексы каналов АЦП для осей
    constexpr uint8_t HMC_ADC_CHANNEL_X = 3;
    constexpr uint8_t HMC_ADC_CHANNEL_Y = 2;
    constexpr uint8_t HMC_ADC_CHANNEL_Z = 5;

    // Состояния конечного автомата калибровки
    enum
    {
        SET_PULSE,
        MEASURE_PLUS,
        RESET_PULSE,
        MEASURE_MINUS_AND_CALCULATE
    };

    Vec mag = {0.0f, 0.0f, 0.0f};
    Vec axel = {0.0f, 0.0f, 0.0f};
    Vec gyro = {0.0f, 0.0f, 0.0f};

    while (1)
    {
        if (ads131.checkDataReady())
        {
            setup_spi_for_adc();
            ads131.data_ready_handler();
            setup_spi_for_accelerometer();

            if ((cnt & CALIBRATION_PERIOD_MASK) == SET_PULSE)
            {
                SR_pin::hi();
            }
            else if ((cnt & CALIBRATION_PERIOD_MASK) == MEASURE_PLUS)
            {
                hmc_plus[0] = ads131.data[HMC_ADC_CHANNEL_X];
                hmc_plus[1] = ads131.data[HMC_ADC_CHANNEL_Y];
                hmc_plus[2] = ads131.data[HMC_ADC_CHANNEL_Z];
            }
            else if ((cnt & CALIBRATION_PERIOD_MASK) == RESET_PULSE)
            {
                SR_pin::lo();
            }
            else if ((cnt & CALIBRATION_PERIOD_MASK) == MEASURE_MINUS_AND_CALCULATE)
            {
                hmc_minus[0] = -ads131.data[HMC_ADC_CHANNEL_X];
                hmc_minus[1] = -ads131.data[HMC_ADC_CHANNEL_Y];
                hmc_minus[2] = -ads131.data[HMC_ADC_CHANNEL_Z];
                // Деление на 2 заменено на сдвиг для производительности
                offset[0] = (hmc_plus[0] - hmc_minus[0]) >> 1;
                offset[1] = (hmc_plus[1] - hmc_minus[1]) >> 1;
                offset[2] = (hmc_plus[2] - hmc_minus[2]) >> 1;
            }
            else
            {
                hmc[0] = ads131.data[HMC_ADC_CHANNEL_X] - offset[0];
                hmc[1] = ads131.data[HMC_ADC_CHANNEL_Y] - offset[1];
                hmc[2] = ads131.data[HMC_ADC_CHANNEL_Z] - offset[2];

                mag = {(float)hmc[0], (float)hmc[1], -(float)hmc[2]};

                Ais2ih::Xyz_data a_res = {0, 0, 0};
                g_axel.read_all_axes(&a_res);
                axel = {(float)a_res.a_x, (float)a_res.a_z, (float)a_res.a_y};

                L3gd20h::Xyz_data w_res = {0, 0, 0};
                g_gyro.read_all_axes(&w_res);
                // Ось X гироскопа совпадает с осью Z прибора
                gyro = {(float)w_res.w_z, (float)w_res.w_y, (float)w_res.w_x};
                G_green_led::hi();
                run_aps(axel, mag, gyro, aps_callback);
                G_green_led::lo();
            }

            if ((cnt > 3) && ((cnt & 0x1FF) == 0)) {};//print(g_dbg_uart, mag.X, ", ", mag.Y, ", ", mag.Z, ", ", axel.X, ", ", axel.Y, ", ", axel.Z);
            cnt++;
        }

//        Ais2ih::Xyz_data res = {0, 0, 0};//g_axel.read_xyz();
//        g_axel.read_all_axes(&res);
//        axel = {(float)res.a_x, (float)res.a_y, (float)res.a_z};
////        print(dbg_uart, mag.X, ", ", mag.Y, ", ", mag.Z, ", ", axel.X, ", ", axel.Y, ", ", axel.Z);
//        print(dbg_uart, axel.X, ", ", axel.Y, ", ", axel.Z);
//        DELAY_MS(100);


////        Kx132::Xyz_data res = g_axel.read_xyz();
//        res_m = {
//            to_signed_18bit(g_mag.auto_sr_result[0]),
//            to_signed_18bit(g_mag.auto_sr_result[1]),
//            to_signed_18bit(g_mag.auto_sr_result[2])};// = g_res_m;//g_mag.read_xyz();
//        G_red_led::hi();
//        g_mag.Measure_XYZ_WithAutoSR();
////        g_mag.Measure_XYZ_Field_WithResetSet();
//        G_red_led::lo();
////        app_log::warning("A_X ", res.a_x, "A_Y ", res.a_y, "A_Z ", res.a_z);
////        app_log::warning("M_X ", res_m.m_x, "  M_Y ", res_m.m_y, "  M_Z ", res_m.m_z);
////        app_log::warning("M_X ", (int32_t)g_mag.auto_sr_result[0] - 0x20000, "  M_Y ", (int32_t)g_mag.auto_sr_result[1] - 0x20000, "  M_Z ", (int32_t)g_mag.auto_sr_result[2] - 0x20000);
////        app_log::warning("M_X ", (int32_t)g_mag.field[0], "  M_Y ", (int32_t)g_mag.field[1], "  M_Z ", (int32_t)g_mag.field[2]);
////        print(dbg_uart, "(", (int32_t)g_mag.field[0], ",", (int32_t)g_mag.field[1], ",", (int32_t)g_mag.field[2], "),");
//        print(dbg_uart, (int32_t)g_mag.field[0], ", ", (int32_t)g_mag.field[1], ", ", (int32_t)g_mag.field[2], "\n");
////        g_mag.SET();
////        g_mag.RESET();
//        DELAY_MS(25);
//
////        run_periodic_scale_stabilization();
    }
}

extern "C" void EXTI4_IRQHandler(void)
{
    Ads::isr_handler();
}
