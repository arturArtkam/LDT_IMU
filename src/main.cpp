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

static bool Powered()
{
	return ads131.IsOn;
}
static void WakeUp(void)
{
	ads131.wake_up();
}
static void StandBy(bool fullStop)
{
	// выполняется вконце фрейма (по даташиту)
	// (после разрешения прерываний sei() в обработчике ads131.DataReadyHandler();)
	(void) fullStop;
	ads131.add_sync_frame_standby();
}

/**
 * @brief Готовит АЦП к переходу в режим StandBy в конце следующего фрейма данных.
 * @tparam AdcType Тип экземпляра ads131_t.
 * @param adc Ссылка на объект АЦП.
 * @param fullStop Параметр оставлен для совместимости, но не используется.
 */
template <typename AdcType>
void StandBy(AdcType& adc, bool fullStop)
{
    // Просто регистрируем команду StandBy, которая будет выполнена в DataReadyHandler
    adc.AddSyncFrameStandBy();
}

/**
 * @brief Отправляет команду на настройку регистров АЦП.
 * @tparam AdcType Тип экземпляра ads131_t.
 * @param adc Ссылка на объект АЦП.
 */
template <typename AdcType>
void AddSyncFrameSetupADC(AdcType& adc)
{
    uint8_t cmdBuffer[14] = {0}; // Локальный буфер для формирования команды

    union rwreg_u rw;
    rw.wr.preambula = PRE_WREG;
    rw.wr.adr = 3;      // Адрес первого регистра для записи (CLOCK)
    rw.wr.cnt = 4;      // Количество регистров для записи - 1 (т.е. 4 регистра)
    cmdBuffer[0] = rw.bt[1]; //h
    cmdBuffer[1] = rw.bt[0]; //l

    union clockreg_u clk;
    clk.clk.ch0_en = 0;
    clk.clk.ch1_en = 0;
    clk.clk.ch2_en = 1;
    clk.clk.ch3_en = 1;
    clk.clk.ch4_en = 0;
    clk.clk.ch5_en = 1;
    clk.clk.ch6_en = 0;
    clk.clk.xtal_dis = 1;
    clk.clk.pwr = 2; // hi power hi resolution
    clk.clk.osr = 0b011;//7; // max filter setup
    cmdBuffer[2] = 0; // Пустой байт
    cmdBuffer[3] = clk.bt[1]; //h
    cmdBuffer[4] = clk.bt[0]; //l

    union gain1reg_u g1;
    // g1.gain.pgagain1 = 1;
    cmdBuffer[5] = 0; // Пустой байт
    cmdBuffer[6] = g1.bt[1]; //h no gain
    cmdBuffer[7] = g1.bt[0]; //l no gain

    union gain2reg_u g2;
    // g2.gain.pgagain4 = 1;
    cmdBuffer[8] = 0; // Пустой байт
    cmdBuffer[9] = g2.bt[1]; //h no gain
    cmdBuffer[10] = g2.bt[0]; //l no gain

    union cfgreg_u cfg;
    cfg.cfg.gc_en = 1; // global chop
    cfg.cfg.gc_delay = 0b11; //default delay
    cmdBuffer[11] = 0; // Пустой байт
    cmdBuffer[12] = cfg.bt[1]; //h
    cmdBuffer[13] = cfg.bt[0]; //l

    // Загружаем сформированную команду в буфер АЦП
    adc.set_next_command(cmdBuffer, sizeof(cmdBuffer));
    // Регистрируем команду без обратного вызова
    adc.AddSyncFrameUserCmd(1, nullptr);
}

// Константы для команды чтения
#define HEADER_LEN 3
#define DATA_POS 2
#define READ_REGS_START 0
#define READ_REGS_CNT 7

/**
 * @brief Отправляет команду на чтение регистров АЦП и устанавливает обработчик ответа.
 * @tparam AdcType Тип экземпляра ads131_t.
 * @param adc Ссылка на объект АЦП.
 * @param Com Ссылка на объект для отправки ответа (например, ваш класс Com).
 * @param Uart Ссылка на объект Uart, где лежат буферы (если нужно).
 */
template <typename AdcType, typename ComType>
void AddSyncFrameReadRegsADC(AdcType& adc, ComType& com)
{
    uint8_t cmdBuffer[2] = {0};

    union rwreg_u rw;
    rw.wr.preambula = PRE_RREG;
    rw.wr.adr = READ_REGS_START;
    rw.wr.cnt = READ_REGS_CNT;
    cmdBuffer[0] = rw.bt[1]; //h
    cmdBuffer[1] = rw.bt[0]; //l

    adc.setNextCommand(cmdBuffer, sizeof(cmdBuffer));

    // Создаем лямбда-функцию в качестве коллбэка
    // Она "захватывает" ссылки на adc и com, чтобы использовать их внутри
    auto callback = [&](uint8_t cmd_code) {
        // Тело старой функции cbReadADC теперь здесь

        // Получаем доступ к rx_buffer напрямую из объекта adc
        const uint8_t* adcRxBuffer = adc.getRxBuffer(); // Предполагается, что вы добавите геттер

        uint8_t* out_ptr = &com.buf[DATA_POS];

        // Пропускаем байты статуса и команды, начинаем с данных регистров
        const uint8_t* data_ptr = adcRxBuffer + 3; // Пропускаем 3 байта ответа на команду

        for (uint8_t i = 0; i < READ_REGS_CNT; i++)
        {
            *out_ptr++ = data_ptr[1]; // Копируем данные регистров
            *out_ptr++ = data_ptr[0];
            data_ptr += 3; // Переходим к следующему регистру (2 байта данных + 1 пустой)
        }
        com.CRCSend(HEADER_LEN + READ_REGS_CNT * 2);
    };

    // Регистрируем команду с нашим коллбэком
    adc.AddSyncFrameUserCmd(2, callback);
}

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

    /* считаем коэф для аппрох для всех К!!!!!!!!!!! */
    for (int i = 0; i < 130; i++) {
        S_X(S_x[i], i);
    }

    fill_aps_buff();

//    g_axel.spibus_conf(LL_SPI_POLARITY_HIGH, LL_SPI_PHASE_2EDGE, LL_SPI_BAUDRATEPRESCALER_DIV64);
    ads131.init();
    WakeUp();
    AddSyncFrameSetupADC(ads131);

    SPI1->CR1 &= ~SPI_CR1_SPE;
//    LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_1EDGE);
//    LL_SPI_SetClockPolarity(SPI1, LL_SPI_POLARITY_HIGH);
    // 3. Настройка параметров SPI
    LL_SPI_InitTypeDef SPI_InitStruct;
    LL_SPI_StructInit(&SPI_InitStruct);
    SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
    SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
    SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
    SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
    SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE; // CPHA = 1
    SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
    SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV128; // Выберите подходящий делитель
    SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;

    LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);

    LL_SPI_Init(SPI1, &SPI_InitStruct);
    SPI1->CR1 |= SPI_CR1_SPE;

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

                DELAY_US(10);
                Ais2ih::Xyz_data a_res = {0, 0, 0};
                g_axel.read_all_axes(&a_res);
                axel = {-(float)a_res.a_x, -(float)a_res.a_z, (float)a_res.a_y};

                L3gd20h::Xyz_data w_res = {0, 0, 0};
                g_gyro.read_all_axes(&w_res);
                // Ось X гироскопа совпадает с осью Z прибора
                gyro = {(float)w_res.w_z, (float)w_res.w_y, (float)w_res.w_x};
                run_aps(axel, mag, gyro);
            }

            if ((cnt > 3) && ((cnt & 0x1FF) == 0)) ;//print(g_dbg_uart, mag.X, ", ", mag.Y, ", ", mag.Z, ", ", axel.X, ", ", axel.Y, ", ", axel.Z);
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
