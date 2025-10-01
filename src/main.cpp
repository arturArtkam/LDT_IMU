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
    /* ��������� � �������������� ����� �� ��������� ������-����������� */
    SET_BIT(_Uart::USARTn->CR3, USART_CR3_HDSEL);
    /* ��������� ������ "�� ��������" */
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
    /* 1. ��������� ���������� ���������� � ��������� ������������ PWR */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

    /* 2. ��������� �������� ���������� (HSE) */
    LL_RCC_HSE_Enable();
    // �������, ���� HSE ����� �����
    while (LL_RCC_HSE_IsReady() != 1)
    {
        // ����� ����� �������� ������� � ����� Error_Handler()
    }
    /* 3. ��������� PLL */
    // ��������, ��� PLL ��������, ����� ��� ����������
    LL_RCC_PLL_Disable();
    while(LL_RCC_PLL_IsReady() != 0)
    {
        // ���� ����������
    }

    // �������� PLL = HSE
    LL_RCC_PLL_SetMainSource(LL_RCC_PLLSOURCE_HSE);

    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_2, 80, LL_RCC_PLLR_DIV_2);

    LL_RCC_PLL_EnableDomain_SYS();

//    // ��������� ��������� � ��������� PLL
//    // PLLM = /2
//    LL_RCC_PLL_SetM(LL_RCC_PLLM_DIV_2);
//    // PLLN = x20
//    LL_RCC_PLL_SetN(20);
//    // PLLP = /2 (��� ��������� �������� ��� ����� ��� SYSCLK)
//    LL_RCC_PLL_SetP(LL_RCC_PLLP_DIV_2);
//    // PLLQ = /2 (��� USB, SDMMC � ��.)
//    LL_RCC_PLL_SetQ(LL_RCC_PLLQ_DIV_2);
//    // PLLR = /2 (�������� ����� ��� SYSCLK �� ������ ����������� STM32)
//    LL_RCC_PLL_SetR(LL_RCC_PLLR_DIV_2);
//
//    // �������� �������� ����� PLL (PLLR ��� SYSCLK)
//    LL_RCC_PLL_EnableR();
    // �������� PLL
    LL_RCC_PLL_Enable();

    // �������, ���� PLL ����� ����� (������������)
    while (LL_RCC_PLL_IsReady() != 1)
    {
        // ������� � Error_Handler()
    }

    /* 4. ���������� �������� Flash ����� ����������� ������� */
    // ��� ���������� ����� ������� �� ������������ �� PLL �� ������� �������
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
    {
        // ������� � Error_Handler()
    }

    /* 5. ��������� ��������� ��� ��� AHB, APB1, APB2 */
    // AHB prescaler = /1
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    // APB1 prescaler = /1
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    // APB2 prescaler = /1
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

    /* 6. ����� PLL ��� ��������� ��������� ����� (SYSCLK) */
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    // �������, ���� ��������� ���� ������������ �� PLL
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
    {
        // ������� � Error_Handler()
    }

    /* 7. ���������� ���������� SystemCoreClock */
    // ��� ���������� ������������ ������� ������� ������� (��������, ��� ��������� SysTick)
    // LL_SetSystemCoreClock() ������������� ����������� �������� � SystemCoreClock
    // ��������������, ��� HSE_VALUE = 8000000 (8 ���) � stm32xxxx_hal_conf.h
    // ������: (HSE / PLLM) * PLLN / PLLR = (8��� / 2) * 20 / 2 = 40 ���
    // ����������: ��� ������ �������� ������ ����� ����� PLLP, � �� PLLR
    // SYSCLK = (HSE_VALUE / PLLM) * PLLN / PLLR = (8MHz / 2) * 20 / 2 = 40 MHz.
    // ��� �������� ����� ����� ������������ ��� ��������� SysTick � ������ ��������.
    // ����� ������� ������� SystemCoreClockUpdate(), ������� ���� ��� ����������.
    SystemCoreClockUpdate();
}

class Dummy_pin{};
Uart_dbg<UART1, Af_pin<PORTA, 9, LL_GPIO_AF_7>> dbg_uart;
ProjectLogger g_logger(dbg_uart);
Kx132 g_axel;
L3gd20h g_gyro;
Mag g_mag;
Ads ads131;

float S_x[130][5] = {0.0f, 0.0f};
extern "C" void EXTI2_IRQHandler(void)
{
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);
//    g_mag.Measure_XYZ_WithAutoSR();

}

int32_t to_signed_18bit(uint32_t raw) {
    int32_t val = (int32_t)raw;
    if (val & 0x20000) { // ���� ���������� ���� (��� 17)
        val -= 0x40000;  // ������� 2^18, �������� ������������� ��������
    }
    return val;
}

struct Res
{
    int32_t X;
    int32_t Y;
    int32_t Z;
};

static Res res_m = {0,0,0};

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
	// ����������� ������ ������ (�� ��������)
	// (����� ���������� ���������� sei() � ����������� ads131.DataReadyHandler();)
	ads131.add_sync_frame_standby();
}

/**
 * @brief ������� ��� � �������� � ����� StandBy � ����� ���������� ������ ������.
 * @tparam AdcType ��� ���������� ads131_t.
 * @param adc ������ �� ������ ���.
 * @param fullStop �������� �������� ��� �������������, �� �� ������������.
 */
template <typename AdcType>
void StandBy(AdcType& adc, bool fullStop)
{
    // ������ ������������ ������� StandBy, ������� ����� ��������� � DataReadyHandler
    adc.AddSyncFrameStandBy();
}

/**
 * @brief ���������� ������� �� ��������� ��������� ���.
 * @tparam AdcType ��� ���������� ads131_t.
 * @param adc ������ �� ������ ���.
 */
template <typename AdcType>
void AddSyncFrameSetupADC(AdcType& adc)
{
    uint8_t cmdBuffer[14] = {0}; // ��������� ����� ��� ������������ �������

    union rwreg_u rw;
    rw.wr.preambula = PRE_WREG;
    rw.wr.adr = 3;      // ����� ������� �������� ��� ������ (CLOCK)
    rw.wr.cnt = 4;      // ���������� ��������� ��� ������ - 1 (�.�. 4 ��������)
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
    cmdBuffer[2] = 0; // ������ ����
    cmdBuffer[3] = clk.bt[1]; //h
    cmdBuffer[4] = clk.bt[0]; //l

    union gain1reg_u g1;
    // g1.gain.pgagain1 = 1;
    cmdBuffer[5] = 0; // ������ ����
    cmdBuffer[6] = g1.bt[1]; //h no gain
    cmdBuffer[7] = g1.bt[0]; //l no gain

    union gain2reg_u g2;
    // g2.gain.pgagain4 = 1;
    cmdBuffer[8] = 0; // ������ ����
    cmdBuffer[9] = g2.bt[1]; //h no gain
    cmdBuffer[10] = g2.bt[0]; //l no gain

    union cfgreg_u cfg;
    cfg.cfg.gc_en = 1; // global chop
    cfg.cfg.gc_delay = 0b11; //default delay
    cmdBuffer[11] = 0; // ������ ����
    cmdBuffer[12] = cfg.bt[1]; //h
    cmdBuffer[13] = cfg.bt[0]; //l

    // ��������� �������������� ������� � ����� ���
    adc.set_next_command(cmdBuffer, sizeof(cmdBuffer));
    // ������������ ������� ��� ��������� ������
    adc.AddSyncFrameUserCmd(1, nullptr);
}

// ��������� ��� ������� ������
#define HEADER_LEN 3
#define DATA_POS 2
#define READ_REGS_START 0
#define READ_REGS_CNT 7

/**
 * @brief ���������� ������� �� ������ ��������� ��� � ������������� ���������� ������.
 * @tparam AdcType ��� ���������� ads131_t.
 * @param adc ������ �� ������ ���.
 * @param Com ������ �� ������ ��� �������� ������ (��������, ��� ����� Com).
 * @param Uart ������ �� ������ Uart, ��� ����� ������ (���� �����).
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

    // ������� ������-������� � �������� ��������
    // ��� "�����������" ������ �� adc � com, ����� ������������ �� ������
    auto callback = [&](uint8_t cmd_code) {
        // ���� ������ ������� cbReadADC ������ �����

        // �������� ������ � rx_buffer �������� �� ������� adc
        const uint8_t* adcRxBuffer = adc.getRxBuffer(); // ��������������, ��� �� �������� ������

        uint8_t* out_ptr = &com.buf[DATA_POS];

        // ���������� ����� ������� � �������, �������� � ������ ���������
        const uint8_t* data_ptr = adcRxBuffer + 3; // ���������� 3 ����� ������ �� �������

        for (uint8_t i = 0; i < READ_REGS_CNT; i++)
        {
            *out_ptr++ = data_ptr[1]; // �������� ������ ���������
            *out_ptr++ = data_ptr[0];
            data_ptr += 3; // ��������� � ���������� �������� (2 ����� ������ + 1 ������)
        }
        com.CRCSend(HEADER_LEN + READ_REGS_CNT * 2);
    };

    // ������������ ������� � ����� ���������
    adc.AddSyncFrameUserCmd(2, callback);
}

typedef Pin<PORTB, 2, LL_GPIO_MODE_OUTPUT, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_OUTPUT_PUSHPULL> SR_pin;
volatile int32_t hmc_plus[3], hmc_minus[3];
volatile int32_t hmc[3], offset[3];
uint32_t cnt = 0;

int main()
{
    SystemClock_Config_LL();

    LL_RCC_ClocksTypeDef clocks;
    LL_RCC_GetSystemClocksFreq(&clocks);

    G_red_led::init();
    G_green_led::init();
//    G_delay::init(clocks.PCLK2_Frequency);
    dbg_uart.init(&clocks);

    SR_pin::init();

    app_log::warning("Wrapper System ini ", clocks.SYSCLK_Frequency, "Hz ", clocks.HCLK_Frequency, "Hz");

    /* ������� ���� ��� ������ ��� ���� �!!!!!!!!!!! */
    for (int i = 0; i < 130; i++) {
        S_X(S_x[i], i);
    }

//    g_axel.spibus_conf(LL_SPI_POLARITY_HIGH, LL_SPI_PHASE_2EDGE, LL_SPI_BAUDRATEPRESCALER_DIV64);
//    if (!g_axel.pwr_up())
//    {
//        app_log::warning("Axel ini error!");
//    }
//    else
//    {
//        app_log::warning("Axel OK");
//    }

    ads131.init();
    WakeUp();
    AddSyncFrameSetupADC(ads131);

//    g_axel.norm_mode_2g_50hz();
//    if (!g_gyro.check_whoiam())
//    {
//        app_log::warning("Gyroscope ini error!");
//    }
//    else
//    {
//        app_log::warning("Gyroscope OK");
//    }
//    (void) res;
//    Gyro::init();
//    Mag::init();



    while (1)
    {
        if (ads131.checkDataReady())
        {
            ads131.data_ready_handler();

            if ((cnt & 511) == 0)
            {
                SR_pin::hi();
            }
            else if ((cnt & 511) == 1)
            {
                hmc_plus[0] = ads131.data[3];
                hmc_plus[1] = ads131.data[2];
                hmc_plus[2] = ads131.data[5];
            }
            else if ((cnt & 511) == 2)
            {
                SR_pin::lo();
            }
            else if ((cnt & 511) == 3)
            {
                hmc_minus[0] = -ads131.data[3];
                hmc_minus[1] = -ads131.data[2];
                hmc_minus[2] = -ads131.data[5];

                offset[0] = (hmc_plus[0] - hmc_minus[0]) / 2;
                offset[1] = (hmc_plus[1] - hmc_minus[1]) / 2;
                offset[2] = (hmc_plus[2] - hmc_minus[2]) / 2;

                if (cnt > 3) print(dbg_uart, (int32_t)hmc[0], ", ", (int32_t)hmc[1], ", ", (int32_t)hmc[2], "\n");
            }
            else
            {
                hmc[0] = ads131.data[3] - offset[0];
                hmc[1] = ads131.data[2] - offset[1];
                hmc[2] = ads131.data[5] - offset[2];


//                app_log::warning(hmc[0], ", ", hmc[1], ", ", hmc[2]);
            }

            cnt++;
        }


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
