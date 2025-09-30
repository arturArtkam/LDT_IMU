#ifndef _ADS131_HPP_
#define _ADS131_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_ll_conf.h"

#ifdef __cplusplus
}
#endif

#include "pin_ll_g4xx.h"

//     GPIO0
// GPIO.7- 2.097152 interval 2c
// GPIO.6 - 64 times per 2c HTC
// GPIO.6 - 69 times per 2c RTC
// 0..4 - uart
// 5 ����� �������������

// ������� ������ ������
#define PRE_RREG 0b101
#define PRE_WREG 0b011
struct rwreg
{
	uint16_t cnt: 7;
	uint16_t adr: 6;
	uint16_t preambula: 3;
};
union rwreg_u
{
	struct rwreg	wr;
	uint8_t bt[2];
};
///
struct cfgreg
{
	uint16_t cd_en: 1;
	uint16_t cd_len: 3;
	uint16_t cd_num: 3;
	uint16_t cd_allch: 1;
	uint16_t gc_en: 1;
	uint16_t gc_delay: 4;
};
union cfgreg_u
{
	struct cfgreg	cfg;
	uint8_t bt[2];
};
///
struct clockreg
{
	uint16_t pwr: 2;
	uint16_t osr: 3;
	uint16_t rezerv1: 1;
	uint16_t extref_en: 1;
	uint16_t xtal_dis: 1;
	uint16_t ch0_en: 1;
	uint16_t ch1_en: 1;
	uint16_t ch2_en: 1;
	uint16_t ch3_en: 1;
	uint16_t ch4_en: 1;
	uint16_t ch5_en: 1;
	uint16_t ch6_en: 1;
	uint16_t ch7_en: 1;
};
union clockreg_u
{
	struct clockreg	clk;
	uint8_t bt[2];
};
///
struct gain1reg
{
	uint16_t pgagain0: 3;
	uint16_t rezerv0: 1;
	uint16_t pgagain1: 3;
	uint16_t rezerv1: 1;
	uint16_t pgagain2: 3;
	uint16_t rezerv2: 1;
	uint16_t pgagain3: 3;
	uint16_t rezerv3: 1;
};
struct gain2reg
{
	uint16_t pgagain4: 3;
	uint16_t rezerv4: 1;
	uint16_t pgagain5: 3;
	uint16_t rezerv5: 1;
	uint16_t pgagain6: 3;
	uint16_t rezerv6: 1;
	uint16_t pgagain7: 3;
	uint16_t rezerv7: 1;
};
union gain1reg_u
{
	struct gain1reg	gain;
	uint8_t bt[2];
};
union gain2reg_u
{
	struct gain2reg	gain;
	uint8_t bt[2];
};

//typedef void (*adscallback_t)(uint8_t usercmd);
//
//template <uint8_t serialNo, uint8_t portCS, uint8_t bitCS, uint8_t portRDY, uint8_t bitRDY, uint8_t portSYNC, uint8_t bitSYNC>
//class ads131_t
//{
//	#define URT (*(USART_t*) (0x0800 + 0x20 * serialNo))
//	#define PCS (*(PORT_t*) (0x0400 + 0x20 * portCS))
//	#define PRDY (*(PORT_t*) (0x0400 + 0x20 * portRDY))
//	#define PSNC (*(PORT_t*) (0x0400 + 0x20 * portSYNC))
//private:
//	uint8_t UserCmd;
//	adscallback_t cbf;
//	void DataReadyIntEnable(void)
//	{
//		register8_t* rdy = &PRDY.PIN0CTRL;
//		rdy += bitRDY;
//		*rdy = PORT_ISC_FALLING_gc;
//	}
//
//	void DataReadyIntDesable(void)
//	{
//		register8_t* rdy = &PRDY.PIN0CTRL;
//		rdy += bitRDY;
//		*rdy = PORT_ISC_INTDISABLE_gc;
//	}
//
//	int32_t _convert(uint8_t* ptr)
//	{
//		unio32_t res;
//		res.B[2] = *ptr++;
//		res.B[1] = *ptr++;
//		res.B[0] = *ptr++;
//		res.B[3] = (res.B[2] & 0x80)? 0xFF:0;
//		return res.u32;
//	}
//public:
//	int32_t data[8];
//	uint8_t IsOn;
//
//	ads131_t(void)
//	{
//		PCS.DIRSET = 1<< bitCS;
//		CS_off();
//		PSNC.DIRSET = 1<< bitSYNC;
//		SNC_off();
//	}
//    USART_t& urt(void){return (*(USART_t*) (0x0800 + 0x20 * serialNo));}
//
//	void Reset(void)
//	{
//		// reset
//		SNC_on();
//		_delay_ms(1);
//		SNC_off();
//		IsOn = 1;
//	}
//
//	/// Reset, setup Uart, Enable DataReady Interrupt, wait Ready
//	void Init(void)
//	{
//		DataReadyIntDesable();
//		Reset();
//		//SPI settings are CPOL = 0 and CPHA = 1
//		URT.STATUS = USART_RXSIE_bm | USART_RXCIE_bm | USART_TXCIE_bm;
//		URT.CTRLC = USART_CMODE_MSPI_gc | USART_UCPHA_bm;// | USART_UDORD_bm ;
//		Uart.setBaud((F_CPU/1e3)/2);
//		while(!(PRDY.IN | (1 << bitRDY)));
//		DataReadyIntEnable();
//	}
//	// ����������� ������ ������ (�� ��������)
//	void AddSyncFrameStandBy(void)
//	{
//		cmdStandBy =1;
//	}
//	// ����������� ����� ����������  !!!
//	void WakeUp(void)
//	{
//		Uart.buf[0] = 0;
//		Uart.buf[1] = 0x33;
//		Uart.buf[2] = 0;
//		CS_on();
//		Uart.SpiTransaction(3);
//		Uart.buf[1] = 0;
//		CS_off();
//		IsOn = 1;
//	}
//	// ����������� ��������� � ������� ������ (�� ��������)
//	// WakeUp() ������ ������������ ����� ����� AddSyncFrameUserCmd
//	// callback: ������������� - ����������� ������ ��������� ����� ������
//	void AddSyncFrameUserCmd(uint8_t cmd, adscallback_t callback)
//	{
//		UserCmd = cmd;
//		cbf = callback;
//	}
//	void Transaction(uint8_t n)
//	{
//			CS_on();
//			Uart.SpiTransaction(n*3);
//			CS_off();
//	}
//
//	// ���������� (������ ������)
//	void _DataReady(void)
//	{
//		ads131_udr.UserDataReady();
//		PRDY.INTFLAGS = 0xFF;
//		// (���� ������)
//		GPR.GPR0 |= 0x20;
//	}
//	// �������� ���� (������ ������)
//	bool checkDataReady(void)
//	{
//		if (GPR.GPR0 & 0x20)
//		{
//			GPR.GPR0 &= ~0x20;
//			return true;
//		}
//		else return false;
//	}
//
//
//	void DataReadyHandler(void)
//	{
//		Transaction(10);
//
//		uint8_t* ptr = Uart.SpiData();
//		ptr += 3;
//		for (uint8_t i = 0; i<8; i++)
//		{
//			data[i] = _convert(ptr);
//
//			ptr += 3;
//		}
//		if (UserCmd)
//		{
//			uint8_t s = UserCmd;
//			UserCmd = 0;
//			if (cbf)
//			{
//				cbf(s);
//				cbf =0;
//			}
//			Uart.buf[0] = 0;
//			Uart.buf[1] = 0;
//		}
//		if (cmdStandBy)
//		{
//			cmdStandBy = 0;
//			Uart.buf[0] = 0;
//			Uart.buf[1] = 0x22;
//			Transaction(1);
//			Uart.buf[1] = 0;
//			IsOn = 0;
//		}
//	}
//
//    void CS_on(void){PCS.OUTCLR = 1 << bitCS;};
//    void CS_off(void){PCS.OUTSET = 1 << bitCS;};
//private:
//	uint8_t cmdStandBy;
//    void SNC_on(void){PSNC.OUTCLR = 1 << bitSYNC;};
//    void SNC_off(void){PSNC.OUTSET = 1 << bitSYNC;};
//};

// ��� ������� ��������� ������
typedef void (*adscallback_t)(uint8_t usercmd);

//==================================================================================
// ��������� ����� �������� ads131_t ��� STM32F4
//==================================================================================

template <class PinCS,
          class PinRDY,
          class PinSYNC
          >
class ads131_t
{
private:
    /* ���������� ��������� �� ������ ��� ������������� � ����������� ����������� ���������� */
    static ads131_t*& instance()
    {
        static ads131_t* instance = nullptr;
        return instance;
    }

    // ������ ��� SPI ����������
    uint8_t _tx_buffer[32];
    uint8_t _rx_buffer[32];

    uint8_t _user_cmd;
    adscallback_t cbf;
    uint8_t _cmd_standby;

    volatile bool _data_ready_flag;

    void start_external_clock()
    {
        // 1. �������� ������������ ����� GPIOA ��� ��������� ���� PA8.
        if (!LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOA))
        {
            LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
        }

        // 2. ��������� ��� PA8 � ����� Alternate Function ��� MCO
        LL_GPIO_InitTypeDef GPIO_InitStruct;
        LL_GPIO_StructInit(&GPIO_InitStruct);
        GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH; // ������� �������� ��� 8 ���
        GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
        GPIO_InitStruct.Alternate = LL_GPIO_AF_0; // AF0 ��� MCO �� PA8 � STM32G4
        LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        // 3. ��������� MCO � ��������� RCC
        // ��������: HSI (High-Speed Internal oscillator), ������� ������ �������� �� 16 ���.
        // ��������: /2, ����� �������� 16 ��� / 2 = 8 ���.
        // HSI ������� �� ��������� ����� ������, ������� ������������� ��� �������� �� �����.
        LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_HSI, LL_RCC_MCO1_DIV_2);
    }

    void spi_init()
    {
        // 1. �������� ������������ SPI1 � GPIO
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
        // ������������ GPIOA ���������� � ������ Pin

        // 2. ��������� ����� SPI (SCK, MISO, MOSI)
        // ��������������, ��� ���� SPI1 ��� ��������� � ����� Alternate Function
        // ��������, ��� SPI1 ��� PA5 (SCK), PA6 (MISO), PA7 (MOSI)
        // �� ����� ����� ���������������� �������� � ������� ������ ������ Pin
        // Pin<GPIOA, 5, LL_GPIO_MODE_ALTERNATE, ... >::init();
        // Pin<GPIOA, 6, LL_GPIO_MODE_ALTERNATE, ... >::init();
        // Pin<GPIOA, 7, LL_GPIO_MODE_ALTERNATE, ... >::init();
        // � ��������� ��� ��� AF5 (SPI1)
        // LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_5);
        // ... � �.�.
        typedef Af_pin<PORTA, 5, LL_GPIO_AF_5> Miso;
        typedef Af_pin<PORTA, 6, LL_GPIO_AF_5> Mosi;
        typedef Af_pin<PORTA, 7, LL_GPIO_AF_5> Clk;

        Miso::init();
        Mosi::init();
        Clk::init();

        // 3. ��������� ���������� SPI
        LL_SPI_InitTypeDef SPI_InitStruct;
        LL_SPI_StructInit(&SPI_InitStruct);
        SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
        SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
        SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
        SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
        SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE; // CPHA = 1
        SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
        SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16; // �������� ���������� ��������
        SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;

        LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);

        LL_SPI_Init(SPI1, &SPI_InitStruct);

        // 4. �������� SPI
        LL_SPI_Enable(SPI1);
    }

    void exti_init()
    {
        // 1. �������� ������������ SYSCFG
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

        // 2. ������� ����� EXTI � ������ ���� RDY
        LL_SYSCFG_SetEXTISource(PinRDY::port_p() == GPIOA ? LL_SYSCFG_EXTI_PORTA :
                                PinRDY::port_p() == GPIOB ? LL_SYSCFG_EXTI_PORTB :
                                PinRDY::port_p() == GPIOC ? LL_SYSCFG_EXTI_PORTC :
                                LL_SYSCFG_EXTI_PORTD,
                                PinRDY::pin_num()); // pin_n() ������ ���������� ����� ���� (0-15)

        // 3. ��������� ����� EXTI
        LL_EXTI_InitTypeDef EXTI_InitStruct;
        LL_EXTI_StructInit(&EXTI_InitStruct);
        EXTI_InitStruct.Line_0_31 = PinRDY::pin_mask(); // pin_mask() ������ ���������� 1 << pin_n
        EXTI_InitStruct.LineCommand = ENABLE;
        EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
        EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING; // ���������� �� ���������� ������
        LL_EXTI_Init(&EXTI_InitStruct);

        // 4. ���������� � NVIC
        // EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn, EXTI4_IRQn,
        // EXTI9_5_IRQn (��� ����� 5-9), EXTI15_10_IRQn (��� ����� 10-15)
        uint8_t pin = PinRDY::pin_num();
        IRQn_Type irq;
        if (pin == 0) irq = EXTI0_IRQn;
        else if (pin == 1) irq = EXTI1_IRQn;
        else if (pin == 2) irq = EXTI2_IRQn;
        else if (pin == 3) irq = EXTI3_IRQn;
        else if (pin == 4) irq = EXTI4_IRQn;
        else if (pin >= 5 && pin <= 9) irq = EXTI9_5_IRQn;
        else irq = EXTI15_10_IRQn;

        NVIC_SetPriority(irq, 1);
        NVIC_EnableIRQ(irq);
    }

    // ������� ��������
    void delay_ms(volatile uint32_t ms)
    {
        // ��� ����� ������ ��������, ��� �������� ����������
        for (uint32_t i = 0; i < ms; ++i) {
            volatile uint32_t counter = 0;
            while (counter < 5000) { // �������� ��������� ��� ~168���
                ++counter;
            }
        }
    }

    int32_t _convert(uint8_t* ptr)
    {
        union {
            uint32_t u32;
            uint8_t B[4];
        } res;
		res.B[2] = *ptr++;
		res.B[1] = *ptr++;
		res.B[0] = *ptr++;
		res.B[3] = (res.B[2] & 0x80)? 0xFF:0;
		return res.u32;
    }

public:
    int32_t data[8];
	uint8_t IsOn = 0;

    ads131_t() : _tx_buffer{0},
        _rx_buffer{0},
        _user_cmd(0),
        cbf(nullptr),
        _cmd_standby(0),
        _data_ready_flag(false)
    {
        PinCS::init();
        PinSYNC::init();

        CS_off();
        SNC_off();
        instance() = this;
    }

    // ���������� ���������� ������ ���� ������ �� C-������� ISR
    static void isr_handler()
    {
        if (LL_EXTI_IsActiveFlag_0_31(PinRDY::pin_mask()) != RESET)
        {
            LL_EXTI_ClearFlag_0_31(PinRDY::pin_mask());
            instance()->_data_ready_flag = true;
//            if (instance()) {
//                instance()->internal_dataready_handler();
//            }
        }
    }

    void reset()
    {
        SNC_on();
        delay_ms(1); // �������� ��������
        SNC_off();
        IsOn = 1;
    }

    void init()
    {
        // ������������� ���� RDY
        PinRDY::init();

        // ��������� 8MHz, SPI � EXTI
        start_external_clock();
        spi_init();
        exti_init();

        reset();

        // ����, ���� ����� /DRDY �� ������ ������ � ������ ���
        uint32_t timeout = 100000;
        while(PinRDY::read() && timeout--);
    }

    void transaction(uint8_t byteCount)
    {
        CS_on();
        for (uint8_t i = 0; i < byteCount; ++i)
        {
            // ����, ���� ����� �������� �� �����������
            while (!LL_SPI_IsActiveFlag_TXE(SPI1));
            LL_SPI_TransmitData8(SPI1, _tx_buffer[i]);

            // ����, ���� �� ����� �������� ������
            while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
            _rx_buffer[i] = LL_SPI_ReceiveData8(SPI1);
        }
        // ���� ���������� ��������
        while (LL_SPI_IsActiveFlag_BSY(SPI1));
        CS_off();
    }

    void wake_up()
    {
        _tx_buffer[0] = 0;
        _tx_buffer[1] = 0x33; // ������� WAKEUP
        _tx_buffer[2] = 0;
        transaction(3);
        IsOn = 1;
    }

    void add_sync_frame_standby()
    {
        _cmd_standby = 1;
    }

    void set_next_command(const uint8_t* cmdData, uint8_t size)
    {
        // �������� ������ ������� � ��� ���������� _tx_buffer
        // ���������, ��� size �� ��������� ������ _tx_buffer
        if (size <= sizeof(_tx_buffer))
        {
            std::memcpy(_tx_buffer, cmdData, size);
        }
    }

    void AddSyncFrameUserCmd(uint8_t cmd, adscallback_t callback)
	{
		_user_cmd = cmd;
		cbf = callback;
	}

    bool checkDataReady()
    {
        if (_data_ready_flag)
        {
            _data_ready_flag = false;
            return true;
        }
        return false;
    }

    void data_ready_handler()
    {
        // ����� ���������� ��� ������ ������ + 2 ����� ���������
        // 2 (������) + 8 (������) = 10 ���� �� 24 ���� = 30 ����
        transaction(30);

        uint8_t* ptr = _rx_buffer + 3; // ���������� ������ 3 ����� �������
        for (uint8_t i = 0; i < 8; i++)
        {
            data[i] = _convert(ptr);
            ptr += 3;
        }

        if (_user_cmd)
		{
			uint8_t s = _user_cmd;
			_user_cmd = 0;
			if (cbf)
			{
				cbf(s);
				cbf = nullptr;
			}
            // ����������� ����� ��� ��������� ���������� (������ �������)
			_tx_buffer[0] = 0;
			_tx_buffer[1] = 0;
		}

        if (_cmd_standby)
		{
			_cmd_standby = 0;
			_tx_buffer[0] = 0;
			_tx_buffer[1] = 0x22; // ������� STANDBY
			transaction(3);
			IsOn = 0;
		}
    }

    void CS_on() { PinCS::lo(); }
    void CS_off() { PinCS::hi(); }
private:
    void SNC_on() { PinSYNC::lo(); }
    void SNC_off() { PinSYNC::hi(); }
};

//extern ads131_t<ADS131> ads131;

#endif
