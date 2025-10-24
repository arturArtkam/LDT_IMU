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
// 5 будем испрользовать

// команда записи чтения
#define PRE_RREG 0b101
#define PRE_WREG 0b011

// Константы для команды чтения
#define HEADER_LEN 3
#define DATA_POS 2
#define READ_REGS_START 0
#define READ_REGS_CNT 7

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

// Тип функции обратного вызова
typedef void (*adscallback_t)(uint8_t usercmd);

//==================================================================================
// Шаблонный класс драйвера ads131_t для STM32F4
//==================================================================================

template <class PinCS,
          class PinRDY,
          class PinSYNC
>
class ads131_t
{
private:
    // --- Глобальный указатель на объект для использования в статическом обработчике прерывания ---
    static ads131_t*& instance()
    {
        static ads131_t* instance = nullptr;
        return instance;
    }

    // Буферы для SPI транзакций
    uint8_t _tx_buffer[32];
    uint8_t _rx_buffer[32];

    uint8_t _user_cmd;
    adscallback_t cbf;
    uint8_t _cmd_standby;

    volatile bool _data_ready_flag;

    void start_external_clock()
    {
        // Тактирование порта GPIOA для настройки пина PA8.
        if (!LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOA))
        {
            LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
        }

        // PA8 - в режим Alternate Function для MCO
        LL_GPIO_InitTypeDef GPIO_InitStruct;
        LL_GPIO_StructInit(&GPIO_InitStruct);
        GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH; // Высокая скорость для 8 МГц
        GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
        GPIO_InitStruct.Alternate = LL_GPIO_AF_0; // AF0 для MCO на PA8 в STM32G4
        LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        // Настройка MCO в периферии RCC
        LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_HSE, LL_RCC_MCO1_DIV_1);
    }
public:
    void spi_init()
    {
        // Тактирование SPI1, тактирование GPIOA включается в классе Pin
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

        // Настройка пинов SPI (SCK, MISO, MOSI) в режим Alternate Function
        typedef Af_pin<PORTA, 5, LL_GPIO_AF_5> Miso;
        typedef Af_pin<PORTA, 6, LL_GPIO_AF_5> Mosi;
        typedef Af_pin<PORTA, 7, LL_GPIO_AF_5> Clk;

        Miso::init();
        Mosi::init();
        Clk::init();

        // Настройка параметров SPI
        LL_SPI_InitTypeDef SPI_InitStruct;
        LL_SPI_StructInit(&SPI_InitStruct);
        SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
        SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
        SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
        SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
        SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE; // CPHA = 1
        SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
        SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV32; // Выберите подходящий делитель
        SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;

        LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);
        LL_SPI_Init(SPI1, &SPI_InitStruct);

        LL_SPI_Enable(SPI1);
    }
private:
    void exti_init()
    {
        // Тактирование SYSCFG
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

        // Связывание линии EXTI с портом пина RDY
        LL_SYSCFG_SetEXTISource(PinRDY::port_p() == GPIOA ? LL_SYSCFG_EXTI_PORTA :
                                PinRDY::port_p() == GPIOB ? LL_SYSCFG_EXTI_PORTB :
                                PinRDY::port_p() == GPIOC ? LL_SYSCFG_EXTI_PORTC :
                                LL_SYSCFG_EXTI_PORTD,
                                PinRDY::pin_num()); // pin_num() возвращает номер пина (0-15)

        // Настройка линии EXTI
        LL_EXTI_InitTypeDef EXTI_InitStruct;
        LL_EXTI_StructInit(&EXTI_InitStruct);
        EXTI_InitStruct.Line_0_31 = PinRDY::pin_mask(); // pin_mask() должен возвращает 1 << pin_n
        EXTI_InitStruct.LineCommand = ENABLE;
        EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
        EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING; // Прерывание по спадающему фронту
        LL_EXTI_Init(&EXTI_InitStruct);

        // Прерывание в NVIC
        // EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn, EXTI4_IRQn,
        // EXTI9_5_IRQn (для пинов 5-9), EXTI15_10_IRQn (для пинов 10-15)
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

    // Простая задержка
    void delay_ms(volatile uint32_t ms)
    {
        // Это очень грубая задержка, для коротких интервалов
        for (uint32_t i = 0; i < ms; ++i)
        {
            volatile uint32_t counter = 0;
            while (counter < 5000)   // Значение подобрано для ~168МГц
            {
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

    // Обработчик прерывания должен быть вызван из C-функции ISR
    static void isr_handler()
    {
        if (LL_EXTI_IsActiveFlag_0_31(PinRDY::pin_mask()) != RESET)
        {
            LL_EXTI_ClearFlag_0_31(PinRDY::pin_mask());
            instance()->_data_ready_flag = true;
        }
    }

    void reset()
    {
        SNC_on();
        delay_ms(1); // Короткая задержка
        SNC_off();
        IsOn = 1;
    }

    void init()
    {
        // Инициализация пина RDY
        PinRDY::init();

        // Настройка 8MHz, SPI и EXTI
        start_external_clock();
        spi_init();
        exti_init();

        reset();

        // Ждем, пока линия /DRDY не станет низкой в первый раз
        uint32_t timeout = 100000;
        while(PinRDY::read() && timeout--);
    }

    void transaction(uint8_t byteCount)
    {
        CS_on();
        for (uint8_t i = 0; i < byteCount; ++i)
        {
            // Ждем, пока буфер передачи не освободится
            while (!LL_SPI_IsActiveFlag_TXE(SPI1));
            LL_SPI_TransmitData8(SPI1, _tx_buffer[i]);

            // Ждем, пока не будут получены данные
            while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
            _rx_buffer[i] = LL_SPI_ReceiveData8(SPI1);
        }
        // Ждем завершения передачи
        while (LL_SPI_IsActiveFlag_BSY(SPI1));
        CS_off();
    }

    void wake_up()
    {
        _tx_buffer[0] = 0;
        _tx_buffer[1] = 0x33; // Команда WAKEUP
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
        // Копируем данные команды во внутренний _tx_buffer
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
        // Общая транзакция для чтения данных + 2 слова состояния
        // 2 (статус) + 8 (каналы) = 10 слов по 24 бита = 30 байт
        transaction(30);

        uint8_t* ptr = _rx_buffer + 3; // Пропускаем первые 3 байта статуса
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
            // Подготовить буфер для следующей транзакции (пустые команды)
			_tx_buffer[0] = 0;
			_tx_buffer[1] = 0;
		}

        if (_cmd_standby)
		{
			_cmd_standby = 0;
			_tx_buffer[0] = 0;
			_tx_buffer[1] = 0x22; // Команда STANDBY
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

/**
 * @brief Готовит АЦП к переходу в режим StandBy в конце следующего фрейма данных.
 * @tparam AdcType Тип экземпляра ads131_t.
 * @param adc Ссылка на объект АЦП.
 * @param fullStop Параметр оставлен для совместимости, но не используется.
 */
template <typename AdcType>
void StandBy(AdcType& adc, bool fullStop)
{
    (void) fullStop;
    // Просто регистрируем команду StandBy, которая будет выполнена в DataReadyHandler
    adc.add_sync_frame_standby();
}

/**
 * @brief Отправляет команду на настройку регистров АЦП.
 * @tparam AdcType Тип экземпляра ads131_t.
 * @param adc Ссылка на объект АЦП.
 */
template <typename AdcType>
void AddSyncFrameSetupADC(AdcType& adc)
{
    uint8_t cmd_buffer[14] = {0};

    union rwreg_u rw;
    rw.wr.preambula = PRE_WREG;
    rw.wr.adr = 3;      // Адрес первого регистра для записи (CLOCK)
    rw.wr.cnt = 4;      // Количество регистров для записи - 1 (т.е. 4 регистра)
    cmd_buffer[0] = rw.bt[1]; //h
    cmd_buffer[1] = rw.bt[0]; //l

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
    cmd_buffer[2] = 0; // Пустой байт
    cmd_buffer[3] = clk.bt[1]; //h
    cmd_buffer[4] = clk.bt[0]; //l

    union gain1reg_u g1;
    // g1.gain.pgagain1 = 1;
    cmd_buffer[5] = 0; // Пустой байт
    cmd_buffer[6] = g1.bt[1]; //h no gain
    cmd_buffer[7] = g1.bt[0]; //l no gain

    union gain2reg_u g2;
    // g2.gain.pgagain4 = 1;
    cmd_buffer[8] = 0; // Пустой байт
    cmd_buffer[9] = g2.bt[1]; //h no gain
    cmd_buffer[10] = g2.bt[0]; //l no gain

    union cfgreg_u cfg;
    cfg.cfg.gc_en = 1; // global chop
    cfg.cfg.gc_delay = 0b1000; //512 delay
    cmd_buffer[11] = 0; // Пустой байт
    cmd_buffer[12] = cfg.bt[1]; //h
    cmd_buffer[13] = cfg.bt[0]; //l

    // Загружаем сформированную команду в буфер АЦП
    adc.set_next_command(cmd_buffer, sizeof(cmd_buffer));
    // Регистрируем команду без обратного вызова
    adc.AddSyncFrameUserCmd(1, nullptr);
}

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
    uint8_t cmd_buffer[2] = {0};

    union rwreg_u rw;
    rw.wr.preambula = PRE_RREG;
    rw.wr.adr = READ_REGS_START;
    rw.wr.cnt = READ_REGS_CNT;
    cmd_buffer[0] = rw.bt[1]; //h
    cmd_buffer[1] = rw.bt[0]; //l

    adc.setNextCommand(cmd_buffer, sizeof(cmd_buffer));

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


#endif
