#ifndef L3GD20H_IC_H
#define L3GD20H_IC_H

template<class Spi, class Cs_pin>
class L3gd20h_ic
{
public:
    typedef enum
    {
        WHO_AM_I    = 0x0F,
        CTRL1       = 0x20,
        CTRL2,
        CTRL3,
        CTRL4,
        CTRL5,
        OUT_TEMP    = 0x26,
        STATUS      = 0x27,
        XOUT_L      = 0x28,
        XOUT_H,
        YOUT_L,
        YOUT_H,
        ZOUT_L,
        ZOUT_H
    } Addr;

    struct Xyz_data
    {
        int16_t w_x;
        int16_t w_y;
        int16_t w_z;
    } __attribute__((packed, aligned(1)));

    struct Ctrl1
    {
        uint8_t yen    : 1;
        uint8_t xen    : 1;
        uint8_t zen    : 1;
        uint8_t pd     : 1;
        uint8_t bw     : 2;
        uint8_t dr     : 2;
    };

    struct Ctrl3
    {
        uint8_t owuf        : 3;
        uint8_t otdt        : 3;
        uint8_t otp         : 2;
    };

    /* Output data control register that configures the acceleration outputs */
    struct odctrl
    {
        uint8_t iir_bypass  : 1;
        uint8_t lpro        : 1;
        uint8_t fstup       : 1;
        uint8_t reserved    : 1;
        uint8_t osa3        : 1;
        uint8_t osa2        : 1;
        uint8_t osa1        : 1;
        uint8_t osa0        : 1;
    };

public:
    L3gd20h_ic()
    {
        Cs_pin::init();
        Spi::init();

        Cs_pin::hi();
    }

    void spibus_conf(uint16_t cpol, uint16_t cpha, uint16_t baud_presc)
    {
        L3gd20h_ic();
        Spi::mode_full_duplex_8b(cpol, cpha, baud_presc);
    }

    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wpedantic"
	bool init()	{		if (check_whoiam())
        {
            Ctrl1 c1 = {.yen = 1, .xen = 1, .zen = 1, .pd = 1, .bw = 2, .dr = 2};            write_reg(CTRL1, *(uint8_t*) &c1);//            write_reg(AIS_CTRL2, CTRL2_CS_PU_DISC|CTRL2_BDU|CTRL2_IF_ADD_INC|CTRL2_I2C_DISABLE);//            ctrl6_t c6 = {.reserv = 0, .lowNoise = 1, .fds = 0, .fs = FS_8g, .bw = BW_ODR_DIV_10};//            write_reg(AIS_CTRL6, *(uint8_t*) &c6);

            return true;
        }
        else
        {
            return false;
        }	}
    #pragma GCC diagnostic pop

    void write_reg(uint8_t addr, uint8_t data)
    {
        Cs_pin::lo();

        // Ожидаем, пока буфер передатчика не освободится
        while (!LL_SPI_IsActiveFlag_TXE(SPI1));
        // Передаем адрес
        LL_SPI_TransmitData8(SPI1, addr);
        // Ожидаем, пока данные не будут приняты (игнорируем их)
        while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
        (void)LL_SPI_ReceiveData8(SPI1); // Холостое чтение для очистки буфера RX

        // Ожидаем, пока буфер передатчика не освободится
        while (!LL_SPI_IsActiveFlag_TXE(SPI1));
        // Передаем данные
        LL_SPI_TransmitData8(SPI1, data);
        // Ожидаем, пока данные не будут приняты
        while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
        (void)LL_SPI_ReceiveData8(SPI1); // Холостое чтение

        // Ждем завершения всех передач
        while (LL_SPI_IsActiveFlag_BSY(SPI1));

        Cs_pin::hi();
    }

    uint8_t read_reg(uint8_t addr)
    {
        uint8_t data_from_reg = 0;

        Cs_pin::lo();

        // Шаг 1: Отправка адреса регистра с флагом чтения
        // Ожидаем, пока буфер передатчика не освободится
        while (!LL_SPI_IsActiveFlag_TXE(SPI1));
        LL_SPI_TransmitData8(SPI1, (0x80 | addr));

        // Шаг 2: Ожидание и игнорирование ответного байта
        // Ожидаем, пока не будут получены ответные "мусорные" данные
        while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
        (void)LL_SPI_ReceiveData8(SPI1); // Холостое чтение для очистки буфера RX

        // Шаг 3: Отправка "пустышки" для получения полезных данных
        // Ожидаем, пока буфер передатчика снова не освободится
        while (!LL_SPI_IsActiveFlag_TXE(SPI1));
        LL_SPI_TransmitData8(SPI1, 0x00); // Отправляем "dummy byte"

        // Шаг 4: Ожидание и чтение полезных данных из регистра
        // Ожидаем, пока не будут получены данные из регистра
        while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
        data_from_reg = LL_SPI_ReceiveData8(SPI1); // Читаем искомые данные

        // Ожидаем завершения всех операций и освобождения шины
        while (LL_SPI_IsActiveFlag_BSY(SPI1));

        Cs_pin::hi();

        return data_from_reg;
    }

    bool check_whoiam()
    {
        return read_reg(Addr::WHO_AM_I) == 0xD7;
    }

    void read_all_axes(Xyz_data* data)
    {
        union
        {
            uint8_t bytes[6];
            Xyz_data raw_data;
        } buffer;

        Cs_pin::lo();

        // 1. Отправляем адрес первого регистра (OUTX_L) с флагом чтения
        while (!LL_SPI_IsActiveFlag_TXE(SPI1));
        LL_SPI_TransmitData8(SPI1, (0x80 | XOUT_L));
        while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
        (void)LL_SPI_ReceiveData8(SPI1); // Холостое чтение

        // 2. Последовательное чтение 6 байт данных
        for (int i = 0; i < 6; ++i)
        {
            while (!LL_SPI_IsActiveFlag_TXE(SPI1));
            LL_SPI_TransmitData8(SPI1, 0x00); // Отправляем пустышку
            while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
            buffer.bytes[i] = LL_SPI_ReceiveData8(SPI1);
        }

        while (LL_SPI_IsActiveFlag_BSY(SPI1));
        Cs_pin::hi();

        // Копируем данные
        data->w_x = buffer.raw_data.w_x;
        data->w_y = buffer.raw_data.w_y;
        data->w_z = buffer.raw_data.w_z;
    }
};

#endif /* L3GD20H_IC_H */
