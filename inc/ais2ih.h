#ifndef AIS2IH_IC_H
#define AIS2IH_IC_H

#define AIS_OUT_T_L			0x0D // R 00001101 00000000 Temp sensor output
#define AIS_OUT_T_H			0x0E // R 00001110 00000000
#define AIS_WHO_AM_I		0x0F // R 00001111 01000100 Who am I ID
// RESERVED - 10-1F - RESERVED
#define AIS_CTRL1			0x20 //R/W 00100000 00000000 Control registers
#define AIS_CTRL2			0x21 //R/W 00100001 00000100

#define CTRL2_BOOT			(1<<7)
#define CTRL2_RESET			(1<<6)
//#define CTRL2_BOOT (1<<5)
#define CTRL2_CS_PU_DISC	(1<<4)
#define CTRL2_BDU			(1<<3)
#define CTRL2_IF_ADD_INC	(1<<2)
#define CTRL2_I2C_DISABLE	(1<<1)
#define CTRL2_SIM			(1<<0)

#define AIS_CTRL3			0x22 //R/W 00100010 00000000
#define AIS_CTRL4_INT1_PAD_CTRL 0x23 //R/W 00100011 00000000
#define AIS_CTRL5_INT2_PAD_CTRL 0x24 //R/W 00100100 00000000

#define AIS_CTRL6			0x25 //R/W 00100101 00000000


#define AIS_OUT_T			0x26 //R 00100110 00000000 Temp sensor output
#define AIS_STATUS			0x27 //R 00100111 00000000 Status data register
// Output registers
#define AIS_OUT_X_L			0x28 // R00101000 00000000
#define AIS_OUT_X_H			0x29 //R 00101001 00000000
#define AIS_OUT_Y_L			0x2A //R 00101010 00000000
#define AIS_OUT_Y_H			0x2B //R 00101011 00000000
#define AIS_OUT_Z_L			0x2C //R 00101100 00000000
#define AIS_OUT_Z_H			0x2D //R 00101101 00000000

#define AIS_MODE_HI	1

typedef struct
{
	uint8_t reserv: 2;
	uint8_t lowNoise: 1;
	uint8_t fds: 1;
	uint8_t fs: 2;
	uint8_t bw: 2;
} ctrl6_t;

enum
{
	BW_ODR_DIV_2 = 0,
	BW_ODR_DIV_4 = 1,
	BW_ODR_DIV_10 = 2,
	BW_ODR_DIV_20 = 3,
};
enum
{
	FS_2g = 0,
	FS_4g = 1,
	FS_8g = 2,
	FS_16g = 3,
};

typedef struct
{
	uint8_t lpmode: 2;
	uint8_t mode: 2;
	uint8_t odr: 4;
} ctrl1_t;

enum
{
	ODR_12HZ = 2,
	ODR_25HZ = 3,
	ODR_50HZ = 4,
	ODR_100HZ = 5,
	ODR_200HZ = 6,
    ODR_400_200HZ = 7,
    ODR_800_200HZ = 8,
	ODR_1600_200HZ = 9
};

template<class Spi, class Cs_pin>
class Ais2ih_ic
{
public:
    struct Xyz_data
    {
        int16_t a_x;
        int16_t a_y;
        int16_t a_z;
    } __attribute__((packed, aligned(1)));

public:
    Ais2ih_ic()
    {
        Cs_pin::init();
        Spi::init();
        Cs_pin::hi();
    }

    void spibus_conf(uint16_t cpol, uint16_t cpha, uint16_t baud_presc)
    {
        Ais2ih_ic();
        Spi::mode_full_duplex_8t(cpol, cpha, baud_presc);
    }

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

    int16_t read_reg_16b(uint8_t addr)
    {
        union
        {
            int16_t out;
            uint8_t p[2];
        };

        Cs_pin::lo();

        // Шаг 1: Отправка адреса и холостое чтение "мусорного" байта
        // Убедимся, что буфер TX пуст перед отправкой
        while (!LL_SPI_IsActiveFlag_TXE(SPI1));
        LL_SPI_TransmitData8(SPI1, (0x80 | addr));
        // Ждем ответный байт (который нам не нужен) и очищаем буфер RX
        while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
        (void)LL_SPI_ReceiveData8(SPI1); // Холостое чтение

        // Шаг 2: Отправка "пустышки" для получения первого байта данных
        while (!LL_SPI_IsActiveFlag_TXE(SPI1));
        LL_SPI_TransmitData8(SPI1, 0x00); // Отправляем dummy byte
        // Ждем и читаем первый полезный байт данных
        while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
        p[0] = LL_SPI_ReceiveData8(SPI1);

        // Шаг 3: Отправка второй "пустышки" для получения второго байта данных
        while (!LL_SPI_IsActiveFlag_TXE(SPI1));
        LL_SPI_TransmitData8(SPI1, 0x00); // Отправляем еще один dummy byte
        // Ждем и читаем второй полезный байт данных
        while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
        p[1] = LL_SPI_ReceiveData8(SPI1);

        // Ждем завершения всех операций и освобождения шины
        while (LL_SPI_IsActiveFlag_BSY(SPI1));

        Cs_pin::hi();

        return out;
    }

    bool check_whoiam()
    {
        return read_reg(AIS_WHO_AM_I) == 0x44;
    }
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wpedantic"
    bool init()
    {
        ctrl1_t c1 = {.lpmode = 0, .mode = AIS_MODE_HI, .odr = ODR_1600_200HZ};
        write_reg(AIS_CTRL1, *(uint8_t*) &c1);
        write_reg(AIS_CTRL2, CTRL2_CS_PU_DISC|CTRL2_BDU|CTRL2_IF_ADD_INC|CTRL2_I2C_DISABLE);
        ctrl6_t c6 = {.reserv = 0, .lowNoise = 1, .fds = 0, .fs = FS_8g, .bw = BW_ODR_DIV_10};
        write_reg(AIS_CTRL6, *(uint8_t*) &c6);

        return check_whoiam();
    }
    #pragma GCC diagnostic pop
	int8_t read_T(void)	{		int8_t r = read_reg(AIS_OUT_T);		return 25 + r;	}

    void read_all_axes(Xyz_data* data)
    {
        union
        {
            uint8_t bytes[6];
            Xyz_data raw_data;
        } buffer;

        Cs_pin::lo();

        while (!(read_reg(AIS_STATUS) & 0b00000001)) __NOP();

        Cs_pin::hi();

        Cs_pin::lo();
        // Отправляем адрес первого регистра (OUTX_L) с флагом чтения
        while (!LL_SPI_IsActiveFlag_TXE(SPI1));
        LL_SPI_TransmitData8(SPI1, (0x80 | AIS_OUT_X_L));
        while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
        (void)LL_SPI_ReceiveData8(SPI1); // Холостое чтение

        // Последовательное чтение 6 байт данных
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
        data->a_x = buffer.raw_data.a_x;
        data->a_y = buffer.raw_data.a_y;
        data->a_z = buffer.raw_data.a_z;
    }
};

#endif /* AIS2IH_IC_H */
