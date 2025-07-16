#ifndef L3GD20H_IC_H
#define L3GD20H_IC_H

template<class Spi, class Cs_pin>
class L3gd20h_ic
{
public:
    typedef enum
    {
        WHO_AM_I    = 0x0F,
        CNTL1       = 0x20,
        CNTL2,
        CNTL3,
        CNTL4,
        CNTL5,
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
        uint8_t bw_0   : 1;
        uint8_t bw_1   : 1;
        uint8_t dr_0   : 1;
        uint8_t dr_1   : 1;
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

    void write_reg(uint8_t adr, uint8_t data)
    {
        Cs_pin::lo();

        __NOP();
        __NOP();
        __NOP();

        Spi::exchange_8t(adr);
        Spi::exchange_8t(data);

        __NOP();
        __NOP();
        __NOP();

        Cs_pin::hi();
    }

    uint8_t read_8b(uint8_t adr)
    {
        uint8_t tmp = 0;

        Cs_pin::lo();

        __NOP();
        __NOP();
        __NOP();

        /* Для чтения регистров необходимо выставить старший бит в адресе. */
        Spi::exchange_8t(0b10000000 | adr);
        tmp = Spi::exchange_8t();

        __NOP();
        __NOP();
        __NOP();

        Cs_pin::hi();

        return tmp;
    }

    int16_t read_16b(uint8_t axis)
    {
        union
        {
            int16_t out;
            uint8_t p[2];
        };

        Cs_pin::lo();

        /* программирование режима работы (чтение с автоинкрементом, за автоинкремент отвечает 7й бит) */
        Spi::exchange_8t(0b11000000 | axis);
        /* 1й байт данных */
        p[0] = Spi::exchange_8t();
        /* 2й байт данных */
        p[1] = Spi::exchange_8t();

        Cs_pin::hi();

        return out;
    }

    bool check_whoiam()
    {
        return read_8b(Addr::WHO_AM_I) == 0xD7;
    }

    auto read_xyz()
    {
        Xyz_data out{};

        out.w_x = read_16b(Addr::XOUT_L);
        out.w_y = read_16b(Addr::YOUT_L);
        out.w_z = read_16b(Addr::ZOUT_L);

        return out;
    }

    bool set_mode()
    {
        if (check_whoiam())
        {
            return true;
        }
        else
        {
            return false;
        }
    }
};

#endif /* L3GD20H_IC_H */
