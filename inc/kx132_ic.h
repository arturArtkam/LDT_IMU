#ifndef KX132_IC_H
#define KX132_IC_H

#include "spi_ll_g4xx.h"

template<class Spi, class Cs_pin>
class Kx132_ic
{
public:
    typedef enum
    {
        MAIN_ID     = 0x01,
        PART_ID     = 0x02,
        XOUT_L      = 0x08,
        XOUT_H,
        YOUT_L,
        YOUT_H,
        ZOUT_L,
        ZOUT_H,
        COTR        = 0X12,
        WHO_AM_I    = 0x13,
        INS1        = 0x16,
        INS2        = 0x17,
        INS3        = 0x18,
        CNTL1       = 0x1B,
        CNTL2,
        CNTL3,
        CNTL4,
        CNTL5,
        CNTL6,
        ODCNTL      = 0X21,
        REG_7F      = 0x7f
    } addr;

    struct Xyz_data
    {
        int16_t a_x;
        int16_t a_y;
        int16_t a_z;
    } __attribute__((packed, aligned(1)));

    struct Ctrl1
    {
        uint8_t tpe         : 1;
        uint8_t reserved    : 1;
        uint8_t tdte        : 1;
        uint8_t gsel0       : 1;
        uint8_t gsel1       : 1;
        uint8_t dr_dye      : 1;
        uint8_t res         : 1;
        uint8_t pc1         : 1;
    };

    struct Ctrl3
    {
        uint8_t owuf        : 3;
        uint8_t otdt        : 3;
        uint8_t otp         : 2;
    };

    /* Output data control register that configures the acceleration outputs */
    struct Odctrl
    {
        uint8_t IIR_BYPASS  : 1;
        uint8_t LPRO        : 1;
        uint8_t FSTUP       : 1;
        uint8_t Reserved    : 1;
        uint8_t OSA3        : 1;
        uint8_t OSA2        : 1;
        uint8_t OSA1        : 1;
        uint8_t OSA0        : 1;
    };

public:
    Kx132_ic()
    {
        Cs_pin::init();
        Spi::init();

        Cs_pin::hi();
    }

    void spibus_conf(uint16_t cpol, uint16_t cpha, uint16_t baud_presc)
    {
        Kx132_ic();
        Spi::mode_full_duplex_8t(cpol, cpha, baud_presc);
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
        //программирование режима работы (чтение с автоинкрементом)
        Spi::exchange_8t(0x80 | axis);
        //1й байт данных
        p[0] = Spi::exchange_8t();
        //2й байт данных
        p[1] = Spi::exchange_8t();

        Cs_pin::hi();

        return out;
    }

    bool check_whoiam()
    {
        return read_8b(addr::WHO_AM_I) == 0x3D;
    }

    bool check_cotr()
    {
        return read_8b(addr::COTR) == 0x55;
    }

    auto read_xyz()
    {
        Xyz_data  out{};
        uint8_t stat = 0;

        while (!(stat & 0x10))
        {
            stat = read_8b(addr::INS2);
        }

        out.a_x = read_16b(addr::XOUT_L);
        out.a_y = read_16b(addr::YOUT_L);
        out.a_z = read_16b(addr::ZOUT_L);

        return out;
    }

    bool norm_mode_2g_50hz()
    {
        union
        {
            uint8_t cmd_ctrl1;
            Ctrl1   ctrl1_reg;
        };

        union
        {
            uint8_t cmd_odctrl;
            Odctrl odctrl_reg;
        };

        odctrl_reg.OSA0 = 1;
        odctrl_reg.OSA3 = 1;

        ctrl1_reg.pc1 = 1;
        ctrl1_reg.res = 1;
        ctrl1_reg.dr_dye = 1;

        /* Выбор диапазона измерения. */
        ctrl1_reg.gsel0 = 1;
        ctrl1_reg.gsel1 = 0;

        if (check_whoiam())
        {
            write_reg(addr::CNTL1, 0);
            write_reg(addr::ODCNTL, cmd_odctrl);
            write_reg(addr::CNTL1, cmd_ctrl1);

            return true;
        }
        else
        {
            return false;
        }
    }

    /* Алгоритм описан в TN027 "Power on procedure." */
    bool pwr_up()
    {
        write_reg(addr::REG_7F, 0);
        write_reg(addr::CNTL2, 0);
        write_reg(addr::CNTL2, 0x80);

        for (auto attempts = 0; attempts < 10 && !check_whoiam(); attempts++)
        {
            if (attempts >= 10) return false;
        }

        for (auto attempts = 0; attempts < 10 && !check_cotr(); attempts++)
        {
            if (attempts >= 10) return false;
        }
        return true;
    }
};

#endif /* KX132_IC_H */
