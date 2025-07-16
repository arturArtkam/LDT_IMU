#ifndef MMC5983_IC_H
#define MMC5983_IC_H

// Bits definitions
#define MEAS_M_DONE                 (1 << 0)
#define MEAS_T_DONE                 (1 << 1)
#define OTP_READ_DONE               (1 << 4)
#define TM_M                        (1 << 0)
#define TM_T                        (1 << 1)
#define INT_MEAS_DONE_EN            (1 << 2)
#define SET_OPERATION               (1 << 3)
#define RESET_OPERATION             (1 << 4)
#define AUTO_SR_EN                  (1 << 5)
#define OTP_READ                    (1 << 6)
#define BW0                         (1 << 0)
#define BW1                         (1 << 1)
#define X_INHIBIT                   (1 << 2)
#define YZ_INHIBIT                  (3 << 3)
#define SW_RST                      (1 << 7)
#define CM_FREQ_0                   (1 << 0)
#define CM_FREQ_1                   (1 << 1)
#define CM_FREQ_2                   (1 << 2)
#define CMM_EN                      (1 << 3)
#define PRD_SET_0                   (1 << 4)
#define PRD_SET_1                   (1 << 5)
#define PRD_SET_2                   (1 << 6)
#define EN_PRD_SET                  (1 << 7)
#define ST_ENP                      (1 << 1)
#define ST_ENM                      (1 << 2)
#define SPI_3W                      (1 << 6)
#define X2_MASK                     (3 << 6)
#define Y2_MASK                     (3 << 4)
#define Z2_MASK                     (3 << 2)
#define XYZ_0_SHIFT                 10
#define XYZ_1_SHIFT                 2

template<class Spi, class Cs_pin>
class Mmc5983_ic
{
public:
    typedef enum
    {
        XOUT_L      = 0x00,
        XOUT_H,
        YOUT_L,
        YOUT_H,
        ZOUT_L,
        ZOUT_H,
        XYZ_OUT_2,
        T_OUT,
        STATUS,
        CTRL_0,
        CTRL_1,
        CTRL_2,
        CTRL_3,
        ID_1 = 0x2F
    } addr;

    struct Xyz_data
    {
        int16_t m_x;
        int16_t m_y;
        int16_t m_z;
    } __attribute__((packed, aligned(1)));

public:
    Mmc5983_ic() : ctrl0(0),
                   stat(0)
    {
        Cs_pin::init();
        Spi::init();

        Cs_pin::hi();
    }

    void spibus_conf(uint16_t cpol, uint16_t cpha, uint16_t baud_presc)
    {
        Mmc5983_ic();
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
        return read_8b(addr::ID_1) == 0x30;
    }

    uint8_t ctrl0;
    uint8_t stat;

    auto read_xyz()
    {
        Xyz_data  out{};

        static uint8_t registerValues[7] = {0};

        ctrl0 |= TM_M;

        write_reg(addr::CTRL_0, ctrl0);

        while (!(stat & MEAS_M_DONE))
        {
            stat = read_8b(addr::STATUS);
        }

        Cs_pin::lo();
        Spi::exchange_8t(0x80 | addr::XOUT_L);
        for (auto i = 0; i < 6; i++)
        {
            registerValues[i] = Spi::exchange_8t();
        }

        Cs_pin::hi();

        int16_t x, y, z;

        x = registerValues[1]; // Xout[17:10]
        x = (x << 8) | registerValues[0]; // Xout[9:2]
//        x = (x << 2) | (registerValues[6] >> 6); // Xout[1:0]
        y = registerValues[3]; // Yout[17:10]
        y = (y << 8) | registerValues[2]; // Yout[9:2]
//        y = (y << 2) | ((registerValues[6] >> 4) & 0x03); // Yout[1:0]
        z = registerValues[5]; // Zout[17:10]
        z = (z << 8) | registerValues[4]; // Zout[9:2]
//        z = (z << 2) | ((registerValues[6] >> 2) & 0x03); // Zout[1:0]

        out.m_x = x;
        out.m_y = y;
        out.m_z = z;

        return out;
    }

    bool set_mode()
    {
        bool success;

//        clearShadowBit(addr::CTRL_1, (YZ_INHIBIT | X_INHIBIT), true);
//
//        setShadowBit(addr::CTRL_2, EN_PRD_SET, true);
        // CM_FREQ[2:0] = 111
//        success = setShadowBit(addr::CTRL_2, (CM_FREQ_2 | CM_FREQ_1 | CM_FREQ_0), true);
        write_reg(addr::CTRL_1, SW_RST);
        while (!(read_8b(addr::STATUS) & OTP_READ_DONE))
        {

        }

        ctrl0 |= (AUTO_SR_EN);// | INT_MEAS_DONE_EN);

        write_reg(addr::CTRL_0, ctrl0);
//        write_reg(addr::CTRL_2, (CM_FREQ_2 | CM_FREQ_1 | CM_FREQ_0));

        return success;
    }
};

#endif /* MMC5983_IC_H */
