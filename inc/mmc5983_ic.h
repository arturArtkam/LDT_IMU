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
        XOUT_H      = 0x00,
        XOUT_L,
        YOUT_H,
        YOUT_L,
        ZOUT_H,
        ZOUT_L,
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
                   ctrl1(0),
                   ctrl2(0),
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
    uint8_t ctrl1;
    uint8_t ctrl2;
    uint8_t stat;
    uint32_t auto_sr_result[3] = {0};
    int32_t sr_result[3] = {0};
    int32_t offset[3] = {0};
    int32_t field [3] = {0};
    uint32_t resultAfter_SET[3] = {0};
    uint32_t resultAfter_RESET[3] = {0};

    auto read_xyz()
    {
        Xyz_data  out{};

        static uint8_t registerValues[7] = {0};

        ctrl0 |= TM_M;

        write_reg(addr::CTRL_0, ctrl0);

//        while (G_mmc_int::read())
//        {
//            __NOP();
//        }

//        while (!(stat & MEAS_M_DONE))
//        {
//            stat = read_8b(addr::STATUS);
//        }
        while (!G_mmc_int::read())
        {
            __NOP();
        }

        Cs_pin::lo();
        Spi::exchange_8t(0x80 | addr::XOUT_L);
        for (auto i = 0; i < 6; i++)
        {
            registerValues[i] = Spi::exchange_8t();
        }

        Cs_pin::hi();

        stat |= MEAS_M_DONE;
        write_reg(addr::STATUS, stat);

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

    /* Получение результатов XYZ (3 необработанных 18-разрядных значения без знака) */
    inline void fetch_xyz(uint32_t (&result)[3])
    {
        uint8_t rawBytes[7];
        /* 7 sequential field measurement bytes */
        Cs_pin::lo();
        Spi::exchange_8t(0x80 | addr::XOUT_H);
        for (auto i = 0; i < 7; i++)
        {
            rawBytes[i] = Spi::exchange_8t();
        }
        Cs_pin::hi();

        result[0] =  ((uint32_t)rawBytes[0] << 10) |
                     ((uint32_t)rawBytes[1] <<  2) |
                    (((uint32_t)rawBytes[6] & 0xC0u) >> 6) ;
        result[1] =  ((uint32_t)rawBytes[2] << 10) |
                     ((uint32_t)rawBytes[3] <<  2) |
                    (((uint32_t)rawBytes[6] & 0x30u) >> 4) ;
        result[2] =  ((uint32_t)rawBytes[4] << 10) |
                     ((uint32_t)rawBytes[5] <<  2) |
                    (((uint32_t)rawBytes[6] & 0x0Cu) >> 2) ;

        auto extract_signed_18bit = [](uint8_t hi, uint8_t lo, uint8_t tail_shift) -> int32_t {
            uint32_t val = ((uint32_t)hi << 10) | ((uint32_t)lo << 2) | tail_shift;
            if (val & 0x20000) // знак — бит 17
                val |= 0xFFFC0000; // расширение знака до 32 бит
            return static_cast<int32_t>(val);
        };
//
    sr_result[0] = extract_signed_18bit(rawBytes[0], rawBytes[1], (rawBytes[6] & 0xC0) >> 6);
    sr_result[1] = extract_signed_18bit(rawBytes[2], rawBytes[3], (rawBytes[6] & 0x30) >> 4);
    sr_result[2] = extract_signed_18bit(rawBytes[4], rawBytes[5], (rawBytes[6] & 0x0C) >> 2);
    }

    int uSecPerMeasurement() const {
        // For current mode, how much time does a measurement take?
        static const int usecGivenBandwidth[4] = { 8000, 4000, 2000, 500 }; // times for a single measurement
        int usec = usecGivenBandwidth[3];
//        if(InAutoSRmode()) {
//            usec = usec*2 + 1; // AutoSR mode makes two measurements, and needs additional 1msec for SET-RESET
//        }
        return usec;
    };

    inline void Measure_XYZ_WithAutoSR()
    {
        fetch_xyz(auto_sr_result);
        write_reg(addr::STATUS, MEAS_M_DONE);
    }

    // Is measurement complete?
    inline bool MeasurementIsComplete() {
        uint8_t status;
//        get_reg(Register::Status,status);
        status = read_8b(addr::STATUS);
        bool complete = (status & (uint8_t)MEAS_M_DONE) != 0;
        return complete;
    }

    inline void MeasureOneTime(uint32_t (&result)[3])
    {
        // Initiate Magnetic Measurement
//        WriteControlAction(ControlRegister::Control_0, (uint8_t)Control_0_Mask::Action_TM_M);
        ctrl0 |= TM_M;
        write_reg(addr::CTRL_0, ctrl0);
        ctrl0 &= ~TM_M;
        // Wait for measurement complete
//        int usec = uSecPerMeasurement(); // 8msec for 100Hz bandwidth, rarely measurement not complete !
        int tries=0;
        for (; tries < 5; tries++)
        {
            DELAY_US(500);
            if (MeasurementIsComplete()) break; // measurement finished =>
//            usec = 1000; // wait another millisecond and try again...
        }
//        assert(MeasurementIsComplete());

        fetch_xyz(result);
    }

    /// RESET or SET generate a 500ns pulse to magnetize ANR film,
    /// after which code must wait 500uSec before making a reading.
    /// 500uSec delay from MEMSIC tech support and sample code (not in datasheet).
    /*
        RESET или SET генерирует импульс продолжительностью 500 нс для намагничивания пленки ANR,
        после чего код должен подождать 500 секунд, прежде чем выполнить считывание.
    */
    static const uint32_t RequiredWaitAfterMagnetizePulse_uSec = 500; // per MEMSIC
    /// Perform SET including required wait.
    inline void SET()
    {
//        WriteControlAction(ControlRegister::Control_0, (uint8_t)Control_0_Mask::Action_SET);
        ctrl0 |= SET_OPERATION;
        write_reg(addr::CTRL_0, ctrl0);
        DELAY_US(500);
        ctrl0 &= ~SET_OPERATION;
    }
    /// Perform RESET including required wait.
    inline void RESET()
    {
//        WriteControlAction(ControlRegister::Control_0, (uint8_t)Control_0_Mask::Action_REVERSE_SET);
        ctrl0 |= RESET_OPERATION;
        write_reg(addr::CTRL_0, ctrl0);
        DELAY_US(500);
        ctrl0 &= ~RESET_OPERATION;
    }

    inline int8_t Measure_XYZ_Field_WithResetSet()
    {
        uint32_t resultAfter_SET[3] = {0}, resultAfter_RESET[3] = {0};
        SET();   // includes required post-pulse delay (nominal 500us, implemented 1msec), now reading ::= +H + Offset
        MeasureOneTime(resultAfter_SET);
        RESET(); // includes required post-pulse delay (nominal 500us, implemented 1msec), now reading ::= -H + Offset
        MeasureOneTime(resultAfter_RESET);
        // Compute offset (zero field value) and signed result for each sensor
        for (int chIdx = 0; chIdx < 3; chIdx++)
        {
            if (chIdx > 0)
            {
                // Work-around MMC5983MA bug: With SPI interface, RESET only works on X channel
                offset[chIdx] = 0x20000; // With this bug, best we can do is use nominal 0 value...
                field [chIdx] = (int32_t)resultAfter_SET[chIdx] - 0x20000;
            }
            else
            {
                offset[chIdx] = (resultAfter_SET[chIdx] + resultAfter_RESET[chIdx]) / 2;
                field[chIdx] = ((int32_t)resultAfter_SET[chIdx] - (int32_t)resultAfter_RESET[chIdx]) / 2;
            }
        }

        return 0;
    }

    void reset_chip()
    {
        write_reg(addr::CTRL_1, SW_RST);
    }

    bool wait_otp_ready(uint32_t timeout_ms = 5)
    {
//        uint32_t t0 = get_millis();  // или ваш системный таймер
//        while (!(read_reg(STATUS) & (1 << 6))) // OTP_Rd_Done = bit 6
//        {
//            if ((get_millis() - t0) > timeout_ms)
//                return false; // таймаут
//        }
        return true;
    }

    bool set_mode()
    {
        ctrl0 |= (AUTO_SR_EN | INT_MEAS_DONE_EN);
        write_reg(addr::CTRL_0, ctrl0);

//        ctrl2 |= (CM_FREQ_2 | CM_FREQ_1 | CM_FREQ_0); //1000Hz
        ctrl2 |= (EN_PRD_SET | CM_FREQ_1 | CM_FREQ_0 | CMM_EN); //100Hz
        write_reg(addr::CTRL_2, ctrl2);
//        ctrl1 |= (BW1 | BW0);

        ctrl1 |= (BW1);
        write_reg(addr::CTRL_1, ctrl1);

        return true;
    }

    bool set_mode_1()
    {
        ctrl1 |= (BW1);
        write_reg(addr::CTRL_1, ctrl1);

        return true;
    }
};

#endif /* MMC5983_IC_H */
