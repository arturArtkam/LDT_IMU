#pragma once
#ifndef SPI_DRV_G4XX_H
#define SPI_DRV_G4XX_H

#include "pin_ll_g4xx.h"
#include "stm32g4xx_ll_spi.h"

#define SPI_DR_8bit (*(__IO uint8_t* )((uint32_t)&(spi_p()->DR)))

template <uintptr_t Spi_n = 0,
          class Miso_pin = Af_pin<>,
          class Mosi_pin = Af_pin<>,
          class Clk_pin = Af_pin<>>
class Hw_spi
{
public:
    Hw_spi() = delete;
    ~Hw_spi() = delete;

    static void init()
    {
        Miso_pin::init();
        Mosi_pin::init();
        Clk_pin::init();

        Miso_pin::pullup();
    }

    constexpr static auto spi_p()
    {
        return reinterpret_cast<SPI_TypeDef* >(Spi_n);
    }

    static void rcc_config()
    {
        if (spi_p() == SPI1) LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
    #if defined (SPI2)
        else if (spi_p() == SPI2) LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
    #endif
    }

    static void rcc_deinit()
    {
//        if(spi_p() == SPI1) LL_APB1_GRP2_DisableClock(LL_APB1_GRP2_PERIPH_SPI1);
//    #if defined (SPI2)
//        else if(spi_p() == SPI2) LL_APB1_DisableClock(LL_APB1_GRP2_PERIPH_SPI1);
//    #endif
    }

    /*
        LL_SPI_POLARITY_LOW, LL_SPI_POLARITY_HIGH

        LL_SPI_PHASE_1EDGE, LL_SPI_PHASE_2EDGE

        LL_SPI_BAUDRATEPRESCALER_DIV2
        LL_SPI_BAUDRATEPRESCALER_DIV4
        LL_SPI_BAUDRATEPRESCALER_DIV8
        LL_SPI_BAUDRATEPRESCALER_DIV16
        LL_SPI_BAUDRATEPRESCALER_DIV32
        LL_SPI_BAUDRATEPRESCALER_DIV64
        LL_SPI_BAUDRATEPRESCALER_DIV128
        LL_SPI_BAUDRATEPRESCALER_DIV256
    */
    static void mode_full_duplex_8t(uint32_t cpol, uint32_t cpha, uint32_t baud_presc)
    {
        LL_SPI_InitTypeDef spi_struct;

        ///rcc_config();
        periph_powerup();
        LL_SPI_StructInit(&spi_struct);

        spi_struct.TransferDirection = LL_SPI_FULL_DUPLEX; //полный дуплекс
        spi_struct.Mode = LL_SPI_MODE_MASTER;
        spi_struct.DataWidth = LL_SPI_DATAWIDTH_8BIT; // передаем по 8 бит
        //spi_struct.SPI_CPOL = SPI_CPOL_Low;
        spi_struct.ClockPolarity = cpol; // Полярность и
        spi_struct.ClockPhase = cpha; // фаза тактового сигнала
        spi_struct.NSS = LL_SPI_NSS_SOFT; // Управлять состоянием сигнала NSS программно
        spi_struct.BaudRate = baud_presc; // Предделитель SCK
        spi_struct.BitOrder = LL_SPI_MSB_FIRST; // Первым отправляется старший бит

        LL_SPI_SetRxFIFOThreshold(spi_p(), LL_SPI_RX_FIFO_TH_QUARTER);

        LL_SPI_Init(spi_p(), &spi_struct);
        LL_SPI_Enable(spi_p());
    }

//    static void mode_full_duplex_16t(void)
//    {
//        SPI_InitTypeDef spi_struct;
//
//        SPI_StructInit(&spi_struct);
//        rcc_config();
//
//        spi_struct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //полный дуплекс
//        spi_struct.SPI_DataSize = SPI_DataSize_16b;
//        spi_struct.SPI_CPOL = SPI_CPOL_High; // Полярность и
//        spi_struct.SPI_CPHA = SPI_CPHA_2Edge; // фаза тактового сигнала
//        spi_struct.SPI_NSS = SPI_NSS_Soft; // Управлять состоянием сигнала NSS программно
//        spi_struct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; // Предделитель SCK
//        spi_struct.SPI_FirstBit = SPI_FirstBit_MSB; // Первым отправляется старший бит
//        spi_struct.SPI_Mode = SPI_Mode_Master; // Режим - мастер
//        SPI_Init(spi_p(), &spi_struct);
//        SPI_Cmd(spi_p(), ENABLE);
//    }

    static void start_reading(void){ spi_p()->CR1 |= SPI_CR1_SPE; }

    static void disable_periph(){ spi_p()->CR1 &= ~SPI_CR1_SPE; }

    static bool wait_rxne(void)
    {
        volatile auto data_tim = 0x00ff;

        while (!(spi_p()->SR & SPI_SR_RXNE) && --data_tim);
        if(data_tim == 0) return false;
        else return true;
    }

    static void wait_txe()
    {
        while(!(spi_p()->SR & SPI_SR_TXE));
    }

    static bool wait_busy()
    {
        volatile auto timer = 0x00ff;

        while ((spi_p()->SR & SPI_SR_BSY) && --timer);
        if (timer == 0) return false;
        else return true;
    }

    static bool wait_rx_fifo_empty()
    {
        volatile auto timer = 0x00ff;

        while ((LL_SPI_GetRxFIFOLevel(spi_p()) != LL_SPI_RX_FIFO_QUARTER_FULL) && --timer);

        if (timer == 0)
            return false;
        else
            return true;
    }

    static uint8_t exchange_8t(volatile uint8_t dat = 0, volatile bool tx_only = false)
    {
        (void) tx_only;
        while (!LL_SPI_IsActiveFlag_TXE(spi_p()))
        {
            __NOP();
        }

        SPI_DR_8bit = dat;

//        if (!tx_only)
//        {
        while (!LL_SPI_IsActiveFlag_RXNE(spi_p()))
        {
            __NOP();
        }
//            return SPI_DR_8bit;
//        }

        return SPI_DR_8bit;
    }

    static void periph_powerup(void)
    {
        rcc_config();
        Clk_pin::pin_mode(LL_GPIO_MODE_ALTERNATE);
        Miso_pin::pin_mode(LL_GPIO_MODE_ALTERNATE);
        Mosi_pin::pin_mode(LL_GPIO_MODE_ALTERNATE);
    }

    static void periph_lopwr(void)
    {
        rcc_deinit();
        Clk_pin::pin_mode(LL_GPIO_MODE_ANALOG);
        Miso_pin::pin_mode(LL_GPIO_MODE_ANALOG);
        Mosi_pin::pin_mode(LL_GPIO_MODE_ANALOG);
    }
};

#undef SPI_DR_8bit

#endif /* SPI_DRV_G4XX_H */
