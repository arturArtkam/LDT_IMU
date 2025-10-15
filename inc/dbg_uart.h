#ifndef DBG_UART_H_INCLUDED
#define DBG_UART_H_INCLUDED

#include "../cxxlog/textstream.h"

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
    /* Переводим в полудуплексный режим до включения приемо-передатчика */
    SET_BIT(_Uart::USARTn->CR3, USART_CR3_HDSEL);
    /* Разрешена работа "на передачу" */
    SET_BIT(_Uart::USARTn->CR1, USART_CR1_TE);
    SET_BIT(_Uart::USARTn->CR1, USART_CR1_UE);
}

template<Uart_num uart_n, class io_pin>
void Uart_dbg<uart_n, io_pin>::PutChar(char ch)
{
    while (!LL_USART_IsActiveFlag_TXE_TXFNF(_Uart::USARTn));
    _Uart::USARTn->TDR = ch;
}

#endif /* DBG_UART_H_INCLUDED */
