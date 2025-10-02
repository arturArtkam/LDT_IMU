#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_ll_conf.h"

#ifdef __cplusplus
}
#endif

#include "pin_ll_g4xx.h"

typedef Pin<PORTA, 15, LL_GPIO_MODE_OUTPUT, LL_GPIO_SPEED_FREQ_MEDIUM, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_PULL_NO> G_green_led;
typedef Pin<PORTB, 3, LL_GPIO_MODE_OUTPUT, LL_GPIO_SPEED_FREQ_MEDIUM, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_PULL_NO> G_red_led;
typedef Pin<PORTB, 11, LL_GPIO_MODE_OUTPUT, LL_GPIO_SPEED_FREQ_MEDIUM> G_pin_int;

#include "uart_ll_g4xx.h"
#include "delay.h"
#include "mb85_ic.h"

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

//#define MAX_W_STOP 1

//typedef Pin<PORTA, 15, LL_GPIO_MODE_OUTPUT, LL_GPIO_SPEED_FREQ_MEDIUM, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_PULL_NO> G_red_led;
//typedef Pin<PORTB, 3, LL_GPIO_MODE_OUTPUT, LL_GPIO_SPEED_FREQ_MEDIUM, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_PULL_NO> G_green_led;
//typedef Pin<PORTB, 11, LL_GPIO_MODE_INPUT, LL_GPIO_SPEED_FREQ_MEDIUM> G_pin_int;

//typedef Delay_tim<TIM::TIM_3> G_delay;
#define DELAY_MS(ms) g_Delay::wait_ms(ms)
#define DELAY_US(us) g_Delay::wait_us(us)

typedef Pin<PORTB, 3, LL_GPIO_MODE_OUTPUT, LL_GPIO_SPEED_FREQ_HIGH> Fram_cs_pin;
typedef Hw_spi<SPI2_BASE, Af_pin<PORTB, 15>, Af_pin<PORTB, 14>, Af_pin<PORTB, 13>> Fram_spi;
typedef ic_mb85<MB85RS256_SIZE, Fram_spi, Fram_cs_pin> G_fram;

#include "ais2ih.h"
#include "l3gd20h_ic.h"
#include "ads131_ic.h"
//#include "mmc5983_ic.h"

typedef Hw_spi<SPI1_BASE, Af_pin<PORTA, 6, LL_GPIO_AF_5>, Af_pin<PORTA, 7, LL_GPIO_AF_5>, Af_pin<PORTA, 5, LL_GPIO_AF_5>> Shared_spi;
typedef Ais2ih_ic<Shared_spi, Pin<PORTA, 3, LL_GPIO_MODE_OUTPUT>> Ais2ih;
typedef L3gd20h_ic<Shared_spi, Pin<PORTA, 4, LL_GPIO_MODE_OUTPUT>> L3gd20h;
//typedef Mmc5983_ic<Shared_spi, Pin<PORTB, 1, LL_GPIO_MODE_OUTPUT>> Mmc5983;
// Пины для АЦП
// CS - Output
using AdcCsPin = Pin<PORTB, 0, LL_GPIO_MODE_OUTPUT, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_OUTPUT_PUSHPULL>;

// RDY - Input, Pull-up (линия /DRDY обычно требует подтяжки)
using AdcRdyPin = Pin<PORTA, 4, LL_GPIO_MODE_INPUT, LL_GPIO_SPEED_FREQ_LOW, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_PULL_UP>;

// SYNC/RESET - Output
using AdcSyncPin = Pin<PORTB, 1, LL_GPIO_MODE_OUTPUT, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_OUTPUT_PUSHPULL>;

// Также не забудьте настроить пины для самого SPI1 (SCK, MISO, MOSI)
// SCK -> PA5, MISO -> PA6, MOSI -> PA7
// Эти пины должны быть настроены в режиме Alternate Function AF5
using Ads = ads131_t<AdcCsPin, AdcRdyPin, AdcSyncPin>;

struct Vec
{
    float X;
    float Y;
    float Z;
};

struct Matrix_3_3
{
    float XX; float YX; float ZX;
    float XY; float YY; float ZY;
    float XZ; float YZ; float ZZ;
};

struct Cal
{
    struct Vec offset;
    struct Matrix_3_3 sens;
};

struct Set
{
   uint32_t MA_WINDOW_SIZE; //32
   uint32_t MLD_WINDOW_SIZE; //32
   uint32_t N; //128
   uint32_t K; //128
   uint32_t auto_delta; //1
   uint32_t INTERRUPT_BY_ANGLE_PATH; // 0
   uint16_t MOVEMENT_CMP_VALUE; //300
   float W_MIN; //3.14
   float W_MAX; //18.84
   float K_predict; //0.5
   float APS_DELTA; //PI/360
   float history_angle; //(20*PI)/180
   float K_delta; //2.5
};

enum Aps_state
{
    APS_READY,
    APS_WAIT_NEXT_POINT,
    APS_COMPLETE
};

enum Fram_markup
{
    G_OFFSET_ADDR = 0,
    M_OFFSET_ADDR = G_OFFSET_ADDR + sizeof(Cal),
    W_OFFSET_ADDR = G_OFFSET_ADDR + 2 * sizeof(Cal),
    SETTINGS_ADDR = G_OFFSET_ADDR + 3 * sizeof(Cal),
};

class Mag
{
public:
    Vec read_xyz()
    {
        Vec res = {0, 0, 0};
        return res;
    }
};

extern Ais2ih g_axel;
extern L3gd20h g_gyro;
extern Mag g_mag;

Vec vctr_summ(Vec a, Vec b);
Vec vctr_diff(Vec a, Vec b);
Vec vctr_mltp(Vec a, Vec b);
Vec vctr_mltp_n(float a, Vec b);
Vec callibrate(Vec Data, Cal calib);
float predict(float *abc, float *data, float S_x[5], int N);
void S_X(float S_x[5], int N);
float modul(Vec a);

void run_aps(Vec& axel, Vec& mag, Vec& gyro);

#endif /* MAIN_H_INCLUDED */
