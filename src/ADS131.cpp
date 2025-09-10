#include <avr/io.h>
#include <tools.h>
#include "ADS131.hpp"

//ADS131=uart_spi_mode,CSport,CSpin,RDYport,RDYpin,SYNCport,SYNCpin"
ADS131_udr ads131_udr;
ads131_t<ADS131> ads131;

#define ADS131_RDY GETARG_4(ADS131)
#if   ADS131_RDY == PA
ISR(PORTA_PORT_vect) {ads131._DataReady();}
#elif ADS131_RDY == PB
ISR(PORTB_PORT_vect) {ads131._DataReady();}
#elif ADS131_RDY == PC
ISR(PORTC_PORT_vect) {ads131._DataReady();}
#elif ADS131_RDY == PD
ISR(PORTD_PORT_vect) {ads131._DataReady();}
#elif ADS131_RDY == PE
ISR(PORTE_PORT_vect) {ads131._DataReady();}
#elif ADS131_RDY == PF
ISR(PORTF_PORT_vect) {ads131._DataReady();}
#endif
