 /*Attiny13A Qi wireless power receiver. PB4 to be connected to mosfet gate
	for modulation....
	PB3 to be connected to rectified DC voltage via 4.7/47k divider).
	
    Copyright (C) <2019>  <Vinod S http://blog.vinu.co.in>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.*/
	
#include <avr/io.h>
#define F_CPU 9600000
#include <util/delay.h>
#include <avr/power.h>
#include <avr/interrupt.h>
# define HIGH 1
# define LOW 0

void qiTransmit(uint8_t state) {
  if (state) PORTB |= _BV(PB4);
  else PORTB &= ~(_BV(PB4));
}

void adc_init(void) {
  ADMUX = 1 << REFS0;
  ADMUX |= 3;
  ADCSRA = (1 << ADEN) | (1 << ADPS2);
}

uint16_t adc_read(void) {
  ADCSRA |= 1 << ADSC;
  while (ADCSRA & (1 << ADSC));
  return ADC;
}

volatile uint8_t bit_state = 0;
void tx_byte(uint8_t data) {
  bit_state ^= 1;
  qiTransmit(bit_state);
  _delay_us(250);
  qiTransmit(bit_state);
  _delay_us(250);

  uint8_t parity = 0;
  for (int i = 0; i < 8; i++) {
    bit_state ^= 1;
    qiTransmit(bit_state);
    _delay_us(250);
    if (data & (1 << i)) {
      parity++;
      bit_state ^= 1;
    }
    qiTransmit(bit_state);
    _delay_us(250);
  }

  if (parity & 1) {
    parity = 0;
  } else
    parity = 1;

  bit_state ^= 1;
  qiTransmit(bit_state);
  _delay_us(250);

  if (parity) {
    bit_state ^= 1;
  }
  qiTransmit(bit_state);
  _delay_us(250);

  bit_state ^= 1;
  qiTransmit(bit_state);
  _delay_us(250);
  bit_state ^= 1;

  qiTransmit(bit_state);
  _delay_us(250);

}

void tx(uint8_t * data, int len) {
  uint8_t checksum = 0;
//  static uint8_t state = 0;
  for (int i = 0; i < 15; i++) {
    qiTransmit(HIGH);
    _delay_us(250);
    qiTransmit(LOW);
    _delay_us(250);
  }
  bit_state = 0;
  for (int i = 0; i < len; i++) {
    tx_byte(data[i]);
    checksum ^= data[i];
  }
  tx_byte(checksum);
}

int main()
{
	uint16_t adcv;
	uint8_t dt[8];

	clock_prescale_set(clock_div_1); //set clock to 9.6MHz, no prescaler
	adc_init();
	DDRB |= (_BV(PB4) | _BV(PB0) | _BV(PB1) | _BV(PB2));

	do
	{
		adcv = adc_read();
	} while(adcv < 423);

	_delay_ms(10);

	dt[0] = 0x01;
	dt[1] = 0xFF;
	tx(dt, 2);	//send ping response so that the transmitter identifies receiver.
	_delay_ms(10);

//	uint8_t dt[] = { 0x71, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	dt[0] = 0x71;
	dt[1] = 0x11;
	dt[2] = 0x00;
	dt[3] = 0x00;
	dt[4] = 0x00;
	dt[5] = 0x00;
	dt[6] = 0x00;
	dt[7] = 0x00;
	tx(dt, 8);	//send ID response
	_delay_ms(10);

//	uint8_t dt[] = { 0x51, 0x01, 0x00, 0x00, 0x12, 0x00 };
	dt[0] = 0x51;
	dt[1] = 0x01;
	dt[2] = 0x00;
	dt[3] = 0x00;
	dt[4] = 0x12;
	dt[5] = 0x00;
	tx(dt, 6);	//send Configuration response
	_delay_ms(10);

	uint16_t targetV = 665;
	uint8_t direction = 0;

	while(1)
	{
		int8_t error = 0;
		int16_t temp_error = 0;
		adcv = adc_read();
		temp_error = (int16_t)((targetV	) - adcv);	//1.1v adc reference. 423 equals to 5V. (4.7/47K voltage divider)

		temp_error /= 5;
		if (temp_error > 127) temp_error = 127;
		if (temp_error < -128) temp_error = -128;
		error = (int8_t) temp_error;
		if(error > 0)
			PORTB |= _BV(PB0);
		else
			PORTB &= ~_BV(PB0);
	
//		uint8_t dt[] = { 0x3, (int8_t) error };
		dt[0] = 0x03;
		dt[1] = (int8_t) error;
		tx(dt, 2);	//send error correction packet. 0x03 is error correction packet header. 1 BYTE payload, check WPC documents for more details.
		_delay_ms(10);

//		uint8_t dt[] = {0x4, 0XFF};
		dt[0] = 0x04;
		dt[1] = 0xFF;
		tx(dt, 2);	//received power indication packet. I am not sure if this is needed or not. Please read the WPC document
					//for more details. I jut implemented and it worked. But I am not sure if this is the proper way to do it.
		_delay_ms(10);
		
		switch(direction)
		{
			case 0:
				targetV -= 10;
				if(targetV < 600)
					direction = 1;
				break;
			case 1:
				targetV += 10;
				if(targetV > 750)
					direction = 0;
		}
	}
}

