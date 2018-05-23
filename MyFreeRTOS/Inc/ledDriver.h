/*******************************************************************************
*  FILENAME: ledDriver.h
*
*  DESCRIPTION: ќписани€ класса работы со светодиодами платы
*
*  Created on: 21 ма€ 2018 г.
*  Author: Alexander Chashkin
******************************************************************************/

#ifndef LED_DRIVER_H
#define LED_DRIVER_H

#include "types.h"

#define LEDS_NUMBER    4

class cLedDriver {
	public:
		explicit cLedDriver(void);
		void ledOn(const tU8 led) const;
	  void ledOff(const tU8 led) const;
	  void ledToggle(const tU8 led) const;
	private:
		static tPort ledsPort[LEDS_NUMBER];
	  static const tU32 ledsPin[LEDS_NUMBER];
};

#endif //LED_DRIVER_H




