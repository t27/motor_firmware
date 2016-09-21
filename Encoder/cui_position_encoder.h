/*
 * cui_position_encoder.h
 *
 *  Created on: Aug 26, 2016
 *      Author: Kaustubh
 */

#ifndef ENCODER_CUI_POSITION_ENCODER_H_
#define ENCODER_CUI_POSITION_ENCODER_H_

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "a_position_encoder.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/qei.h"
#include "../commondef.h"


class CUIPositionEncoder: public PositionEncoder {

public:
	CUIPositionEncoder();
	~CUIPositionEncoder();

//	void setIndexHandler(void (*handler)());
	uint32_t getPosition();
	uint32_t getVelocity();
	void setPosition(uint32_t position);
	uint16_t getIndex();
};

#endif /* ENCODER_CUI_POSITION_ENCODER_H_ */
