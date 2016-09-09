/*
 * cui_load_encoder.h
 *
 *  Created on: 09-Sep-2016
 *      Author: Tarang
 */

#ifndef ENCODER_CUI_LOAD_ENCODER_H_
#define ENCODER_CUI_LOAD_ENCODER_H_

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


class CUILoadEncoder: public PositionEncoder {
public:
	CUILoadEncoder();
	virtual ~CUILoadEncoder();

	uint32_t getPosition();
	void setPosition(uint32_t position);
};

#endif /* ENCODER_CUI_LOAD_ENCODER_H_ */
