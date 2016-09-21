/*
 * ForceSensor.h
 *
 *  Created on: 25-Jul-2016
 *      Author: Tarang
 */

#ifndef HALCLASSES_FORCESENSOR_FORCESENSOR_H_
#define HALCLASSES_FORCESENSOR_FORCESENSOR_H_
#include <stdint.h>
class ForceSensor {
public:
	ForceSensor();
	virtual ~ForceSensor();
	virtual uint32_t getTorque() = 0;
};

#endif /* HALCLASSES_FORCESENSOR_FORCESENSOR_H_ */
