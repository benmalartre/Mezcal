//--------------------------------------------------------
// COMMON
//--------------------------------------------------------
#ifndef _COMMON_H_
#define _COMMON_H_

namespace BOB
{
#define DEGREE_TO_RADIAN 0.0174533
#define RADIAN_TO_DEGREE 57.2958

/* a=target variable, b=bit number to act upon 0-n */
#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1<<(b)))
#define BIT_CHECK(a,b) ((a) & (1<<(b)))

/* x=target variable, y=mask */
#define BITMASK_SET(x,y) ((x) |= (y))
#define BITMASK_CLEAR(x,y) ((x) &= (~(y)))
#define BITMASK_FLIP(x,y) ((x) ^= (y))
#define BITMASK_CHECK(x,y) (((x) & (y)) == (y))

#define RESCALE(value, inmin, inmax, outmin, outmax) (value -inmin)*(outmax-outmin)/(inmax-inmin)+outmin
#define CLAMP(value, low, high)(value < low) ? low : ((value > high) ? high : value)
#define MAXIMUM(value, limit)value<limit ? limit : value
#define MINIMUM(value, limit)value>limit ? limit : value

#define F32_MAX 3.402823466e+38
#define F32_MIN 1.175494351e-38
#define F32_EPS 1e-6

#define APPROXIMATIVE_PI 3.14
}
#endif /* _COMMON_H_ */
