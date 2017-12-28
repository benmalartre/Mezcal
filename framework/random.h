//--------------------------------------------------------
// Random
//--------------------------------------------------------
#ifndef _RANDOM_H_
#define _RANDOM_H_

#include <math.h>
#include <stdlib.h>

namespace BOB{

    float random_0_1(){return ((float) rand() / (RAND_MAX));};
    float random_1_1(){return (1.0f-2.f*((float) rand() / (RAND_MAX)));};
    float random_range(float _min, float _max){
        return _min+((float) rand() / (RAND_MAX))*(_max-_min);
    }

}// end namespace BOB
#endif /* _GRID_H_ */
