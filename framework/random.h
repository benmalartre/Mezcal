//--------------------------------------------------------
// Random
//--------------------------------------------------------
#ifndef _RANDOM_H_
#define _RANDOM_H_

#include <math.h>
#include <stdlib.h>

namespace BOB{
    //----------------------------------------------------
    // std
    //----------------------------------------------------
    float random_0_1();
    float random_1_1();
    float random_range(float _min, float _max);
    
    //-----------------------------------------------------
    // iq
    //-----------------------------------------------------
    unsigned int rand_int15b( int *mirand );
    float        rand_flo23b( int *mirand );
    float        rand_sflo23b( int *mirand );
    
    //-----------------------------------------------------
    // numerical recipes
    //-----------------------------------------------------
    // period = 2*10^18
    void nrcp_rand2_init( long *idnum );
    long nrcp_rand2_int31( long *idum );
    
    
    //-----------------------------------------------------
    // MT19937, by Takuji Nishimura and Makoto Matsumoto
    //-----------------------------------------------------
    // period = 2^19937
    void          mt19937_init(unsigned long s);
    void          mt19937_init_by_array(unsigned long init_key[], int key_length);
    unsigned long mt19937_int32(void);
    double        mt19937_dou53(void); // [0,1) with 53-bit resolution

}// end namespace BOB
#endif /* _GRID_H_ */
