#ifndef TEMPERATURE_CONTROLLER_H
#define TEMPERATURE_CONTROLLER_H
/* Iteration options:
 *  1. 2 seperate wave-forms (1 sens, 1 heat), 4 different idxs
 *      - Minimises memory use
 *      - More cycles incremeting 4 pointers
 *  2. 4 wave-forms, 2 idxs
 *      - more memory use
 *      - less time incrementing pointers
 *      - still need to apply the index
 *  3. 4 wave-forms, 4 pointers
 *      - Increment the pointer itself
 *      - Maybe saves time on the look-up part
 *      - However still need to check for being out-of-range +
 *          looping back
 *      - Probably doesn't save any time over all at present?
 *      - May be slightly more efficient if each wfm has a different
 *          size
*/

//GLOBAL CONSTANTS
const int Nbase=64;
const int NbaseMask=63;
const int N_HEATER_WV=0x004;//Nbase;
const int HEATER_WV_MASK = 0x003;
const int N_SENS_WV=0x010;
const int SENS_WV_MASK = 0x00F;
const int N_REF_WV=128;//N_HEATER_WV*8;
const int NCYC_SENS=2;
const int NCYC_HEAT=8;
const float PI_ON_2 = PI/2.0;
const float TWOPI= 2*PI;
const float N_REF_WV_ON_2PI = N_REF_WV/(2*PI);
const int IMAX=2147483647;
//
//Functions:
void write_wave_form(int wv_fm[], unsigned int Npts, unsigned int Ncyc, int amplitude, float phase);
void ADC_Handler();
#endif //TEMPERATURE_CONTROLLER_H
