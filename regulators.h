/*
 * regulators.h
 *
 *  Created on: 18. 3. 2026
 *      Author: Stastny Martin
 */

#ifndef SRC_REGULATORS_H_
#define SRC_REGULATORS_H_

typedef struct{
    float err;
    float state;
    float Kp;
    float Ki;
    float Kwind;
    float Max;
    float Min;
    float int_sum;
    uint16_t stop_int;
    float e_p;
    float e_i;
    float p_out;
    float i_out;
} RegPI_t;

void PIstop(RegPI_t* REG);
void RegI(RegPI_t* REG);
void I_reg(float w, float y, float *e, float *u, float *sum, float Ki, float max, float min);

void RegInit(RegPI_t* REG,float Kp, float Ki, float Min, float Max);


#endif /* SRC_REGULATORS_H_ */
