/*
 * regulators.c
 *
 *  Created on: 18. 3. 2026
 *      Author: Stastny Martin
 */
#include "src_hw/rumm_hwlib.h"                  // MUST BE INCLUDED IN EVERY SOURCE FILE ON THE FIRST ROW
#include "regulators.h"
#include "global_constants.h"

void PIstop(RegPI_t* reg)
{

    if (!reg->stop_int)
        reg->int_sum = reg->int_sum + reg->err*reg->Ki*T_SAMPLE;

    reg->state = reg->int_sum + reg->err*reg->Kp;

    if (reg->state > reg->Max)
    {
        reg->state = reg->Max;
        reg->stop_int = 1;
    }
    else if(reg->state < reg->Min)
    {
        reg->state = reg->Min;
        reg->stop_int = 1;
    }
    else
        reg->stop_int = 0;
}

void RegI(RegPI_t* reg)
{
    if (!reg->stop_int)
        reg->int_sum = reg->int_sum + reg->err*reg->Ki*T_SAMPLE;

    reg->state = reg->int_sum;

    if (reg->state > reg->Max)
    {
        reg->state = reg->Max;
        reg->stop_int = 1;
    }
    else if(reg->state < reg->Min)
    {
        reg->state = reg->Min;
        reg->stop_int = 1;
    }
    else
        reg->stop_int = 0;
}

void I_reg(float w, float y, float *e, float *u, float *sum, float Ki, float max, float min) {
    float d_sum;

    *e = w - y;
    d_sum = *e * Ki;
    *u = *sum;

    if (*u > max) {
        *u = max;
        if (d_sum > 0)
            d_sum = 0;
    }
    else if (*u < min) {
        *u = min;
        if (d_sum < 0)
            d_sum = 0;
    }

    *sum = *sum + d_sum * T_SAMPLE;
    *u = *sum;
}

void RegInit(RegPI_t* REG,float Kp, float Ki, float Min, float Max)
{
    REG->Ki = Ki;
    REG->Kp = Kp;
    REG->Max = Max;
    REG->Min = Min;

    REG->err = 0.0f;
    REG->Kwind = 0.0f;
    REG->state = 0.0f;
    REG->int_sum = 0.0f;

    REG->stop_int = 0;

    REG->e_p = 0.0f;
    REG->e_i = 0.0f;
    REG->p_out = 0.0f;
    REG->i_out = 0.0f;
}
