/*
 * droop_ctrl.c
 *
 *  Created on: 16. 3. 2026
 *      Author:
 */
#include "src_hw/rumm_hwlib.h"                  // MUST BE INCLUDED IN EVERY SOURCE FILE ON THE FIRST ROW
#include "droop_ctrl.h"
#include "global_constants.h"
#include "measure.h"
#include <src_hw/pwm_hw.h>
#include "Transformations.h"
//#include <src_usr/app.h>

#include "power_restriction.h"
#include "src/islanding.h"
#include "src/protection.h"

CONTROL_t CONTROL_obj;
CONTROL_t *CONTROL = &CONTROL_obj;

uint16_t droopInit()
{
    /* Init full CONTROL_t Structure */
    CONTROL->MEAS.U_a = 0;
    CONTROL->MEAS.U_b = 0;
    CONTROL->MEAS.U_c = 0;

    CONTROL->I_d_conv = 0;
    CONTROL->I_q_conv = 0;

    CONTROL->I_d_grid = 0;
    CONTROL->I_q_grid = 0;

    CONTROL->I_d_w = 0;
    CONTROL->I_q_w = 0;

    CONTROL->P_w = 0.0f;
    CONTROL->Q_w = 0.0f;

    CONTROL->P_w_res = 0.0f;
    CONTROL->Q_w_res = 0.0f;

    CONTROL->U_a_reg = 0.0f;
    CONTROL->U_b_reg = 0.0f;
    CONTROL->U_c_reg = 0.0f;

    CONTROL->U_d = 0.0f;
    CONTROL->U_q = 0.0f;

    CONTROL->U_d_FF = 0.0f;
    CONTROL->U_q_FF = 0.0f;

    CONTROL->U_d_grid = 0.0f;
    CONTROL->U_q_grid = 0.0f;

    CONTROL->U_d_w = 0.0f;
    CONTROL->U_q_w = 0.0f;

    CONTROL->U_ramp = 0.0f;
    CONTROL->ramp_done = 0;

    CONTROL->f_w = 0.0f;
    CONTROL->theta = 0.0f;

    CONTROL->state = 0; // Default state 0 == Standby

    /* Reset DSOGI structure??? */
    CONTROL->DSOGI.U_amp_act = 0.0f;
    CONTROL->DSOGI.U_d_grid = 0.0f;
    CONTROL->DSOGI.U_q_grid = 0.0f;
    CONTROL->DSOGI.Uf_max = 0.0f;
    CONTROL->DSOGI.cos_th = 0.0f;
    CONTROL->DSOGI.sin_th = 0.0f;
    CONTROL->DSOGI.theta_Uf = 0.0f;
    CONTROL->DSOGI.theta_reg_cos_120 = 0.0f;
    CONTROL->DSOGI.theta_reg_cos_240 = 0.0f;
    CONTROL->DSOGI.w_n = 0.0f;


    /* Init regulators */
    RegInit(&CONTROL->Reg_voltage_d, Kp_PI_V, Ki_PI_V, -Imax, Imax);
    RegInit(&CONTROL->Reg_voltage_q, Kp_PI_V, Ki_PI_V, -Imax, Imax);
    RegInit(&CONTROL->Reg_current_d, Kp_PI_I, Ki_PI_I, -U_dc/2, U_dc/2);
    RegInit(&CONTROL->Reg_current_q, Kp_PI_I, Ki_PI_I, -U_dc/2, U_dc/2);

    /* Map variables to int/ext ADC */
    CONTROL->MEAS.U_ab = &adc_ex_x8->U_GRID_AB;
    CONTROL->MEAS.U_bc = &adc_ex_x8->U_GRID_BC;
    CONTROL->MEAS.U_ca = &adc_ex_x8->U_GRID_CA;

    CONTROL->MEAS.U_DC = &adc_ex_x8->U_DC;

    CONTROL->MEAS.I_a_conv = &adc_in_x17->I_CONV_A;
    CONTROL->MEAS.I_b_conv = &adc_in_x17->I_CONV_B;
    CONTROL->MEAS.I_c_conv = &adc_in_x17->I_CONV_C;

    CONTROL->MEAS.I_a_grid = &adc_in_x17->I_GRID_A;
    CONTROL->MEAS.I_b_grid = &adc_in_x17->I_GRID_B;
    CONTROL->MEAS.I_c_grid = &adc_in_x17->I_GRID_C;

    /* Init DEVICE_STATE Structure */
    CONTROL->DSTATE.ASM_relay = 0;
    CONTROL->DSTATE.P_f_curve = 0;
    CONTROL->DSTATE.Q_U_PI = 0;
    CONTROL->DSTATE.Q_U_curve = 0;
    CONTROL->DSTATE.activate = 0;
    CONTROL->DSTATE.button = 0;
    CONTROL->DSTATE.connected = 0;
    CONTROL->DSTATE.grid_voltage = 0;
    CONTROL->DSTATE.locked = 0;
    CONTROL->DSTATE.locking = 0;
    CONTROL->DSTATE.master = 0;
    CONTROL->DSTATE.ramp_done = 0;
    CONTROL->DSTATE.state = 0;
    CONTROL->DSTATE.stnd_op = 0;
    CONTROL->DSTATE.reg_on = 0;
    CONTROL->DSTATE.res_reg_on = 0;
    CONTROL->DSTATE.prot_on = 0;

    // Set DSTATE->MASTER bit to 1 --> for island operation
    //CONTROL->DSTATE.master = 1;

    /* Resonant regulators init */
    CONTROL->I_alpha = 0.0f;
    CONTROL->I_beta = 0.0f;

    CONTROL->act_3rd = 0;
    CONTROL->act_5th = 0;
    CONTROL->act_7th = 0;
    CONTROL->act_9th = 0;
    CONTROL->act_11th = 0;
    CONTROL->act_13th = 0;

    Rez_reg_init(&CONTROL->Rez_150_a,150,KR_3rd);
    Rez_reg_init(&CONTROL->Rez_150_b,150,KR_3rd);
    Rez_reg_init(&CONTROL->Rez_150_c,150,KR_3rd);
    Rez_reg_init(&CONTROL->Rez_250_a,250,KR_5th);
    Rez_reg_init(&CONTROL->Rez_250_b,250,KR_5th);
    Rez_reg_init(&CONTROL->Rez_250_c,250,KR_5th);
    Rez_reg_init(&CONTROL->Rez_350_a,350,KR_7th);
    Rez_reg_init(&CONTROL->Rez_350_b,350,KR_7th);
    Rez_reg_init(&CONTROL->Rez_450_a,450,KR_9th);
    Rez_reg_init(&CONTROL->Rez_450_b,450,KR_9th);
    Rez_reg_init(&CONTROL->Rez_550_a,550,KR_11th);
    Rez_reg_init(&CONTROL->Rez_550_b,550,KR_11th);
    Rez_reg_init(&CONTROL->Rez_650_a,650,KR_13th);
    Rez_reg_init(&CONTROL->Rez_650_b,650,KR_13th);


    return 0;
}


/* Ukradeno */
#define Kp_DQ_curr 0.5
#define Ki_DQ_curr 30

#define Kp_osc  0.0

#define Ramp_w 1*T_SAMPLE

#define I_reg_max 10

#define Ki_stab_d 0.5
#define Kp_stab_d 0.1
#define Kp_Pf_d 1




void DROOP_init(DROOP_DATA *DROOP,DEVICE_STATE *STATE)
{
//"DROOP" control init

    DROOP->var.Ki_stab=Ki_stab_d;
    DROOP->var.Kp_Pf=Kp_Pf_d;
    DROOP->var.Kp_stab=Kp_stab_d;

    /*
//Resonant controler init
    DROOP->var.act_3rd=0;
    DROOP->var.act_5th=0;
    DROOP->var.act_7th=0;
    DROOP->var.act_9th=0;
    DROOP->var.act_11th=0;
    DROOP->var.act_13th=0;

    Rez_reg_init(&DROOP->var.Rez_150_a,150,KR_3rd);
    Rez_reg_init(&DROOP->var.Rez_150_b,150,KR_3rd);
    Rez_reg_init(&DROOP->var.Rez_150_c,150,KR_3rd);
    Rez_reg_init(&DROOP->var.Rez_250_a,250,KR_5th);
    Rez_reg_init(&DROOP->var.Rez_250_b,250,KR_5th);
    Rez_reg_init(&DROOP->var.Rez_250_c,250,KR_5th);
    Rez_reg_init(&DROOP->var.Rez_350_a,350,KR_7th);
    Rez_reg_init(&DROOP->var.Rez_350_b,350,KR_7th);
    Rez_reg_init(&DROOP->var.Rez_450_a,450,KR_9th);
    Rez_reg_init(&DROOP->var.Rez_450_b,450,KR_9th);
    Rez_reg_init(&DROOP->var.Rez_550_a,550,KR_11th);
    Rez_reg_init(&DROOP->var.Rez_550_b,550,KR_11th);
    Rez_reg_init(&DROOP->var.Rez_650_a,650,KR_13th);
    Rez_reg_init(&DROOP->var.Rez_650_b,650,KR_13th);
    */
}

void DROOP_exe(CONTROL_t *CONTROL_STR)
{
    //float e_pd, e_id, e_pq, e_iq;
    float I_ref_d, I_ref_q;

    if(CONTROL_STR->state == 0) // Standby
    {
        CONTROL_STR->U_a_reg = 0;
        CONTROL_STR->U_b_reg = 0;
        CONTROL_STR->U_c_reg = 0;
        CONTROL_STR->U_ramp = 0;
    }
    else if(CONTROL_STR->state == 1)   // Ramp
    {
        // Pri Grid forming je CONTROL_STR->DSTATE.master == 1 a DSOGI pak dava natvrdo g_Um=325 g_w=2*pi*50
        // Jinak se normalne zachyti a dava aktualni g_Um, g_w a g_theta podle PLL
        // Kdyz budu chtit startovat do ostrovu, tak musim napred nastavit CONTROL_STR->DSTATE.master == 1

        if (CONTROL_STR->U_ramp < 1)
            CONTROL_STR->U_ramp += 0.2*T_SAMPLE;
        else
        {
            CONTROL_STR->U_ramp = 1;
            CONTROL_STR->ramp_done = 1;

        }
        // CONTROL->DSOGI.Uf_max == amplituda DSOGI
        CONTROL_STR->U_a_reg = CONTROL_STR->U_ramp*CONTROL_STR->DSOGI.Uf_max*CONTROL_STR->DSOGI.cos_th;
        CONTROL_STR->U_b_reg = CONTROL_STR->U_ramp*CONTROL_STR->DSOGI.Uf_max*CONTROL_STR->DSOGI.theta_reg_cos_120;
        CONTROL_STR->U_c_reg = CONTROL_STR->U_ramp*CONTROL_STR->DSOGI.Uf_max*CONTROL_STR->DSOGI.theta_reg_cos_240;

    }
    else if(CONTROL_STR->state == 2) // Grid Forming
    {
        /*
        // Namapovani do struktury
        CONTROL_STR->DSOGI.cos_th = cosf(CONTROL_STR->theta);
        CONTROL_STR->DSOGI.sin_th = sinf(CONTROL_STR->theta);
        CONTROL_STR->DSOGI.theta_Uf = CONTROL_STR->theta;*/

        /* Transformations */
        abc_to_dq(*CONTROL_STR->MEAS.I_a_conv,*CONTROL_STR->MEAS.I_b_conv,*CONTROL_STR->MEAS.I_c_conv,CONTROL_STR->DSOGI.cos_th,CONTROL_STR->DSOGI.sin_th,&CONTROL_STR->I_d_conv,&CONTROL_STR->I_q_conv);
        abc_to_dq(*CONTROL_STR->MEAS.I_a_grid,*CONTROL_STR->MEAS.I_b_grid,*CONTROL_STR->MEAS.I_c_grid,CONTROL_STR->DSOGI.cos_th,CONTROL_STR->DSOGI.sin_th,&CONTROL_STR->I_d_grid,&CONTROL_STR->I_q_grid);
        // prevod sdruzene na fazove
        CONTROL_STR->MEAS.U_a = (*CONTROL_STR->MEAS.U_ab-*CONTROL_STR->MEAS.U_ca)*JednaD3;
        CONTROL_STR->MEAS.U_b = (*CONTROL_STR->MEAS.U_bc-*CONTROL_STR->MEAS.U_ab)*JednaD3;
        CONTROL_STR->MEAS.U_c = (*CONTROL_STR->MEAS.U_ca-*CONTROL_STR->MEAS.U_bc)*JednaD3;
        abc_to_dq(CONTROL_STR->MEAS.U_a,CONTROL_STR->MEAS.U_b,CONTROL_STR->MEAS.U_c,CONTROL_STR->DSOGI.cos_th,CONTROL_STR->DSOGI.sin_th,&CONTROL_STR->U_d_grid, &CONTROL_STR->U_q_grid);


        /* Voltage regulator */
        CONTROL_STR->Reg_voltage_d.err = CONTROL_STR->U_d_w - CONTROL_STR->U_d_grid;
        CONTROL_STR->Reg_voltage_q.err = CONTROL_STR->U_q_w - CONTROL_STR->U_q_grid;

        PIstop(&CONTROL_STR->Reg_voltage_d);
        PIstop(&CONTROL_STR->Reg_voltage_q);

        /* Current regulator */
        /* Vypocet odchylek */
        CONTROL_STR->Reg_current_d.e_p = CONTROL_STR->Reg_voltage_d.state - CONTROL_STR->I_d_conv;
        CONTROL_STR->Reg_current_q.e_p = CONTROL_STR->Reg_voltage_q.state - CONTROL_STR->I_q_conv;

        CONTROL_STR->Reg_current_d.e_i = CONTROL_STR->Reg_voltage_d.state - CONTROL_STR->I_d_grid;
        CONTROL_STR->Reg_current_q.e_i = CONTROL_STR->Reg_voltage_q.state - CONTROL_STR->I_q_grid;
        /* Vypocet P a I slozky */
        CONTROL_STR->Reg_current_d.p_out = CONTROL_STR->Reg_current_d.Kp * CONTROL_STR->Reg_current_d.e_p;
        float d_i_step = CONTROL_STR->Reg_current_d.e_i * CONTROL_STR->Reg_current_d.Ki * T_SAMPLE;
        CONTROL_STR->Reg_current_q.p_out = CONTROL_STR->Reg_current_q.Kp * CONTROL_STR->Reg_current_q.e_p;
        float q_i_step = CONTROL_STR->Reg_current_q.e_i * CONTROL_STR->Reg_current_q.Ki * T_SAMPLE;
        /* Vystup bez saturace */
        float u_d_unsat = CONTROL_STR->Reg_current_d.p_out + (CONTROL_STR->Reg_current_d.int_sum + d_i_step);
        float u_q_unsat = CONTROL_STR->Reg_current_q.p_out + (CONTROL_STR->Reg_current_q.int_sum + q_i_step);
        /* Zasaturovani */
        if (u_d_unsat > CONTROL_STR->Reg_current_d.Max)
        {
            CONTROL_STR->U_d = CONTROL_STR->Reg_current_d.Max;
        }
        else if (u_d_unsat < CONTROL_STR->Reg_current_d.Min)
        {
            CONTROL_STR->U_d = CONTROL_STR->Reg_current_d.Min;
        }
        else
        {
            CONTROL_STR->U_d = u_d_unsat;
        }

        if (u_q_unsat > CONTROL_STR->Reg_current_q.Max)
        {
            CONTROL_STR->U_q = CONTROL_STR->Reg_current_q.Max;
        }
        else if (u_q_unsat < CONTROL_STR->Reg_current_q.Min)
        {
            CONTROL_STR->U_q = CONTROL_STR->Reg_current_q.Min;
        }
        else
        {
            CONTROL_STR->U_q = u_q_unsat;
        }

        bool sat_d_high = (u_d_unsat > CONTROL_STR->Reg_current_d.Max);
        bool sat_d_low  = (u_d_unsat < CONTROL_STR->Reg_current_d.Min);

        bool sat_q_high = (u_q_unsat > CONTROL_STR->Reg_current_q.Max);
        bool sat_q_low  = (u_q_unsat < CONTROL_STR->Reg_current_q.Min);

        if ( !(sat_d_high && CONTROL_STR->Reg_current_d.e_i > 0) &&
                !(sat_d_low  && CONTROL_STR->Reg_current_d.e_i < 0) )
        {
            CONTROL_STR->Reg_current_d.int_sum += d_i_step;
        }

        if ( !(sat_q_high && CONTROL_STR->Reg_current_q.e_i > 0) &&
                !(sat_q_low  && CONTROL_STR->Reg_current_q.e_i < 0) )
        {
            CONTROL_STR->Reg_current_q.int_sum += q_i_step;
        }



        /* Feed-Forward */
        // ToDo add decoupling???
        //CONTROL_STR->U_d_FF = CONTROL_STR->U_d_w;
        //CONTROL_STR->U_q_FF = CONTROL_STR->U_q_w;
        CONTROL_STR->U_d_FF = CONTROL_STR->U_d_w ;//- (X_L_conv + X_L_grid)*CONTROL_STR->I_q_grid;
        CONTROL_STR->U_q_FF = CONTROL_STR->U_q_w ;//+ (X_L_conv + X_L_grid)*CONTROL_STR->I_d_grid;
        //CONTROL_STR->U_d_FF = CONTROL_STR->U_d_w - (X_L_conv + X_L_grid)*CONTROL_STR->I_q_grid + (R_conv + R_grid)*CONTROL_STR->I_d_grid;
        //CONTROL_STR->U_q_FF = CONTROL_STR->U_q_w + (X_L_conv + X_L_grid)*CONTROL_STR->I_d_grid + (R_conv + R_grid)*CONTROL_STR->I_q_grid;

        CONTROL_STR->U_d = 0;
        CONTROL_STR->U_q = 0;
        // Transformace do abc
        dq_to_abc(CONTROL_STR->U_d+CONTROL_STR->U_d_FF,CONTROL_STR->U_q+CONTROL_STR->U_q_FF,CONTROL_STR->DSOGI.theta_Uf,
                  &CONTROL_STR->U_a_reg,&CONTROL_STR->U_b_reg,&CONTROL_STR->U_c_reg);

        //DROOP->var.Ua=DROOP->dsogi.Uf_max*DROOP->dsogi.cos_th+DROOP->var.U_reg_a+DROOP->var.U_res_a;
        //DROOP->var.Ub=DROOP->dsogi.Uf_max*DROOP->dsogi.theta_reg_cos_120+DROOP->var.U_reg_b+DROOP->var.U_res_b;
        //DROOP->var.Uc=DROOP->dsogi.Uf_max*DROOP->dsogi.theta_reg_cos_240+DROOP->var.U_reg_c+DROOP->var.U_res_c;
    }
    else if (CONTROL_STR->state == 3)   // Grid Following
    {
        /* Transformations */
        abc_to_dq(*CONTROL_STR->MEAS.I_a_conv,*CONTROL_STR->MEAS.I_b_conv,*CONTROL_STR->MEAS.I_c_conv,CONTROL_STR->DSOGI.cos_th,CONTROL_STR->DSOGI.sin_th,&CONTROL_STR->I_d_conv,&CONTROL_STR->I_q_conv);
        abc_to_dq(*CONTROL_STR->MEAS.I_a_grid,*CONTROL_STR->MEAS.I_b_grid,*CONTROL_STR->MEAS.I_c_grid,CONTROL_STR->DSOGI.cos_th,CONTROL_STR->DSOGI.sin_th,&CONTROL_STR->I_d_grid,&CONTROL_STR->I_q_grid);
        //abc_to_dq(CONTROL_STR->MEAS.U_a,CONTROL_STR->MEAS.U_b,CONTROL_STR->MEAS.U_c,CONTROL_STR->DSOGI.cos_th,CONTROL_STR->DSOGI.sin_th,&CONTROL_STR->U_d_grid, &CONTROL_STR->U_q_grid);

        CONTROL_STR->U_d_grid = CONTROL_STR->DSOGI.U_d_grid;
        CONTROL_STR->U_q_grid = CONTROL_STR->DSOGI.U_q_grid;
        /* Pozadovany P a Q zmenit v glob struct CONTROL->P_W */
        CONTROL_STR->P_w_res = activePower(CONTROL_STR->DSOGI.w_n, CONTROL_STR->DSOGI.U_amp_act, CONTROL_STR->P_w);
        //CONTROL_STR->Q_w_res = reactivePower(CONTROL_STR->DSOGI.U_amp_act, CONTROL_STR->Q_w);
        CONTROL_STR->Q_w_res = CONTROL_STR->Q_w;

        /* Prepocet vykonu na proudy Id,Iq */
        //ToDo je prepocet spravne???
        I_ref_d = (CONTROL_STR->P_w_res/CONTROL_STR->DSOGI.U_amp_act)*DvaD3;
        I_ref_q = (CONTROL_STR->Q_w_res/CONTROL_STR->DSOGI.U_amp_act)*DvaD3*(-1.0f);

        if (CONTROL_STR->DSTATE.reg_on)
        {
            /* Proudova rampa */
            if (CONTROL_STR->I_d_w < I_ref_d)
            {
                CONTROL_STR->I_d_w += Ramp_w;
                if (CONTROL_STR->I_d_w > I_ref_d)
                    CONTROL_STR->I_d_w = I_ref_d;
            }
            else if (CONTROL_STR->I_d_w > I_ref_d)
            {
                CONTROL_STR->I_d_w -= Ramp_w;
                if (CONTROL_STR->I_d_w < I_ref_d)
                    CONTROL_STR->I_d_w = I_ref_d;
            }

            if (CONTROL_STR->I_q_w < I_ref_q)
            {
                CONTROL_STR->I_q_w += Ramp_w;
                if (CONTROL_STR->I_q_w > I_ref_q)
                    CONTROL_STR->I_q_w = I_ref_q;
            }
            else if (CONTROL_STR->I_q_w > I_ref_q)
            {
                CONTROL_STR->I_q_w -= Ramp_w;
                if (CONTROL_STR->I_q_w < I_ref_q)
                    CONTROL_STR->I_q_w = I_ref_q;
            }

            /* PI LCL Regulator */

            /* Vypocet odchylek */
            CONTROL_STR->Reg_current_d.e_p = CONTROL_STR->I_d_w - CONTROL_STR->I_d_conv;
            CONTROL_STR->Reg_current_q.e_p = CONTROL_STR->I_q_w - CONTROL_STR->I_q_conv;

            CONTROL_STR->Reg_current_d.e_i = CONTROL_STR->I_d_w - CONTROL_STR->I_d_grid;
            CONTROL_STR->Reg_current_q.e_i = CONTROL_STR->I_q_w - CONTROL_STR->I_q_grid;
            /* Vypocet P a I slozky */
            CONTROL_STR->Reg_current_d.p_out = CONTROL_STR->Reg_current_d.Kp * CONTROL_STR->Reg_current_d.e_p;
            float d_i_step = CONTROL_STR->Reg_current_d.e_i * CONTROL_STR->Reg_current_d.Ki * T_SAMPLE;
            CONTROL_STR->Reg_current_q.p_out = CONTROL_STR->Reg_current_q.Kp * CONTROL_STR->Reg_current_q.e_p;
            float q_i_step = CONTROL_STR->Reg_current_q.e_i * CONTROL_STR->Reg_current_q.Ki * T_SAMPLE;
            /* Vystup bez saturace */
            float u_d_unsat = CONTROL_STR->Reg_current_d.p_out + (CONTROL_STR->Reg_current_d.int_sum + d_i_step);
            float u_q_unsat = CONTROL_STR->Reg_current_q.p_out + (CONTROL_STR->Reg_current_q.int_sum + q_i_step);
            /* Zasaturovani */
            if (u_d_unsat > CONTROL_STR->Reg_current_d.Max)
            {
                CONTROL_STR->U_d = CONTROL_STR->Reg_current_d.Max;
            }
            else if (u_d_unsat < CONTROL_STR->Reg_current_d.Min)
            {
                CONTROL_STR->U_d = CONTROL_STR->Reg_current_d.Min;
            }
            else
            {
                CONTROL_STR->U_d = u_d_unsat;
            }

            if (u_q_unsat > CONTROL_STR->Reg_current_q.Max)
            {
                CONTROL_STR->U_q = CONTROL_STR->Reg_current_q.Max;
            }
            else if (u_q_unsat < CONTROL_STR->Reg_current_q.Min)
            {
                CONTROL_STR->U_q = CONTROL_STR->Reg_current_q.Min;
            }
            else
            {
                CONTROL_STR->U_q = u_q_unsat;
            }

            bool sat_d_high = (u_d_unsat > CONTROL_STR->Reg_current_d.Max);
            bool sat_d_low  = (u_d_unsat < CONTROL_STR->Reg_current_d.Min);

            bool sat_q_high = (u_q_unsat > CONTROL_STR->Reg_current_q.Max);
            bool sat_q_low  = (u_q_unsat < CONTROL_STR->Reg_current_q.Min);

            if ( !(sat_d_high && CONTROL_STR->Reg_current_d.e_i > 0) &&
                    !(sat_d_low  && CONTROL_STR->Reg_current_d.e_i < 0) )
            {
                CONTROL_STR->Reg_current_d.int_sum += d_i_step;
            }

            if ( !(sat_q_high && CONTROL_STR->Reg_current_q.e_i > 0) &&
                    !(sat_q_low  && CONTROL_STR->Reg_current_q.e_i < 0) )
            {
                CONTROL_STR->Reg_current_q.int_sum += q_i_step;
            }
        }
        else
        {
            /* Reset regulators output */
            CONTROL_STR->U_d = 0.0f;
            CONTROL_STR->U_q = 0.0f;
            /* Reset regulators */
            CONTROL_STR->Reg_current_d.int_sum = 0.0f;
            CONTROL_STR->Reg_current_q.int_sum = 0.0f;
        }
        /* Resonant control */
        if (CONTROL_STR->DSTATE.res_reg_on)
        {
            abc_2_alpha_beta(*CONTROL_STR->MEAS.I_a_conv, *CONTROL_STR->MEAS.I_b_conv,
                             *CONTROL_STR->MEAS.I_c_conv, &CONTROL_STR->I_alpha, &CONTROL_STR->I_beta);

            if(CONTROL_STR->act_3rd==1)
            {
                CONTROL_STR->Rez_150_a.err=-*CONTROL_STR->MEAS.I_a_conv;
                CONTROL_STR->Rez_150_b.err=-*CONTROL_STR->MEAS.I_b_conv;
                CONTROL_STR->Rez_150_c.err=-*CONTROL_STR->MEAS.I_c_conv;
                //CONTROL_STR->Rez_150_a.err = -CONTROL_STR->I_alpha;
                //CONTROL_STR->Rez_150_b.err = -CONTROL_STR->I_beta;
                Rez_simplest(&CONTROL_STR->Rez_150_a);
                Rez_simplest(&CONTROL_STR->Rez_150_b);
                Rez_simplest(&CONTROL_STR->Rez_150_c);
            }
            else
            {
                Resonant_ctrl_reset(CONTROL_STR,0,150);
            }

            if(CONTROL_STR->act_5th==1)
            {
                CONTROL_STR->Rez_250_a.err=-*CONTROL_STR->MEAS.I_a_grid;
                CONTROL_STR->Rez_250_b.err=-*CONTROL_STR->MEAS.I_b_grid;
                CONTROL_STR->Rez_250_c.err=-*CONTROL_STR->MEAS.I_c_grid;
                //CONTROL_STR->Rez_250_a.err = -CONTROL_STR->I_alpha;
                //CONTROL_STR->Rez_250_b.err = -CONTROL_STR->I_beta;
                Rez_simplest(&CONTROL_STR->Rez_250_a);
                Rez_simplest(&CONTROL_STR->Rez_250_b);
                Rez_simplest(&CONTROL_STR->Rez_250_c);
            }
            else
            {
                Resonant_ctrl_reset(CONTROL_STR,0,250);
            }

            if(CONTROL_STR->act_7th==1)
            {
                //CONTROL_STR->Rez_350_a.err=-CONTROL_STR->I_alpha;
                //CONTROL_STR->Rez_350_b.err=-CONTROL_STR->I_beta;
                //Rez_simplest(&CONTROL_STR->Rez_350_a);
                //Rez_simplest(&CONTROL_STR->Rez_350_b);
                CONTROL_STR->Rez_350_a.err = -*CONTROL_STR->MEAS.I_a_grid;
                CONTROL_STR->Rez_350_b.err = -*CONTROL_STR->MEAS.I_b_grid;
                CONTROL_STR->Rez_350_c.err = -*CONTROL_STR->MEAS.I_c_grid;
                Rez_simplest(&CONTROL_STR->Rez_350_a);
                Rez_simplest(&CONTROL_STR->Rez_350_b);
                Rez_simplest(&CONTROL_STR->Rez_350_c);
            }
            else
            {
                Resonant_ctrl_reset(CONTROL_STR,0,350);
            }
            if(CONTROL_STR->act_9th==1)
            {
                /*CONTROL_STR->Rez_450_a.err=-CONTROL_STR->I_alpha;
                CONTROL_STR->Rez_450_b.err=-CONTROL_STR->I_beta;
                Rez_simplest(&CONTROL_STR->Rez_450_a);
                Rez_simplest(&CONTROL_STR->Rez_450_b);*/
                CONTROL_STR->Rez_450_a.err = -*CONTROL_STR->MEAS.I_a_conv;
                CONTROL_STR->Rez_450_b.err = -*CONTROL_STR->MEAS.I_b_conv;
                CONTROL_STR->Rez_450_c.err = -*CONTROL_STR->MEAS.I_c_conv;
                Rez_simplest(&CONTROL_STR->Rez_450_a);
                Rez_simplest(&CONTROL_STR->Rez_450_b);
                Rez_simplest(&CONTROL_STR->Rez_450_c);
            }
            else
            {
                Resonant_ctrl_reset(CONTROL_STR,0,450);
            }

            //CONTROL_STR->U_res_alpha =  CONTROL_STR->Rez_150_a.vyst+CONTROL_STR->Rez_250_a.vyst+CONTROL_STR->Rez_350_a.vyst+CONTROL_STR->Rez_450_a.vyst+CONTROL_STR->Rez_550_a.vyst+CONTROL_STR->Rez_650_a.vyst;
            //CONTROL_STR->U_res_beta = CONTROL_STR->Rez_150_b.vyst+CONTROL_STR->Rez_250_b.vyst+CONTROL_STR->Rez_350_b.vyst+CONTROL_STR->Rez_450_b.vyst+CONTROL_STR->Rez_550_b.vyst+CONTROL_STR->Rez_650_b.vyst;

            //alpha_beta_2_abc(CONTROL_STR->U_res_alpha, CONTROL_STR->U_res_beta, &CONTROL_STR->U_res_a, &CONTROL_STR->U_res_b, &CONTROL_STR->U_res_c);

            CONTROL_STR->U_res_a=CONTROL_STR->Rez_150_a.vyst + CONTROL_STR->Rez_250_a.vyst
                    + CONTROL_STR->Rez_350_a.vyst + CONTROL_STR->Rez_450_a.vyst;
            CONTROL_STR->U_res_b=CONTROL_STR->Rez_150_b.vyst + CONTROL_STR->Rez_250_b.vyst
                                + CONTROL_STR->Rez_350_b.vyst + CONTROL_STR->Rez_450_b.vyst;
            CONTROL_STR->U_res_c=CONTROL_STR->Rez_150_c.vyst + CONTROL_STR->Rez_250_c.vyst
                                + CONTROL_STR->Rez_350_c.vyst + CONTROL_STR->Rez_450_c.vyst;
        }
        else
        {
            CONTROL_STR->U_res_a = 0.0f;
            CONTROL_STR->U_res_b = 0.0f;
            CONTROL_STR->U_res_c = 0.0f;
        }

        /* Feed-Forward */
        CONTROL_STR->U_d_FF = CONTROL_STR->U_d_grid - (X_L_conv + X_L_grid)*CONTROL_STR->I_q_grid;
        CONTROL_STR->U_q_FF = CONTROL_STR->U_q_grid + (X_L_conv + X_L_grid)*CONTROL_STR->I_d_grid;
        //ToDo pouzit FF co je dole???
        //CONTROL_STR->U_d_FF = CONTROL_STR->U_d_grid - (X_L_conv + X_L_grid)*CONTROL_STR->I_q_grid + (R_conv + R_grid)*CONTROL_STR->I_d_grid;
        //CONTROL_STR->U_q_FF = CONTROL_STR->U_q_grid + (X_L_conv + X_L_grid)*CONTROL_STR->I_d_grid + (R_conv + R_grid)*CONTROL_STR->I_q_grid;
        /* DQ to abc, normovat netreba (to je v modulatoru) */
        dq_to_abc(CONTROL_STR->U_d + CONTROL_STR->U_d_FF, CONTROL_STR->U_q + CONTROL_STR->U_q_FF, CONTROL_STR->DSOGI.theta_Uf,
                  &CONTROL_STR->U_a_reg, &CONTROL_STR->U_b_reg, &CONTROL_STR->U_c_reg);
        /* Add resonant controlers output */
        CONTROL_STR->U_a_reg -= CONTROL_STR->U_res_a;
        CONTROL_STR->U_b_reg -= CONTROL_STR->U_res_b;
        CONTROL_STR->U_c_reg -= CONTROL_STR->U_res_c;

    }
    else if(CONTROL_STR->state == 4)    // Ramp down
    {
        if(CONTROL_STR->U_ramp > 0)
            CONTROL_STR->U_ramp -= 0.2*T_SAMPLE;
        else
        {
            CONTROL_STR->U_ramp = 0;
            CONTROL_STR->ramp_done = 1;
        }
        CONTROL_STR->U_a_reg = CONTROL_STR->U_ramp*CONTROL_STR->DSOGI.Uf_max*CONTROL_STR->DSOGI.cos_th;
        CONTROL_STR->U_b_reg = CONTROL_STR->U_ramp*CONTROL_STR->DSOGI.Uf_max*CONTROL_STR->DSOGI.theta_reg_cos_120;
        CONTROL_STR->U_c_reg = CONTROL_STR->U_ramp*CONTROL_STR->DSOGI.Uf_max*CONTROL_STR->DSOGI.theta_reg_cos_240;
    }
    else    // Default set voltages to 0
    {
        /*
        DROOP->var.Ua=0;
        DROOP->var.Ub=0;
        DROOP->var.Uc=0;
        DROOP->var.U_ramp=0;*/
        CONTROL_STR->U_a_reg = 0;
        CONTROL_STR->U_b_reg = 0;
        CONTROL_STR->U_c_reg = 0;
    }

    //MODULATION !!!Phase order changed due to connection of converter!!!
    PWM_H4_3->CMPA.bit.CMPA = modulator(CONTROL_STR->U_a_reg);
    PWM_H4_3->CMPB.bit.CMPB = modulator(CONTROL_STR->U_a_reg);

    PWM_H4_2->CMPA.bit.CMPA = modulator(CONTROL_STR->U_b_reg);
    PWM_H4_2->CMPB.bit.CMPB = modulator(CONTROL_STR->U_b_reg);

    PWM_H4_1->CMPA.bit.CMPA = modulator(CONTROL_STR->U_c_reg);
    PWM_H4_1->CMPB.bit.CMPB = modulator(CONTROL_STR->U_c_reg);

    /*
    ad5672Write(0, 2.5f+(DROOP->var.U_res_alpha)/5.0f);
    //ad5672Write(1, 2.5f+(DROOP->var.Ua-DROOP->var.Ub)/250.0f);
    //ad5672Write(0, adc_ex_x8->X1_8);
    ad5672Write(1, 2.5f+(DROOP->var.U_res_beta)/5.0f);
    ad5672Write(2, 2.5f+(DROOP->var.P_act)/200.0f);
    ad5672Write(3, 2.5f+(DROOP->var.Q_act)/200.0f);
    //ad5672Write(2, 2.5f+(DROOP->dsogi.Uf_max-335)/2.5f);
    //ad5672Write(0, 2.5f+(DROOP->dsogi.theta_Uf)/1.6f);
    //ad5672Write(1, 2.5f+(DROOP->var.Ua-DROOP->var.Ub)/240.0f);
    //ad5672Write(2, 2.5f+(DROOP->var.Ub-DROOP->var.Uc)/240.0f);
    //ad5672Write(3, 2.5f+(DROOP->var.Uc-DROOP->var.Ua)/240.0f);
     */
}

// --------------------------------------------------------- Finite State Space machine --------------------------------------------------------------------

void FSM_exe(CONTROL_t *CONTROL_STR)
{

    switch(CONTROL_STR->state)
    {
    case 0: // STANDBY

        break;
    case 1: // RAMPING VOLTAGE UP
        if (CONTROL_STR->ramp_done)
        {
            CONTROL_STR->ramp_done = 0;
            // Prechazim do grid following
            CONTROL_STR->state = 3;
            // Prechazim do grid forming
            /*
            CONTROL_STR->state = 2;
            CONTROL_STR->DSTATE.master = 1;
            CONTROL_STR->U_d_w = 200.0f;
            CONTROL_STR->U_q_w = 0.0f;
             */

            // Vzdy sepnout stykac
            CONTROL_STR->DSTATE.connected = 1;  // V dalsim ISR sepne stykac
        }
        break;
    case 2: // GRID FORMING

        break;
    case 3: // GRID FOLLOWING
/*
        if (Protection_IsTripped())
        {
            CONTROL_STR->state = 5;                 // GoTo TRIPPED STATE
            CONTROL_STR->DSTATE.connected = 0;      // Vypne stykac

            EALLOW;
            PWM_H4_1->TZFRC.bit.OST = 1;
            PWM_H4_2->TZFRC.bit.OST = 1;
            PWM_H4_3->TZFRC.bit.OST = 1;
            EDIS;
        }*/
        /*
        if (Islanding_Check(CONTROL_STR))   // Prechod do ostrova
        {
            //ToDO odpojit stykac od site
            CONTROL_STR->state = 2; //2==grid forming mode
            CONTROL_STR->theta = CONTROL_STR->DSOGI.theta_Uf;   // Copy actual theta value
            // Set DSOGI MASTER BITE???
            // CONTROL_STR->DSTATE.master = 1;
        }*/

        break;
    case 4: // RAMPING VOLTAGE DOWN
        // ToDo Odpojit stykac? Nebo az po sjeti rampy?
        CONTROL_STR->DSTATE.connected = 0;  // V dalsim ISR odepne stykac
        if (CONTROL_STR->ramp_done)
        {
            CONTROL_STR->state = 0;
            CONTROL_STR->ramp_done = 0;
        }
        break;
    case 5: // Tripped
        // Send UDP packet and goto STANDBY state
        CONTROL_STR->DSTATE.connected = 0;  // Disconnect main relay
        CONTROL_STR->state = 4;             // GoTo STANDBY state via RAMP_DOWN state


        // Disconnect from grid and transition into island mode
        // ToDo disconnect via relay
        // ToDo after disconnect transition into grid forming mode?
        //CONTROL_STR->state = 2;
        break;
    default:
        // ToDo Add SW breakpoint???
        ESTOP0;
        break;
    }

}

void Powers(EC205_Powers *data, B2B_DSOGI *TG_sogi )
{
    data->dsogi.Uf_max = TG_sogi->Uf_max ;
    data->dsogi.w_n = TG_sogi->w_n;

/*    data->Uab = adc_ex_x8->U_TG_UV;
    data->Ubc = adc_ex_x8->U_TG_VW;
    data->Uca = adc_ex_x8->U_TG_WU;*/

    data->Ubc = adc_ex_x8->U_GRID_AB;
    data->Uca = adc_ex_x8->U_GRID_BC;
    data->Uab = adc_ex_x8->U_GRID_CA;

/*    data->Uca = adc_ex_x8->U_TG_UV;
    data->Uab = adc_ex_x8->U_TG_VW;
    data->Ubc = adc_ex_x8->U_TG_WU;*/

    data->Ia_B2B = adc_in_x17->I_TG_G_U;
    data->Ib_B2B = adc_in_x17->I_TG_G_V;
    data->Ia_syn_gen = adc_in_x17->Ia_syn_gen;
    data->Ib_syn_gen = adc_in_x17->Ib_syn_gen;
    data->Ia_asm = adc_in_x17->Ia_asm;
    data->Ib_asm = adc_in_x17->Ia_asm;
    data->Ia_BESS = adc_in_x17->Ia_BESS;
    data->Ib_BESS = adc_in_x17->Ib_BESS;
    data->Ia_Goodwe = adc_in_x17->Ia_Goodwe;
    data->Ib_Goodwe = adc_in_x17->Ib_Goodwe;

    Power_calc(data->Uab, data->Ubc, data->Ia_B2B, data->Ib_B2B, &data->P_B2B , &data->Q_B2B);
    Power_calc(data->Uab, data->Ubc, data->Ia_syn_gen, data->Ib_syn_gen, &data->P_syn_gen , &data->Q_syn_gen);
    Power_calc(data->Uab, data->Ubc, data->Ia_asm, data->Ib_asm, &data->P_asm , &data->Q_asm);
    Power_calc(data->Uab, data->Ubc, data->Ia_BESS, data->Ib_BESS, &data->P_BESS , &data->Q_BESS);
    Power_calc(data->Uab, data->Ubc, data->Ia_Goodwe, data->Ib_Goodwe, &data->P_Goodwe , &data->Q_Goodwe);
}

int modulator(float u)
{
    float CMP_val = 0.0f;
    float U_dc_pul = 0.5f*adc_ex_x8->U_DC;
    float TBPRD_pul = 0.5f*myTBPRD;

    if (u > U_dc_pul) {
        u=U_dc_pul;
        }
    if (u < -U_dc_pul) {
        u=-U_dc_pul;
        }
    CMP_val = TBPRD_pul + TBPRD_pul*(u/U_dc_pul);
    return (int)CMP_val;
}

void Resonant_ctrl_reset(CONTROL_t *CONTROL_STR, int all, int specific)
{
    if (all==1||specific==150)
    {
        CONTROL_STR->Rez_150_a.state1=0;
        CONTROL_STR->Rez_150_a.state2=0;
        CONTROL_STR->Rez_150_a.vyst=0;
        CONTROL_STR->Rez_150_b.state1=0;
        CONTROL_STR->Rez_150_b.state2=0;
        CONTROL_STR->Rez_150_b.vyst=0;
        CONTROL_STR->Rez_150_c.state1=0;
        CONTROL_STR->Rez_150_c.state2=0;
        CONTROL_STR->Rez_150_c.vyst=0;
    }
    if (all==1||specific==250)
    {
        CONTROL_STR->Rez_250_a.state1=0;
        CONTROL_STR->Rez_250_a.state2=0;
        CONTROL_STR->Rez_250_a.vyst=0;
        CONTROL_STR->Rez_250_b.state1=0;
        CONTROL_STR->Rez_250_b.state2=0;
        CONTROL_STR->Rez_250_b.vyst=0;
        CONTROL_STR->Rez_250_c.state1=0;
        CONTROL_STR->Rez_250_c.state2=0;
        CONTROL_STR->Rez_250_c.vyst=0;
    }
    if (all==1||specific==350)
    {
        CONTROL_STR->Rez_350_a.state1=0;
        CONTROL_STR->Rez_350_a.state2=0;
        CONTROL_STR->Rez_350_a.vyst=0;
        CONTROL_STR->Rez_350_b.state1=0;
        CONTROL_STR->Rez_350_b.state2=0;
        CONTROL_STR->Rez_350_b.vyst=0;
    }
    if (all==1||specific==450)
    {
        CONTROL_STR->Rez_450_a.state1=0;
        CONTROL_STR->Rez_450_a.state2=0;
        CONTROL_STR->Rez_450_a.vyst=0;
        CONTROL_STR->Rez_450_b.state1=0;
        CONTROL_STR->Rez_450_b.state2=0;
        CONTROL_STR->Rez_450_b.vyst=0;
    }
    if (all==1||specific==550)
    {
        CONTROL_STR->Rez_550_a.state1=0;
        CONTROL_STR->Rez_550_a.state2=0;
        CONTROL_STR->Rez_550_a.vyst=0;
        CONTROL_STR->Rez_550_b.state1=0;
        CONTROL_STR->Rez_550_b.state2=0;
        CONTROL_STR->Rez_550_b.vyst=0;
    }
    if (all==1||specific==650)
    {
        CONTROL_STR->Rez_650_a.state1=0;
        CONTROL_STR->Rez_650_a.state2=0;
        CONTROL_STR->Rez_650_a.vyst=0;
        CONTROL_STR->Rez_650_b.state1=0;
        CONTROL_STR->Rez_650_b.state2=0;
        CONTROL_STR->Rez_650_b.vyst=0;
    }
}
