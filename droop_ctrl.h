/*
 * droop_ctrl.h
 *
 *  Created on: 16. 3. 2026
 *      Author: Stastny Martin
 */

#ifndef SRC_DROOP_CTRL_H_
#define SRC_DROOP_CTRL_H_

/* Includes */
#include "math.h"
#include "stdint.h"
#include "regulators.h"
#include "res_reg.h"

/* Voltage PI reg constants */
#define Kp_PI_V 0.1f
#define Ki_PI_V 8.0f

/* Current PI reg constants */
#define Kp_PI_I 2.0f    // 8.0f
#define Ki_PI_I 400.0f  //400.0f

/* Limitations for voltage/current controlers */
#define U_dc     750.0f
#define Imax    16.0f

typedef struct
{
    float *U_ab;
    float *U_bc;
    float *U_ca;

    float U_a;
    float U_b;
    float U_c;

    float *U_DC;

    float *I_a_conv;
    float *I_b_conv;
    float *I_c_conv;
    float *I_a_grid;
    float *I_b_grid;
    float *I_c_grid;
}MEAS_t;

typedef struct{
    float theta_Uf;
    float U_amp_act;
    float Uf_max;
    float U_d_grid;
    float U_q_grid;
    float w_n;
    float cos_th;
    float theta_reg_cos_120;
    float theta_reg_cos_240;
    float sin_th;
} B2B_DSOGI;

typedef struct{
    uint16_t activate:1;        //device is active
    uint16_t ASM_relay:1;       //Control relay connecting ASM to the grid
    uint16_t grid_voltage:1;    //grid voltage detection
    uint16_t master:1;          //master mode (no grid voltage detected)
    uint16_t ramp_done:1;       //slow ramp to avoid oscillations
    uint16_t locking:1;         //PLL is locking to existing grid
    uint16_t locked:1;          //PLL is locked
    uint16_t connected:1;       //Main switch is active
    uint16_t stnd_op:1;         //Standard operation (Droop control active)
    uint16_t button:1;          //Button has been pressed
    uint16_t Q_U_curve:1;       //Activate Q/U proportional control
    uint16_t Q_U_PI:1;          //Activate Q/U PI control
    uint16_t P_f_curve:1;       //Activate P/f proportional control
    uint16_t state;             //1-standby Uout=0 2-ramping output voltage 3-nominal output voltage 4-switch on 5-active control
    uint16_t reg_on;          //0-Regulators off --> only Feed-Forward
    uint16_t res_reg_on;        //0-Resonant regulators for high freq OFF
    uint16_t prot_on;
}DEVICE_STATE;

typedef struct
{
    MEAS_t MEAS;
    B2B_DSOGI DSOGI;

    uint16_t state;
    /*
     * if 0 VSI in standby - U_reg_abc == 0
     * if 1 VSI ramping output voltage to nominal
     * if 2 VSI in grid forming mode
     * if 3 VSI in grid following mode
     * if 4 VSI is ramping output voltage to 0
     *  */

    uint16_t ramp_done;

    float I_d_conv;
    float I_q_conv;

    float I_d_grid;
    float I_q_grid;

    // Napeti regulatoru
    float U_d;
    float U_q;

    float U_d_grid;
    float U_q_grid;

    // Napeti do modulatoru
    float U_a_reg;
    float U_b_reg;
    float U_c_reg;

    float I_d_w;
    float I_q_w;

    float U_d_w;
    float U_q_w;

    float f_w;
    float theta;    // Integration of w

    RegPI_t Reg_voltage_d;
    RegPI_t Reg_voltage_q;
    RegPI_t Reg_current_d;
    RegPI_t Reg_current_q;

    float U_d_FF;
    float U_q_FF;

    // Pro grid-feeding
    float P_w;
    float Q_w;

    float P_w_res;  // Pozadavek na P po restrikci --> Pf/PU krivka
    float Q_w_res;

    float U_ramp;



    DEVICE_STATE DSTATE;

    //Resonant controllers
    float I_alpha;
    float I_beta;

    float U_res_alpha;          // Sum of all resonant controlers outputs in alpha/beta coord
    float U_res_beta;           // Sum of all resonant controlers outputs in alpha/beta coord

    float U_res_a;              // Sum of all resonant controlers outputs in abc coord
    float U_res_b;              // Sum of all resonant controlers outputs in abc coord
    float U_res_c;              // Sum of all resonant controlers outputs in abc coord

    uint16_t act_3rd:1;
    uint16_t act_5th:1;
    uint16_t act_7th:1;
    uint16_t act_9th:1;
    uint16_t act_11th:1;
    uint16_t act_13th:1;


    RegRez_t Rez_150_a;
    RegRez_t Rez_150_b;
    RegRez_t Rez_150_c;
    RegRez_t Rez_250_a;
    RegRez_t Rez_250_b;
    RegRez_t Rez_250_c;
    RegRez_t Rez_350_a;
    RegRez_t Rez_350_b;
    RegRez_t Rez_350_c;
    RegRez_t Rez_450_a;
    RegRez_t Rez_450_b;
    RegRez_t Rez_450_c;
    RegRez_t Rez_550_a;
    RegRez_t Rez_550_b;
    RegRez_t Rez_650_a;
    RegRez_t Rez_650_b;

}CONTROL_t;



/* Structures */
typedef struct{
    //Currents
    volatile float* Ia;
    volatile float* Ia_grid;
    volatile float* Ib;
    volatile float* Ib_grid;
    volatile float* Ic;
    volatile float* Ic_grid;
    float I_conv_d;
    float I_conv_q;
    float I_grid_d;
    float I_grid_q;

    float I_w_d;
    float I_w_q;
    float I_sum_d;
    float I_sum_q;
    //Voltages
    volatile float* Udc;
    float U_ramp;
    float Ua;
    float Ub;
    float Uc;
    //Resonant controllers
    float I_alpha;
    float I_beta;

    float U_res_alpha;          // Sum of all resonant controlers outputs in alpha/beta coord
    float U_res_beta;           // Sum of all resonant controlers outputs in alpha/beta coord

    float U_res_a;              // Sum of all resonant controlers outputs in abc coord
    float U_res_b;              // Sum of all resonant controlers outputs in abc coord
    float U_res_c;              // Sum of all resonant controlers outputs in abc coord

    uint16_t act_3rd:1;
    uint16_t act_5th:1;
    uint16_t act_7th:1;
    uint16_t act_9th:1;
    uint16_t act_11th:1;
    uint16_t act_13th:1;


    RegRez_t Rez_150_a;
    RegRez_t Rez_150_b;
    RegRez_t Rez_150_c;
    RegRez_t Rez_250_a;
    RegRez_t Rez_250_b;
    RegRez_t Rez_250_c;
    RegRez_t Rez_350_a;
    RegRez_t Rez_350_b;
    RegRez_t Rez_450_a;
    RegRez_t Rez_450_b;
    RegRez_t Rez_550_a;
    RegRez_t Rez_550_b;
    RegRez_t Rez_650_a;
    RegRez_t Rez_650_b;

    //Control voltages
    float U_reg_d;
    float U_reg_q;
    float U_fw_d;
    float U_fw_q;
    float U_osc_d;
    float U_osc_q;
    float U_reg_a;
    float U_reg_b;
    float U_reg_c;
    //Voltage stab
    float U_stab_sum;
    float Kp_stab;
    float Ki_stab;
    float e_stab;
    volatile float* Q_max;
    //Frequency stab
    float delta_f;
    float Kp_Pf;

    volatile float* U_amp_w;
    volatile float* f_w;

    //Powers
    float P_act;
    volatile float* P_w;
    float Q_act;
    volatile float* Q_w;

} DROOP_data;

/*
typedef enum
{
    STATE_STANDBY,          // Uout==0
    STATE_RAMP,             //
    STATE_GRID_FORMING,     // Output voltage on nominal values, PI regulation voltage on PCC
    STATE_GRID_FOLLOWING    // Injects current depended on P_w, Q_w and actual grid parameters from MEAS such as U_amp, freq, etc.
}STATE;*/

typedef struct{
    B2B_DSOGI dsogi;
    float Uab;
    float Ubc;
    float Uca;
    float Ia_B2B;
    float Ib_B2B;
    float P_B2B;
    float Q_B2B;
    float Ia_syn_gen;
    float Ib_syn_gen;
    float P_syn_gen;
    float Q_syn_gen;
    float Ia_asm;
    float Ib_asm;
    float P_asm;
    float Q_asm;
    float Ia_BESS;
    float Ib_BESS;
    float P_BESS;
    float Q_BESS;
    float Ia_Goodwe;
    float Ib_Goodwe;
    float P_Goodwe;
    float Q_Goodwe;
//    float Uf_max;
//    float w_n;
} EC205_Powers;

typedef struct {
    DROOP_data var;
    B2B_DSOGI dsogi;
} DROOP_DATA;


/* Global var */
extern CONTROL_t *CONTROL;

/* Functions */
uint16_t droopInit();


/* Ukradeno */
void DROOP_init(DROOP_DATA *DROOP,DEVICE_STATE *STATE);
void DROOP_exe(CONTROL_t *CONTROL_STR);
void FSM_exe(CONTROL_t *CONTROL_STR);
void Powers(EC205_Powers *data, B2B_DSOGI *TG_sogi);

int modulator(float u);

void Resonant_ctrl_reset(CONTROL_t *CONTROL_STR, int all, int specific);

//void Resonant_ctrl_reset(DROOP_DATA *DROOP, int all, int specific);

#endif /* SRC_DROOP_CTRL_H_ */
