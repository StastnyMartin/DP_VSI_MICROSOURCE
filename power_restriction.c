#include "power_restriction.h"

#include "src/global_constants.h"

// Pasmo nuloveho jaloveho vykonu
float zero_Q_min = 0.97;
float zero_Q_max = 1.05;

// Hranicni pasma napeti pro plny odber/dodavku Q
float full_Q_dodavka = 0.94;
float full_Q_odber = 1.08;

float f_threshold = FREQ_THRESHOLD_HIGH;

float f_nominal_min = 49.0f;

float f_decrease_static = 5.0f;

float f_stop = 50.05f;

// Pro PU krivky
float PU_U_FULL = 1.09f;
float PU_U_ZERO = 1.19f;



float reactivePower(float g_Um_DSOGI, float w_q_power)
{
    static enum VOLTAGE_STATE v_state = Nominal;
    if (g_Um_DSOGI > zero_Q_max * VOLTAGE_NOMINAL)
    {
        if (g_Um_DSOGI > full_Q_odber * VOLTAGE_NOMINAL)
        {
            v_state = High_full_Q;
        }
        else
        {
            v_state = High;
        }
    }
    else if (g_Um_DSOGI < zero_Q_min * VOLTAGE_NOMINAL)
    {
        if (g_Um_DSOGI < full_Q_dodavka * VOLTAGE_NOMINAL)
        {
            v_state = Low_full_Q;
        }
        else
        {
            v_state = Low;
        }
    }
    else
    {
        v_state = Nominal;
    }

    /* Finite state machine */
    float k1 = -1.0f / (zero_Q_min - full_Q_dodavka);
    float k2 = -1.0f / (full_Q_odber - zero_Q_max);
    switch (v_state)
    {
    case Low:
        w_q_power = k1 * ((g_Um_DSOGI / VOLTAGE_NOMINAL) - zero_Q_min);
        break;
    case Low_full_Q:
        w_q_power = 1.0;
        break;
    case High:
        w_q_power = k2 * ((g_Um_DSOGI / VOLTAGE_NOMINAL) - zero_Q_max);
        break;
    case High_full_Q:
        w_q_power = -1.0;
        break;
    case Nominal:
        w_q_power = 0.0;
        break;
    default:
        //printf("Error, state not defined!");
        break;
    }

    return w_q_power * Q_NOMINAL;
}

float activePower(float g_w_DSOGI, float g_Um_DSOGI, float w_p_power)
{
    // Pf 
	static enum FREQ_STATE f_state = NOMINAL;
	float g_freq = g_w_DSOGI / (2.0f * PI);

    uint16_t f_stop_flag = 0;

   

    // Calc freq equal to 0 active power
    //float f_end = f_decrease_static * f_threshold + f_threshold;
    float f_end = (f_decrease_static / 100.0f) * f_threshold + f_threshold;

    float delta_P = 0.0f;

    if (g_freq > f_threshold)
    {
		f_stop_flag = 1;
		if (g_freq > f_end)
			f_state = HIGH_ZERO_P;
        else
			f_state = HIGH;
	}
    else if (g_freq < f_nominal_min)
    {
		f_state = LOW;
    }
    else
    {
        f_state = NOMINAL;
	}

    /* Finite state machine */
    switch (f_state)
    {
    case NOMINAL:
        //printf("Nominal frequency, no power restriction");)
        break;
    case HIGH:
    {
        //delta_P = P_NOMINAL * (f_end - g_freq) / (f_decrease_static * FREQ_NOMINAL);
        delta_P = P_NOMINAL * (100.0f / f_decrease_static) * ((g_freq - f_threshold) / FREQ_NOMINAL);
        float p_power = P_NOMINAL - delta_P;

        if (w_p_power > p_power)
            w_p_power = p_power;

        break;
    }
    case HIGH_ZERO_P:
        w_p_power = 0.0f;
        break;

    case LOW:
        // Defaultne nerestrikovat odber pri nizke frekvenci, ale muze se nastavit i omezeni
        break;

    default:
        //printf("Error, state not defined!");
        break;
    }

    // PU
    static enum POW_VOL_STATE pow_vol_state = FULL;

    if (g_Um_DSOGI > PU_U_FULL*VOLTAGE_NOMINAL)
    {
        if (g_Um_DSOGI > PU_U_ZERO*VOLTAGE_NOMINAL)
        {
            pow_vol_state = ZERO;
        }
        else
        {
            pow_vol_state = RESTRICTION;
		}
    }
    else
    {
        pow_vol_state = FULL;
    }

    /* Finite state machine */

    switch (pow_vol_state)
    {
    case FULL:
        break;
    case ZERO:
        w_p_power = 0.0f;
        break;
    case RESTRICTION:
    {
        float k = -1.0f / (PU_U_ZERO - PU_U_FULL);
        float p_power = P_NOMINAL + (k * P_NOMINAL * ((g_Um_DSOGI / VOLTAGE_NOMINAL) - PU_U_FULL));
        if (w_p_power > p_power)
            w_p_power = p_power;
        break;
    }
    default:
        //printf("Error, state not defined!");
        break;
    }
    
	return w_p_power;
}
