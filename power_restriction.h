#ifndef POWER_RESTRICTION_H
#define POWER_RESTRICTION_H

//#include <iostream>
//#include <random>
//#include <fstream>
//#include <iomanip>

#include <stdint.h>

//#define PI  3.14159265358979323846


#define Q_NOMINAL       5000.0f
#define VOLTAGE_NOMINAL 325.0f

#define P_NOMINAL       5000.0f
#define FREQ_NOMINAL    50.0f

#define FREQ_THRESHOLD_HIGH  50.2f


// Pasmo nuloveho jaloveho vykonu
extern float zero_Q_min;
extern float zero_Q_max;

// Hranicni pasma napeti pro plny odber/dodavku Q
extern float full_Q_dodavka;
extern float full_Q_odber;


// pf
extern float f_threshold;

extern float f_stop;



enum VOLTAGE_STATE
{
    Nominal,        // Nominal voltage state
    Low,            // Voltage lower than nominal, start supply reactive power (Q)
    Low_full_Q,     // Voltage low, full reactive power supply
    High,           // Voltage higher than nominal, start consume reactive power (-Q)
    High_full_Q     // Voltage high, full reactive power consume
};

enum FREQ_STATE {
    NOMINAL,        // Nominal freq state
    HIGH,           // High freq state, start power restriction
    HIGH_ZERO_P,    // High freq zero power
    LOW,            // Low freq state
};

enum POW_VOL_STATE {
    FULL,
    ZERO,
    RESTRICTION,
};


float reactivePower(float g_Um_DSOGI, float w_q_power);

float activePower(float g_w_DSOGI, float g_Um_DSOGI, float w_p_power);


#endif // POWER_CONTROL_H
