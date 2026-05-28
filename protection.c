/**
 ********************************************************************************
 * @file    protection.c
 * @author  Stastny Martin
 * @date    27. 4. 2026
 * @brief   
 ********************************************************************************
 */

#include "src/protection.h"

/* --- Interní typy --- */
typedef struct
{
    const float *p_amplitude;
    const float *p_frequency;
    uint32_t *p_status_word;

    float current_rms;
    float avg_u_10min;
    bool trip;

    uint32_t cnt_u_over_2;
    uint32_t cnt_u_over_1;
    uint32_t cnt_u_under_1;
    uint32_t cnt_u_under_2;
    uint32_t cnt_f_over;
    uint32_t cnt_f_under;

    float sec_accumulator;
    uint32_t sec_sample_cnt;
} Protection_Internal_t;

/* --- Statické promìnné modulu --- */

/* Instance je viditelná pouze v tomto souboru */
static volatile Protection_Internal_t prot;

#define AVG_WINDOW_S 600
static float ring_buffer[AVG_WINDOW_S];
static uint16_t ring_idx = 0;
static float ring_sum = 0;

/* Pomocné makro */
#define SEC_TO_SAMPLES(sec) ((uint32_t)((sec) * (float)(F_PWM)))

/* --- Implementace funkcí --- */

void Protection_Init(const float *ext_amp,const float *ext_freq,uint32_t *ext_status_word)
{
    prot.p_amplitude = ext_amp;
    prot.p_frequency = ext_freq;
    prot.p_status_word = ext_status_word;

    prot.trip = false;
    if (prot.p_status_word != 0)
    {
        *(prot.p_status_word) = 0;
    }

    prot.avg_u_10min = NOMINAL_VOLTAGE_UN;

    ring_sum = NOMINAL_VOLTAGE_UN * (float)AVG_WINDOW_S;
    for (int i = 0; i < AVG_WINDOW_S; i++)
    {
        ring_buffer[i] = NOMINAL_VOLTAGE_UN;
    }

    prot.sec_accumulator = 0.0f;
    prot.sec_sample_cnt = 0;

    prot.cnt_u_over_2 = 0;
    prot.cnt_u_over_1 = 0;
    prot.cnt_u_under_1 = 0;
    prot.cnt_u_under_2 = 0;
    prot.cnt_f_over = 0;
    prot.cnt_f_under = 0;
}

void Protection_Update(void)
{
    /* 1. Výpoèet aktuálních hodnot */
    prot.current_rms = (*prot.p_amplitude) / SQRT2;
    float f = *prot.p_frequency / (PIx2);

    /* --- U>> --- */
    if (prot.current_rms > (1.20f * NOMINAL_VOLTAGE_UN))
    {
        prot.cnt_u_over_2++;
        if (prot.cnt_u_over_2 >= SEC_TO_SAMPLES(0.1f))
        {
            prot.trip = true;
            if (prot.p_status_word) *(prot.p_status_word) |= PROT_BIT_U_OVER_2;
        }
    }
    else { prot.cnt_u_over_2 = 0; }

    /* --- U> --- */
    if (prot.current_rms > (1.15f * NOMINAL_VOLTAGE_UN))
    {
        prot.cnt_u_over_1++;
        if (prot.cnt_u_over_1 >= SEC_TO_SAMPLES(5.0f))
        {
            prot.trip = true;
            if (prot.p_status_word) *(prot.p_status_word) |= PROT_BIT_U_OVER_1;
        }
    }
    else { prot.cnt_u_over_1 = 0; }

    /* --- U< --- */
    if (prot.current_rms < (0.70f * NOMINAL_VOLTAGE_UN))
    {
        prot.cnt_u_under_1++;
        if (prot.cnt_u_under_1 >= SEC_TO_SAMPLES(2.7f))
        {
            prot.trip = true;
            if (prot.p_status_word) *(prot.p_status_word) |= PROT_BIT_U_UNDER_1;
        }
    }
    else { prot.cnt_u_under_1 = 0; }

    /* --- U<< --- */
    if (prot.current_rms < (0.45f * NOMINAL_VOLTAGE_UN))
    {
        prot.cnt_u_under_2++;
        if (prot.cnt_u_under_2 >= SEC_TO_SAMPLES(0.2f))
        {
            prot.trip = true;
            if (prot.p_status_word) *(prot.p_status_word) |= PROT_BIT_U_UNDER_2;
        }
    }
    else { prot.cnt_u_under_2 = 0; }

    /* --- f> --- */
    if (f > 51.5f)
    {
        prot.cnt_f_over++;
        if (prot.cnt_f_over >= SEC_TO_SAMPLES(0.1f))
        {
            prot.trip = true;
            if (prot.p_status_word) *(prot.p_status_word) |= PROT_BIT_F_OVER;
        }
    }
    else { prot.cnt_f_over = 0; }

    /* --- f< --- */
    if (f < 47.5f)
    {
        prot.cnt_f_under++;
        if (prot.cnt_f_under >= SEC_TO_SAMPLES(0.1f))
        {
            prot.trip = true;
            if (prot.p_status_word) *(prot.p_status_word) |= PROT_BIT_F_UNDER;
        }
    }
    else { prot.cnt_f_under = 0; }

    /* --- 10min prùmìr --- */
    prot.sec_accumulator += prot.current_rms;
    prot.sec_sample_cnt++;

    if (prot.sec_sample_cnt >= (uint32_t)F_PWM)
    {
        float sec_avg = prot.sec_accumulator / (float)prot.sec_sample_cnt;
        ring_sum -= ring_buffer[ring_idx];
        ring_buffer[ring_idx] = sec_avg;
        ring_sum += ring_buffer[ring_idx];

        ring_idx = (ring_idx + 1) % AVG_WINDOW_S;
        prot.avg_u_10min = ring_sum / (float)AVG_WINDOW_S;

        if (prot.avg_u_10min > (1.11f * NOMINAL_VOLTAGE_UN))
        {
            prot.trip = true;
            if (prot.p_status_word) *(prot.p_status_word) |= PROT_BIT_U_10MIN;
        }
        prot.sec_accumulator = 0.0f;
        prot.sec_sample_cnt = 0;
    }
}

bool Protection_IsTripped(void)
{
    return prot.trip;
}

void Protection_Reset(void)
{
    prot.trip = false;
    if (prot.p_status_word) *(prot.p_status_word) = 0;

    prot.cnt_u_over_2 = 0;
    prot.cnt_u_over_1 = 0;
    prot.cnt_u_under_1 = 0;
    prot.cnt_u_under_2 = 0;
    prot.cnt_f_over = 0;
    prot.cnt_f_under = 0;
}

uint32_t Protection_GetDiagnosticWord(void)
{
    uint32_t diag = 0;

    // 1. Naèteme bity jednotlivých poruch, které tam zapsal Protection_Update[cite: 20]
    if (prot.p_status_word != 0)
    {
        diag = *(prot.p_status_word);
    }

    // 2. Na 8. bit (index 7) pøidáme souhrnný stav tripu[cite: 21]
    if (prot.trip)
    {
        diag |= (1U << 7);
    }
    else
    {
        diag &= ~(1U << 7);
    }

    return diag;
}
