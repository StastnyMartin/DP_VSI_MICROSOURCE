/**
 ********************************************************************************
 * @file    protection.h
 * @author  Stastny Martin
 * @date    27. 4. 2026
 * @brief   
 ********************************************************************************
 */

#ifndef SRC_PROTECTION_H_
#define SRC_PROTECTION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "src/global_constants.h"

/* Parametry soustavy */
#define NOMINAL_VOLTAGE_UN  325.0f
#define SQRT2               1.41421356f

/* Definice bitových masek poruch */
#define PROT_BIT_U_OVER_2   (1U << 0)
#define PROT_BIT_U_OVER_1   (1U << 1)
#define PROT_BIT_U_10MIN    (1U << 2)
#define PROT_BIT_U_UNDER_1  (1U << 3)
#define PROT_BIT_U_UNDER_2  (1U << 4)
#define PROT_BIT_F_OVER     (1U << 5)
#define PROT_BIT_F_UNDER    (1U << 6)

/* Veøejné rozhraní modulu */

/**
 * Inicializace ochran. Propojí vnitøní logiku s externími daty z PLL.
 */
void Protection_Init(const float *ext_amp,
                     const float *ext_freq,
                     uint32_t *ext_status_word);

/**
 * Hlavní procesní funkce. Volat v ADC ISR (10 kHz).
 */
void Protection_Update(void);

/**
 * Vrací true, pokud došlo k aktivaci jakékoliv ochrany.
 */
bool Protection_IsTripped(void);

/**
 * Resetuje pøíznak tripu a stavové slovo.
 */
void Protection_Reset(void);


/**
 * Zapíše souhrnný stav tripu do externí promìnné na 8. bit (index 7) a sepnuti prislusnych ochran
 */
uint32_t Protection_GetDiagnosticWord(void);

#ifdef __cplusplus
}
#endif

#endif 
