#ifndef _STM8_DEF_
#define _STM8_DEF_

/**
 * @brief STM8 core version number
 */
#define STM8_CORE_VERSION_MAJOR    (0x01U) /*!< [31:24] major version */
#define STM8_CORE_VERSION_MINOR    (0x00U) /*!< [23:16] minor version */
#define STM8_CORE_VERSION_PATCH    (0x00U) /*!< [15:8]  patch version */
/*
 * Extra label for development:
 * 0: official release
 * [1-9]: release candidate
 * F[0-9]: development
 */
#define STM8_CORE_VERSION_EXTRA    (0x00U) /*!< [7:0]  extra version */
#define STM8_CORE_VERSION          ((STM8_CORE_VERSION_MAJOR << 24U)\
                                        |(STM8_CORE_VERSION_MINOR << 16U)\
                                        |(STM8_CORE_VERSION_PATCH << 8U )\
                                        |(STM8_CORE_VERSION_EXTRA))

#if defined(STM8Sxx)
#include "stm8s.h"
#elif defined(STM8Lxx)
#include "stm8l15x.h"
#else
#error "Please select first the target STM8S/A/L device used in your application "
#endif

/* Here define some compatibility */
#if defined(STM8Sxx)
#define AFIO_NONE 0
#if defined(ADC2)
#define ADCx ADC2
#elif defined(ADC1)
#define ADCx ADC1
#else
#error "Can't define ADCx"
#endif
#endif /* STM8Sxx */

#ifdef __cplusplus
extern "C"
{
#endif
    void SystemClock_Config(void);
    void _Error_Handler(const char *, int);
#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif //_STM8_DEF_
