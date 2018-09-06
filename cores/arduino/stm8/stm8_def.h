#ifndef _STM8_DEF_
#define _STM8_DEF_

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
