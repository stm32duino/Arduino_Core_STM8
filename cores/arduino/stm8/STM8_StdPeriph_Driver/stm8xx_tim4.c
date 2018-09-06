#if defined(STM8Sxx)
#if !defined(STM8S903) || !defined(STM8AF622x)
#include "stm8s_tim4.c"
#endif /* (STM8S903) || (STM8AF622x) */
#endif
#if defined(STM8Lxx)
#include "stm8l15x_tim4.c"
#endif