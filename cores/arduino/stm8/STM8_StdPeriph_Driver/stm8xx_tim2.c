#if defined(STM8Sxx)
#if !defined(STM8S903) || !defined(STM8AF622x)
#include "stm8s_tim2.c"
#endif
#endif /* (STM8S903) || (STM8AF622x) */
#if defined(STM8Lxx)
#include "stm8l15x_tim2.c"
#endif