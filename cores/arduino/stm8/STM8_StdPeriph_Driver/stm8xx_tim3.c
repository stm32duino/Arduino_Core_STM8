#if defined(STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8S105) || \
    defined(STM8S005) || defined(STM8AF52Ax) || defined(STM8AF62Ax) || defined(STM8AF626x)
#include "stm8s_tim3.c"
#endif /* (STM8S208) ||defined(STM8S207) || defined(STM8S007) ||defined(STM8S105) */
#if defined(STM8Lxx)
#include "stm8l15x_tim3.c"
#endif