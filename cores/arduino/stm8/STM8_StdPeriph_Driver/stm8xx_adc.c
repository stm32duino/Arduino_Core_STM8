#if defined(STM8S105) || defined(STM8S005) || defined(STM8S103) || defined(STM8S003) || \
    defined(STM8S903) || defined(STM8AF626x) || defined(STM8AF622x)
#include "stm8s_adc1.c"
#endif

#if defined(STM8S208) || defined(STM8S207) || defined (STM8S007) || defined (STM8AF52Ax) ||\
    defined (STM8AF62Ax)
#include "stm8s_adc2.c"
#endif

#if defined (STM8L15X_MD) || defined (STM8L15X_MDP) || defined (STM8L15X_HD) || defined (STM8L15X_LD) \
|| defined (STM8L05X_LD_VL) || defined (STM8L05X_MD_VL) || defined (STM8L05X_HD_VL) || defined (STM8AL31_L_MD)
#include "stm8l15x_adc.c"
#endif
