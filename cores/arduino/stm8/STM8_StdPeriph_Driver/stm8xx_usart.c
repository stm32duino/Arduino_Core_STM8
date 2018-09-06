#if !defined(NO_HWSERIAL)
#if defined (STM8L15X_MD) || defined (STM8L15X_MDP) || defined (STM8L15X_HD) || defined (STM8L15X_LD) \
|| defined (STM8L05X_LD_VL) || defined (STM8L05X_MD_VL) || defined (STM8L05X_HD_VL) || defined (STM8AL31_L_MD)
#include "stm8l15x_usart.c"
#endif
#endif /* !NO_HWSERIAL */