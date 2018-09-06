#if !defined(NO_HWSERIAL)
#if defined(STM8S105) || defined(STM8S005) ||  defined (STM8AF626x)
#include "stm8s_uart2.c"
#endif
#endif /* !NO_HWSERIAL */