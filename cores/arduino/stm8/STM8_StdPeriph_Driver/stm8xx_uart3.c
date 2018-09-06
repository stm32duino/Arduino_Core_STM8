#if !defined(NO_HWSERIAL)
#if defined(STM8S208) ||defined(STM8S207) || defined(STM8S007) || defined (STM8AF52Ax) ||\
    defined (STM8AF62Ax)
#include "stm8s_uart3.c"
#endif
#endif /* !NO_HWSERIAL */