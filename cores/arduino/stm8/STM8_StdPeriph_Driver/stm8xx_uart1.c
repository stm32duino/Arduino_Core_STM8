#if !defined(NO_HWSERIAL)
#if defined(STM8S208) ||defined(STM8S207) || defined(STM8S007) ||defined(STM8S103) ||\
    defined(STM8S003) ||defined(STM8S001) || defined(STM8S903) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
 #include "stm8s_uart1.c"
#endif /* (STM8S208) || (STM8S207) || (STM8S103) || (STM8S001) || (STM8S903) || (STM8AF52Ax) || (STM8AF62Ax) */
#endif /* !NO_HWSERIAL */