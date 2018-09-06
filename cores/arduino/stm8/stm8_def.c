#include "stm8_def.h"

#ifdef __cplusplus
extern "C"
{
#endif

#pragma weak _Error_Handler
    void _Error_Handler(const char *msg, int val)
    {
        while (1)
        {
        }
    }
#ifdef __cplusplus
}
#endif