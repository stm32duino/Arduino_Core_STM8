#ifndef _PINNAMES_H
#define _PINNAMES_H

#include "PinNamesTypes.h"
#include "PortNames.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        PA_0 = (PortA << 4) + 0x00,
        PA_1 = (PortA << 4) + 0x01,
        PA_2 = (PortA << 4) + 0x02,
        PA_3 = (PortA << 4) + 0x03,
        PA_4 = (PortA << 4) + 0x04,
        PA_5 = (PortA << 4) + 0x05,
        PA_6 = (PortA << 4) + 0x06,
        PA_7 = (PortA << 4) + 0x07,

        PB_0 = (PortB << 4) + 0x00,
        PB_1 = (PortB << 4) + 0x01,
        PB_2 = (PortB << 4) + 0x02,
        PB_3 = (PortB << 4) + 0x03,
        PB_4 = (PortB << 4) + 0x04,
        PB_5 = (PortB << 4) + 0x05,
        PB_6 = (PortB << 4) + 0x06,
        PB_7 = (PortB << 4) + 0x07,

#if defined (GPIOC_BaseAddress) || defined(GPIOC_BASE)
        PC_0 = (PortC << 4) + 0x00,
        PC_1 = (PortC << 4) + 0x01,
        PC_2 = (PortC << 4) + 0x02,
        PC_3 = (PortC << 4) + 0x03,
        PC_4 = (PortC << 4) + 0x04,
        PC_5 = (PortC << 4) + 0x05,
        PC_6 = (PortC << 4) + 0x06,
        PC_7 = (PortC << 4) + 0x07,
#endif
#if defined GPIOD_BaseAddress || defined(GPIOD_BASE)
        PD_0 = (PortD << 4) + 0x00,
        PD_1 = (PortD << 4) + 0x01,
        PD_2 = (PortD << 4) + 0x02,
        PD_3 = (PortD << 4) + 0x03,
        PD_4 = (PortD << 4) + 0x04,
        PD_5 = (PortD << 4) + 0x05,
        PD_6 = (PortD << 4) + 0x06,
        PD_7 = (PortD << 4) + 0x07,
#endif
#if defined GPIOE_BaseAddress || defined(GPIOE_BASE)
        PE_0 = (PortE << 4) + 0x00,
        PE_1 = (PortE << 4) + 0x01,
        PE_2 = (PortE << 4) + 0x02,
        PE_3 = (PortE << 4) + 0x03,
        PE_4 = (PortE << 4) + 0x04,
        PE_5 = (PortE << 4) + 0x05,
        PE_6 = (PortE << 4) + 0x06,
        PE_7 = (PortE << 4) + 0x07,
#endif
#if defined GPIOF_BaseAddress || defined(GPIOF_BASE)
        PF_0 = (PortF << 4) + 0x00,
        PF_1 = (PortF << 4) + 0x01,
        PF_2 = (PortF << 4) + 0x02,
        PF_3 = (PortF << 4) + 0x03,
        PF_4 = (PortF << 4) + 0x04,
        PF_5 = (PortF << 4) + 0x05,
        PF_6 = (PortF << 4) + 0x06,
        PF_7 = (PortF << 4) + 0x07,
#endif
#if defined GPIOG_BaseAddress || defined(GPIOG_BASE)
        PG_0 = (PortG << 4) + 0x00,
        PG_1 = (PortG << 4) + 0x01,
        PG_2 = (PortG << 4) + 0x02,
        PG_3 = (PortG << 4) + 0x03,
        PG_4 = (PortG << 4) + 0x04,
        PG_5 = (PortG << 4) + 0x05,
        PG_6 = (PortG << 4) + 0x06,
        PG_7 = (PortG << 4) + 0x07,
#endif
#if defined GPIOH_BaseAddress || defined(GPIOH_BASE)
        PH_0 = (PortH << 4) + 0x00,
        PH_1 = (PortH << 4) + 0x01,
        PH_2 = (PortH << 4) + 0x02,
        PH_3 = (PortH << 4) + 0x03,
        PH_4 = (PortH << 4) + 0x04,
        PH_5 = (PortH << 4) + 0x05,
        PH_6 = (PortH << 4) + 0x06,
        PH_7 = (PortH << 4) + 0x07,
#endif
#if defined GPIOI_BaseAddress || defined(GPIOI_BASE)
        PI_0 = (PortI << 4) + 0x00,
#endif
        //NOT connecter
        NC = (int)0xFF
    } PinName;

#ifdef __cplusplus
}
#endif

#endif
