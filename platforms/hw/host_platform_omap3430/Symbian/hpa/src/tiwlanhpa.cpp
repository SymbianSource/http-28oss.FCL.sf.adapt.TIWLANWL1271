/*
 * tiwlanhpa.cpp
 *
 * Copyright(c) 1998 - 2010 Texas Instruments. All rights reserved.      
 * All rights reserved.      
 * 
 * This program and the accompanying materials are made available under the 
 * terms of the Eclipse Public License v1.0 or BSD License which accompanies
 * this distribution. The Eclipse Public License is available at
 * http://www.eclipse.org/legal/epl-v10.html and the BSD License is as below.                                   
 *                                                                       
 * Redistribution and use in source and binary forms, with or without    
 * modification, are permitted provided that the following conditions    
 * are met:                                                              
 *                                                                       
 *  * Redistributions of source code must retain the above copyright     
 *    notice, this list of conditions and the following disclaimer.      
 *  * Redistributions in binary form must reproduce the above copyright  
 *    notice, this list of conditions and the following disclaimer in    
 *    the documentation and/or other materials provided with the         
 *    distribution.                                                      
 *  * Neither the name Texas Instruments nor the names of its            
 *    contributors may be used to endorse or promote products derived    
 *    from this software without specific prior written permission.      
 *                                                                       
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT      
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT   
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



/** \file  TIWlanHpa.cpp 
 *  \brief  Interface between the sandbox to the BSP of Omap3430
 *
 *  \see   
 */

#include <omap_reg.h>

#include "kernel.h"
#include "TIWlanHpa.h"
#include "wlanhpa.h"
#include "wlanosaext.h"

/* To debug HPA, define some prints here using OSA*/
#define     TRACE(msg) 
/*Kern::Printf(msg)*/
#define     ERROR_TRACE(msg) Kern::Printf(msg);

 
extern void Assert( const TInt8*, TInt, TInt );

/** 
 * \fn     Create
 * \brief  static creator
 * 
 * Create hpa layer object
 *
 * \note   
 * \param aOsaExt osa extension object
 * \return hpa layer object, NULL upon failure
 */ 
WlanHpa* WlanHpa::Create( MWlanOsaExt& aOsaExt, MWlanOsa& aOsa)
{
    TRACE(("WlanHpa::Create\n"));

    return new TIWlanHpa(aOsaExt, aOsa);
}

/** 
 * \fn     Destroy
 * \brief  
 * 
 * Destroy hpa layer object
 *
 * \note   
 * \param aWlanHpa hpa layer object
 */ 
void WlanHpa::Destroy( WlanHpa* aWlanHpa )
{    
    if (aWlanHpa)
    {
        delete aWlanHpa;
    }
}

/** 
 * \fn     TIWlanHpa
 * \brief  constructor
 * 
 * \note   
 * \param  aOsaExt 
 */ 
TIWlanHpa::TIWlanHpa(MWlanOsaExt& aOsaExt, MWlanOsa& aOsa):
	WlanHpa( aOsaExt, aOsa),
	iIrqDfc(InterruptHandlerDfc,(TAny*)this,0)
{
    TRACE(("%s\n",__FUNCTION__));
    TRACE(("construct"));
    iIrqDfc.SetDfcQ( Kern::DfcQue0() );
}

/** 
 * \fn     TIWlanHpa
 * \brief  destructor
 * 
 * \note   
 */ 
TIWlanHpa::~TIWlanHpa()
{
        TRACE(("%s\n",__FUNCTION__));

}
 
/** 
 * \fn     TIWlanHpa
 * \brief  Turns WLAN device power on
 * 
 * \note   
 */ 
void TIWlanHpa::PowerOnDevice()
{
    TRACE(("PowerOnDevice\n")); 

    /* Hook to IRQ line */
    SetupIrqHw(iConfig.iIsrPolarity);
	    TRACE(("PowerOnDevice 2\n")); 

    /* Hook to Reset (PM Enable) line */
    SetupRstHw();
    TRACE(("PowerOnDevice 3\n")); 

	TOmap::ModifyRegister32(KHwBaseSysCtrlReg +0x0c8,0xFFFF,0x411C);

	TOmap::ModifyRegister32(KHwBaseSysCtrlReg +0xA0C,0xFFFF,0x104);

    /* Activate PM Enable line */
    ExitReset();
}

/** 
 * \fn     TIWlanHpa
 * \brief  Turns WLAN devices power off
 * 
 * \note   
 */ 
void TIWlanHpa::PowerOffDevice()
{
    TRACE(("%s\n",__FUNCTION__));

    /* Unbind ISR and disable notification */
    KillInterrupts();

	TOmap::ModifyRegister32(KHwBaseSysCtrlReg +0x0c8,0xFFFF,0x10C);

	TOmap::ModifyRegister32(KHwBaseSysCtrlReg +0xA0C,0xFFFF,0x10C);

    /* Deactivate PM Enable */
    EnterReset();

    /* Release reset & IRQ pins */
    GPIO::SetPinMode(RST_GPIO_NUM, GPIO::EDisabled);

    /* cancel any pending interrupts (i.e DFC was scheduled before calling PowerOffDevice() ) */
    iIrqDfc.Cancel();
}

/** 
 * \fn     InterruptServiced
 * \brief  
 * 
 * Called by hpa layer object client when peripheral interrupt
 * informed by OnInterrupt method has been serviced.
 * NOTE: on systems implementing level sensitive interrupt handling
 * host interrupt is enabled at this state
 *
 * \note   
 */     
void TIWlanHpa::InterruptServiced()
{
    Enable();
}

/** 
 * \fn     Configure
 * \brief  configure polarity of IRQ
 * 
 * \note   
 * \param aWlanHpa::TConfig
 */ 
void TIWlanHpa::Configure( const WlanHpa::TConfig& aConfig )
{    
		TRACE(("Configure"));

    /* Save configuration of polarity, to be used in SetupIrqHw() */
    iConfig = aConfig;
}

/** 
 * \fn     Configure
 * \brief  configure polarity of IRQ
 * 
 * \note   
 * \param aWlanHpa::TConfig
 */ 
void TIWlanHpa::EnableIrq()
{
	TRACE(("EnableIrq"));

    Enable();
}

/** 
 * \fn     DebugGpioToggle
 * \brief 
 * 
 * \note   
 * \param 
 */ 
TBool TIWlanHpa::DebugGpioToggle( TBool aHigh ){ return 0;}
/** 
 * \fn     ExitReset
 * \brief  Sets WLAN PMEN line.
 * 
 * \note   
 * \param  
 */ 
void TIWlanHpa::ExitReset()
{
    /* Set GPIO driver output to logic High */
    GPIO::SetOutputState(RST_GPIO_NUM, GPIO::EHigh);
}

/** 
 * \fn     EnterReset
 * \brief  Clears WLAN PMEN line.
 * 
 * \note   
 * \param  
 */ 
void TIWlanHpa::EnterReset()
{
    /* Set GPIO driver output to logic Low */
    GPIO::SetOutputState(RST_GPIO_NUM, GPIO::ELow);
}

/** 
 * \fn     Disable
 * \brief  Disables WLAN interrupt notification.
 * 
 * \note   
 * \param  aOsaExt 
 */ 
void TIWlanHpa::Disable()
{
    GPIO::DisableInterrupt(IRQ_GPIO_NUM);
}

/** 
 * \fn     Enable
 * \brief  Enables WLAN interrupt notification.
 * 
 * \note   
 * \param  
 */ 
void TIWlanHpa::Enable()
{
    GPIO::EnableInterrupt(IRQ_GPIO_NUM);
}

/** 
 * \fn     SetupIrqHw
 * \brief  set IRQ line
 * 
 * Sets GPIO as input line, binds it to an interrupt. Static function
 * Isr() is bound to the interrupt service routine. 
 *
 * Uses following defines:
 *
 *	#define IRQ_GPIO_NUM	61 
 *	#define IRQ_MOD_NUM		0
 * 
 * of which IRQ_GPIO_NUM is the GPIO number and 
 * IRQ_MOD_NUM is the module number where the corresponding
 * GPIO belongs to.
 *
 * \note   
 * \param  
 */ 
void TIWlanHpa::SetupIrqHw( WlanHpa::TIsrPolarity aPolarity)
{
    Kern::Printf("SetupIrqHw %d", IRQ_GPIO_NUM);

   /*--------------------------------------------------------*/
   /* common code to init the irq pin                        */
   /*--------------------------------------------------------*/
    TInt r = GPIO::SetPinMode(IRQ_GPIO_NUM, GPIO::EEnabled);
    if (r != KErrNone)
    {
        ERROR_TRACE(("ERROR:cant grab wlan interrupt pin"))
        Assert( __FILE__, __LINE__, EFalse );
    }
	GPIO::SetPinDirection(IRQ_GPIO_NUM, GPIO::EInput);
	GPIO::ClearInterrupt(IRQ_GPIO_NUM);
	GPIO::DisableInterrupt(IRQ_GPIO_NUM);
	GPIO::DisableWakeup(IRQ_GPIO_NUM);

    r = GPIO::BindInterrupt (IRQ_GPIO_NUM, Isr, this);
    if (r != KErrNone)
    {
        ERROR_TRACE(("ERROR:cant bind wlan interrupt to pin"))
        Assert( __FILE__, __LINE__, EFalse );
    }

    /* #warning "IRQ type is EDGE" */
    if( WlanHpa::EIsrPolarityHigh == aPolarity)
        /* Configure the Intr on Rising edge sensitive */
        GPIO::SetInterruptTrigger(IRQ_GPIO_NUM, GPIO::EEdgeRising);
    else
        /* Configure the Intr on Falling edge sensitive */
        GPIO::SetInterruptTrigger(IRQ_GPIO_NUM, GPIO::EEdgeFalling);
#if 0
    #warning "IRQ type is LEVEL"
    if( WlanHpa::EIsrPolarityHigh == aPolarity)
        /* Configure the Intr on Rising leve sensitive */
        GPIO::SetInterruptTrigger(IRQ_GPIO_NUM, GPIO::ELevelHigh);
    else
        /* Configure the Intr on Falling leve sensitive */
        GPIO::SetInterruptTrigger(IRQ_GPIO_NUM, GPIO::ELevelLow);
#endif
    Disable(); // disable int until required
}
	
/** 
 * \fn     SetupRstHw
 * \brief  set reset line.
 * 
 * Sets GPIO as output line. Used for controlling the PMEN signal of the WLAN chip.
 * #define RST_GPIO_NUM	3 
 * #define RST_MOD_NUM		0
 *
 * of which RST_GPIO_NUM is the GPIO number and 
 * RST_MOD_NUM is the module number where the corresponding
 * GPIO belongs to.
 * \note   
 * \param  
 */ 
void TIWlanHpa::SetupRstHw()
{
    Kern::Printf("SetupRstHw %d", RST_GPIO_NUM);

    /*--------------------------------------------------------*/
    /* common code to init the rst pin                        */
    /*--------------------------------------------------------*/
    /* Get control of GPIO driver line */
    TInt r = GPIO::SetPinMode(RST_GPIO_NUM, GPIO::EEnabled);
    if (r != KErrNone)
    {
        ERROR_TRACE(("ERROR:cant grab wlan enable pin"))
        Assert( __FILE__, __LINE__, EFalse );
    }

    GPIO::SetPinDirection(RST_GPIO_NUM, GPIO::EOutput);

    EnterReset();
}

/** 
 * \fn     Isr
 * \brief  Interrupt handler function. 
 *
 * \note   
 * \param aPtr HPA object context.
 * \param aPinHandle is not used. 
 */ 
void TIWlanHpa::Isr( TAny* aPtr)
{
    TIWlanHpa* p = static_cast<TIWlanHpa*>( aPtr );

       // Ack and Disable the interrupt
	GPIO::ClearInterrupt(IRQ_GPIO_NUM);
	GPIO::DisableInterrupt(IRQ_GPIO_NUM);
	GPIO::DisableWakeup(IRQ_GPIO_NUM);

    p->Disable();

    p->InterruptHandlerStartDfc();
}

/** 
 * \fn     InterruptHandlerStartDfc
 * \brief  Start the DFC queue
 * 
 * \note   
 * \param  
 */ 
 
void TIWlanHpa::InterruptHandlerStartDfc()
{
    iIrqDfc.Add();
}

/** 
 * \fn     InterruptHandlerDfc
 * 
 * DFC - function called by a DFC to handle the interrupt
 * call another function to get in the correct class and
 * therefore don't need me-> all the time
 * Doesn't always work having me-> for the C software
 * \note   
 * \param  
 */ 
void TIWlanHpa::InterruptHandlerDfc(TAny* aPtr)
{
    TIWlanHpa* me = (TIWlanHpa*)aPtr;
    me->InterruptHandler();
}

 /** 
 * \fn     InterruptHandler
 * \brief  Call the actual interrupt handling routine and
 *
 * \note   
 * \param  
 */ 
void TIWlanHpa::InterruptHandler()
{
    OsaExtCb().MutexAcquire();
    HpaCb().OnInterrupt();
    OsaExtCb().MutexRelease();
}

 /** 
 * \fn     KillInterrupts
 * \brief  Call the actual interrupt handling routine and
 *
 *  Disables interrupts and 
 *  unbinds the interrupt service routine.
 *
 * \note   
 * \param  
 */ 
void TIWlanHpa::KillInterrupts()
{
    GPIO::ClearInterrupt(IRQ_GPIO_NUM);
	GPIO::DisableInterrupt(IRQ_GPIO_NUM);
	GPIO::DisableWakeup(IRQ_GPIO_NUM);
	GPIO::UnbindInterrupt(IRQ_GPIO_NUM);
	GPIO::SetPinMode(IRQ_GPIO_NUM, GPIO::EDisabled);
}

