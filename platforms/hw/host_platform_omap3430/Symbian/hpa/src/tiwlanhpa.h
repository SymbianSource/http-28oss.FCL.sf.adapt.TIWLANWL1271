/*
 * tiwlanhpa.h
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



/** \file  TIWlanHpa.h
 *  \brief  Interface between the sandbox to the BSP of Omap3430
 *
 *  \see   
 */

#ifndef _TIWLANHPA_
#define _TIWLANHPA_

#include <silicon_assp.h>
#include <gpio.h>
#include <assp.h>

#include "wlanhpa.h"

#ifdef WLAN_SDIO
#define RST_GPIO_NUM	101
#define IRQ_GPIO_NUM	162
#else //WLAN_SPI
#define RST_GPIO_NUM	3  
#define IRQ_GPIO_NUM	61 
#endif //WLAN_SDIO

#define RST_MOD_NUM		0	
#define IRQ_MOD_NUM		0

#define ASIC_CLASS					TOmap
#define ASIC_CLASS_PLAT				TOmapPlat	

#define PIN_TO_HANDLE OMAPGPIO_GPINID_TO_PHANDLE
#define INVALID_HANDLE 				KOmapGpioInvalidHandle


class TIWlanHpa : public WlanHpa
{
public:

	/**
	 * Costructor.
	 */
	TIWlanHpa(MWlanOsaExt& aOsaExt, MWlanOsa& aOsa);

	/**
	 * Destructor
	 */
	virtual ~TIWlanHpa();

	/**
	 * Turns WLAN devices power on
	 */
	virtual void PowerOnDevice();

	/**
	 * Turns WLAN devices power off
	 */
	virtual void PowerOffDevice();

	/**
	 * Called by hpa layer object client when peripheral interrupt
	 * informed by OnInterrupt method has been serviced.
	 * NOTE: on systems implementing level sensitive interrupt handling
	 * host interrupt is enabled at this state
	 */
	virtual void InterruptServiced();

	    /**
	 * Configures the HPA layer
	 */
	virtual void Configure( const WlanHpa::TConfig& aConfig );  

	/**
	 * Enables Irq
	 *
	 * NOTE: usage limited to bootup sequence only. 
	 * Untill this call the host side interrupts are disabled
	 *
	 */
	 virtual void EnableIrq();
	 
	 /**
	 * Toggles debug GPIO
	 * Usage limited to R&D (debug) purpose only
	 */
	 virtual TBool DebugGpioToggle( TBool aHigh );

private:

	/*
	* DFC - function called by a DFC to handle the interrupt
	* call another function to get in the correct class and
	* therefore don't need me-> all the time
	* Doesn't always work having me-> for the C software
	*/
	static void InterruptHandlerDfc(TAny *aPtr);

	/**
	* Start the DFC queue
	*/
	void InterruptHandlerStartDfc();

	
	/*
	* Call the actual interrupt handling routine and
	* re-enable the interrupts.
	*
	* The interrupt handler is in the third party software
	* block and performs the necessary functions
	*/
	void InterruptHandler();


	/**
	*  Disables interrupts and 
	*  unbinds the interrupt service routine.
	*/
	void KillInterrupts();

	/**
	* Sets GPIO as input line, binds it to an interrupt. Static function
	* Isr() is bound to the interrupt service routine. 
	*
	* Uses following defines:
	*
	*	#define IRQ_GPIO_NUM	9 
	*	#define IRQ_MOD_NUM		1
	* 
	* of which IRQ_GPIO_NUM is the GPIO number and 
	* IRQ_MOD_NUM is the module number where the corresponding
	* GPIO belongs to.
	*/		                   
	void SetupIrqHw( WlanHpa::TIsrPolarity aPolarity);

	/**
	* Sets GPIO as output line. Used for controlling the PMEN signal of the WLAN chip.
	#define RST_GPIO_NUM	10 
	#define RST_MOD_NUM		1

	* of which RST_GPIO_NUM is the GPIO number and 
	* RST_MOD_NUM is the module number where the corresponding
	* GPIO belongs to.

	*/		  

	void SetupRstHw			();


	/**
	* Interrupt handler function. 
	* Is bound to the ISR and handles continues serving the interrupt.
	*
	* \param aPtr HPA object context.
	* \param aPinHandle is not used. 
	*/	
	static void Isr			(TAny* aPtr);

	/**
	* Clears WLAN PMEN line.
	*
	*/
	void EnterReset();

	/**
	* Sets WLAN PMEN line.
	*
	*/
	void ExitReset();

	/**
	* Disables WLAN interrupt notification.
	*
	*/
	void Disable();

	/**
	* Enables WLAN interrupt notification.
	*
	*/
	void Enable();

private:

	TDfc    		        iIrqDfc;
	WlanHpa::TConfig		iConfig;
};



#endif //_TIWLANHPA_

