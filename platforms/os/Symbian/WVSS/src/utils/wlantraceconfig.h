/*
 * wlantraceconfig.h
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

/* order matches that in CDebugOutput.cpp */
#define WLAN_MODULE_ID_WLAN_OSA			 			0
#define WLAN_MODULE_ID_SPI_CLIENT 				1
#define WLAN_MODULE_ID_WLAN_WHA				 		2
#define WLAN_MODULE_ID_SPIA	 					3
#define WLAN_MODULE_ID_CONTROL 					4
#define WLAN_MODULE_ID_WLAN_HPA                  5


#define TRACE_ENABLED 	1
#define TRACE_DISABLED 	2


#ifdef _DEBUG
	/* UDEB */
	/* enable all TI trace - check MACROs in wlanpdd.mmh */
	#if defined(REPORT_LOG) && defined(TI_DBG)
		#define WLAN_TI_TRACE_ENABLED 			TRACE_ENABLED
	#else
		#define WLAN_TI_TRACE_ENABLED 			TRACE_DISABLED //trace is disabled because it is not enabled in wlanpdd.mmh
	#endif

	/* file specific */
	#define WLAN_OSA				 			TRACE_DISABLED
	#define WLAN_SPI_CLIENT 					TRACE_DISABLED
	#define WLAN_WHA				 			TRACE_DISABLED
	#define WLAN_SPIA 							TRACE_DISABLED
	#define WLAN_CONTROL 						TRACE_DISABLED
    #define WLAN_HPA 						    TRACE_DISABLED
	
	/* action specific */
	#define PROCESS_TRACE_ENABLED 			TRACE_DISABLED
	#define IRQ_TRACE_ENABLED 				TRACE_DISABLED
	#define PACKET_DATA_TRACE_ENABLED 		TRACE_DISABLED
	#define SPI_DATA_TRACE_ENABLED 			TRACE_DISABLED
	#define TIMER_TRACE_ENABLED 				TRACE_DISABLED
	#define MEMORY_TRACE_ENABLED 				TRACE_DISABLED

	/* should always be enabled - to show any errors */
	#define ERROR_TRACE_ENABLED 				TRACE_ENABLED

#else
	/* UREL */
	/* Must be disbled in urel build - will not compile if it is not!!! */
	#define WLAN_TI_TRACE_ENABLED 			TRACE_DISABLED
	#define WLAN_OSA				 			TRACE_DISABLED
	#define WLAN_SPI_CLIENT 					TRACE_DISABLED
    #define WLAN_WHA 							TRACE_DISABLED
    #define WLAN_SPIA 							TRACE_DISABLED
	#define WLAN_CONTROL 						TRACE_DISABLED
    #define WLAN_HPA 						    TRACE_DISABLED

	#define PROCESS_TRACE_ENABLED 			TRACE_DISABLED
	#define IRQ_TRACE_ENABLED 					TRACE_DISABLED
	#define PACKET_DATA_TRACE_ENABLED 		TRACE_DISABLED
	#define SPI_DATA_TRACE_ENABLED 			TRACE_DISABLED
	#define TIMER_TRACE_ENABLED 				TRACE_DISABLED
	#define MEMORY_TRACE_ENABLED 				TRACE_DISABLED
	#define ERROR_TRACE_ENABLED 				TRACE_DISABLED

#endif
