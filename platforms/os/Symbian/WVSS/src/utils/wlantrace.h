/*
 * wlantrace.h
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


	void wlanTrace(const char* const pFormat, ...);
	void wlanTraceLine(TUint moduleId, ...);
	void wlanTraceEnterFn(TUint moduleId, ...);
	void wlanTraceExitFn(TUint moduleId, ...);
	void wlanTraceProcess(TUint moduleId, ...);
	void wlanTraceModuleSet(TUint moduleId);
	void wlanTraceData(TUint moduleId, const char* const pString, const TUint8* const pData, TUint dataLength);
	void wlanTraceAscii(TUint moduleId, const char* const pString, const TUint8* pBuffer, TUint length);
	extern TUint gModuleId;

//wlanTraceEnterFn(MODULE_ID, __FUNCTION__);
//wlanTraceExitFn(MODULE_ID, __FUNCTION__);


/* per file basis */
#if defined(WLAN_TRACE_ENABLED) && (WLAN_TRACE_ENABLED==TRACE_ENABLED)

	#define TRACE(msg) 									{ gModuleId = MODULE_ID; wlanTrace msg; }
	#define LINE_TRACE() 								wlanTraceLine(MODULE_ID, __FUNCTION__, __LINE__);
	#define ENTER_FN() 									
	#define EXIT_FN() 									
	#define DATA_TRACE(string, data, length) 		wlanTraceData(MODULE_ID, string, (TUint8*)data, length);
	#define ASCII_TRACE(string, data, length) 	wlanTraceAscii(MODULE_ID, string, (TUint8*)data, length);
#else
	#ifndef WLAN_TRACE_ENABLED
		#error "WLAN_TRACE_ENABLED not defined"
	#endif
	
	#define TRACE(msg) 		;
	#define LINE_TRACE()
	#define ENTER_FN()
	#define EXIT_FN()
	#define DATA_TRACE(string, data, length)
	#define ASCII_TRACE(string, data, length)
#endif

/* globally enabled - not per file */
#ifndef IRQ_TRACE_ENABLED
	#error "IRQ_TRACE_ENABLED not defined"
#else
	#if (IRQ_TRACE_ENABLED==TRACE_ENABLED)
		#define IRQ_TRACE(msg) 		{ gModuleId = MODULE_ID; wlanTrace msg; }
		#define ENTER_IRQ_FN()		wlanTraceEnterFn(MODULE_ID, __FUNCTION__);
		#define EXIT_IRQ_FN()		wlanTraceExitFn(MODULE_ID, __FUNCTION__);
	#else
		#define IRQ_TRACE(msg) 		;
		#define ENTER_IRQ_FN()
		#define EXIT_IRQ_FN()
	#endif
#endif

#ifndef MEMORY_TRACE_ENABLED
	#error "MEMORY_TRACE_ENABLED not defined"
#else
	#if (MEMORY_TRACE_ENABLED==TRACE_ENABLED)
		#define MEMORY_TRACE(msg) 		{ gModuleId = MODULE_ID; wlanTrace msg; }
		#define ENTER_MEMORY_FN()		wlanTraceEnterFn(MODULE_ID, __FUNCTION__);
		#define EXIT_MEMORY_FN()		wlanTraceExitFn(MODULE_ID, __FUNCTION__);
	#else
		#define MEMORY_TRACE(msg) 		;
		#define ENTER_MEMORY_FN()
		#define EXIT_MEMORY_FN()
	#endif
#endif

#ifndef TIMER_TRACE_ENABLED
	#error "TIMER_TRACE_ENABLED not defined"
#else
	#if (TIMER_TRACE_ENABLED==TRACE_ENABLED)
		#define TIMER_TRACE(msg) 		{ gModuleId = MODULE_ID; wlanTrace msg; }
		#define ENTER_TIMER_FN()		wlanTraceEnterFn(MODULE_ID, __FUNCTION__);
		#define EXIT_TIMER_FN()			wlanTraceExitFn(MODULE_ID, __FUNCTION__);
	#else
		#define TIMER_TRACE(msg) 		;
		#define ENTER_TIMER_FN()
		#define EXIT_TIMER_FN()
	#endif
#endif

#ifndef ERROR_TRACE_ENABLED
	#error "ERROR_TRACE_ENABLED not defined"
#else
	#if (ERROR_TRACE_ENABLED==TRACE_ENABLED)
		#define ERROR_TRACE(msg) 		{ gModuleId = MODULE_ID; wlanTrace msg; }
	#else
		#define ERROR_TRACE(msg) 		;
	#endif
#endif

#ifndef PROCESS_TRACE_ENABLED
	#error "PROCESS_TRACE_ENABLED not defined"
#else
	#if (PROCESS_TRACE_ENABLED==TRACE_ENABLED)
		#define PROCESS_TRACE() 	wlanTraceProcess(MODULE_ID, __FUNCTION__, (TUint32)NKern::CurrentThread() );
	#else
		#define PROCESS_TRACE()
	#endif
#endif

#ifndef SPI_DATA_TRACE_ENABLED
	#error "SPI_DATA_TRACE_ENABLED not defined"
#else
	#if (SPI_DATA_TRACE_ENABLED==TRACE_ENABLED)
		#define SPI_DATA_TRACE(string, data, length) 	wlanTraceData(MODULE_ID, string, (TUint8*)data, length);
	#else
		#define SPI_DATA_TRACE(string, data, length)
	#endif
#endif

#ifndef PACKET_DATA_TRACE_ENABLED
	#error "PACKET_DATA_TRACE_ENABLED not defined"
#else
	#if (PACKET_DATA_TRACE_ENABLED==TRACE_ENABLED)
		#define PACKET_TRACE(string, data, length) 	wlanTraceData(MODULE_ID, string, (TUint8*)data, length);
	#else
		#define PACKET_TRACE(string, data, length)
	#endif
#endif
















