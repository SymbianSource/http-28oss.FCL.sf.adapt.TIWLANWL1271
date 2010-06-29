/*
 * osapi.cpp
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



/** \file  osapi.cpp 
 *  \brief  Interface between the generic os abstration to symbian OSA
 *
 *  \see   
 */

#include "TIWhaDef.h"
#include "timerclient.h"


extern "C" 
{
#include "tidef.h"
#include "cdebugoutput.h"
#include "osApi.h"
#define __FILE_ID__								FILE_ID_138

/****************************************************************************************
 *						OS Print API					                *       
 ****************************************************************************************/

/** 
 * \fn     os_setDebugMode
 * \brief  not in use
 * 
 * \note   
 * \param  
 * \return 
 * \sa     
 */ 
void os_setDebugMode (TI_BOOL enable) 
{
	
}


/** 
 * \fn     os_printf
 * \brief  Print formatted output
 * 
 * \note   
 * \param  format -  Specifies the string, to be printed
 * \return 
 * \sa     
 */ 
#include <libc/stdarg.h>
extern "C" {void wlanPrintf(const char* const pPrefix, const char* const pFormat, va_list args);}
void os_printf (const char *format ,...) 
{
	#if _DEBUG
		VA_LIST args;
		VA_START(args, format);
        wlanPrintf("TI_TRC: ", format, args);
		VA_END(args);
	#endif
}

/** 
 * \fn     os_InterruptServiced
 * \brief   inform that interrupt was serviced (i.e. enable WLAN IRQ in LEVEL_IRQ)
 * 
 * \param  OsContext
 * \return 
 * \sa     
 */ 
void os_InterruptServiced (TI_HANDLE OsContext)
{
    /* Mark that interrupt was serviced */        
    GET_HPA(OsContext)->InterruptServiced();
}

/****************************************************************************************
 *						OS Memory API													*       
 ****************************************************************************************/

/** 
 * \fn     os_memoryAlloc
 * \brief  Allocates resident (nonpaged) system-space memory
 * 
 * \note   
 * \param  OsContext	- os context.
 * \param Size		- Specifies the size, in bytes, to be allocated
 * \return Pointer to the allocated memory.
				NULL if there is insufficient memory available
 * \sa     
 */ 
void *os_memoryAlloc (TI_HANDLE OsContext,TI_UINT32 Size,TI_BOOL MemoryType) 
{
#if defined DMA_CAPABLE_BUFFERS
	MWlanOsa::TOsaMemoryType iOsMemType;
    if (MemoryType == MemoryNormal) 
    {
        iOsMemType = MWlanOsa::ENormalMemory;
    }
    else
    {
		iOsMemType = MWlanOsa::EInterconnectMemory;
    }
	return GET_OSA(OsContext)->Alloc(iOsMemType,Size,0);
#else
	MWlanOsa::TOsaMemoryType iOsMemType = MWlanOsa::ENormalMemory;
	return GET_OSA(OsContext)->Alloc(iOsMemType,Size,0);
#endif

}


/** 
 * \fn     os_memorySet
 * \brief  This function fills a block of memory with given value
 * 
 * \note   
 * \param OsContext		- os context.
 * \param pMemPtr		- Specifies the base address of a block of memory
 * \param Value			- Specifies the value to set
 * \param Length		- Specifies the size, in bytes, to copy.
 * \return 
 * \sa     
 */ 
void os_memorySet (TI_HANDLE OsContext, void *pMemPtr, TI_INT32 Value, TI_UINT32 Length) 
{
	MWlanOsa::MemSet(pMemPtr,Value,Length);
}


/** 
 * \fn     os_memoryZero
 * \brief  This function fills a block of memory with 0s.
 * 
 * \note   
 * \param OsContext		- os context.
 * \param pMemPtr		- Specifies the base address of a block of memory
 * \param Length		- Specifies how many bytes to fill with 0s.
 * \return 
 * \sa     
 */ 
void os_memoryZero (TI_HANDLE OsContext, void *pMemPtr, TI_UINT32 Length) 
{
	MWlanOsa::MemClr(pMemPtr,Length);
}


/** 
 * \fn     os_memoryCopy
 * \brief  This function copies a specified number of bytes from one caller-supplied
				location to another.
 * 
 * \note   
 * \param OsContext		- os context.
 * \param pDstPtr		- Destination buffer
 * \param pSrcPtr		- Source buffer
 * \param Size			- Specifies the size, in bytes, to copy.
 * \return 
 * \sa     
 */ 
void os_memoryCopy (TI_HANDLE pOsContext, void *pDestination, void *pSource, TI_UINT32 Size)
{
	MWlanOsa::MemCpy(pDestination,(const void *)pSource,Size);
}


/** 
 * \fn     os_memoryFree
 * \brief  This function releases a block of memory previously allocated with the
				os_memoryAlloc function.
 * 
 * \note   
 * \param OsContext		- os context.
 * \param pMemPtr		-	Pointer to the base virtual address of the allocated memory.
								This address was returned by the os_memoryAlloc function.
 * \param Size		-	Specifies the size, in bytes, of the memory block to be released.
						This parameter must be identical to the Length that was passed to
								os_memoryAlloc.
 * \return 
 * \sa     
 */ 
void os_memoryFree (TI_HANDLE pOsContext, void *pMemPtr, TI_UINT32 Size) 
{
	GET_OSA(pOsContext)->Free(pMemPtr);
}

/** 
 * \fn     os_memoryCompare
 * 
 * \note   
 * \return 
 * \sa     
 */ 
TI_INT32 os_memoryCompare (TI_HANDLE OsContext, TI_UINT8* Buf1, TI_UINT8* Buf2, TI_INT32 Count)
{
    return (TI_INT32)(MWlanOsa::MemCmp( Buf1, Buf2, Count ));
    
}


/** 
 * \fn     os_timeStampMs
 * \brief  This function returns the number of milliseconds that have elapsed since
				the system was booted.
 * 
 * \note   
 * \param OsContext		- os context.
 * \return
 * \sa     
 */ 
TI_INT32 os_timeStampMs (TI_HANDLE OsContext) 
{
    TInt64  iTime;

    iTime = MWlanOsa::Time() / 1000;

    /* return only LSB */
    return ((TUint32*)&iTime)[0];
}




/** 
 * \fn     os_StalluSec
 * \brief  This function make delay in microseconds.
 * 
 * \note   
 * \param OsContext		- os context.
 * \param uSec			- delay time in microseconds
 * \return
 * \sa     
 */ 
void os_StalluSec (TI_HANDLE OsContext, TI_UINT32 uSec) 
{
	MWlanOsa::BusyWait(uSec);
}

/*****************************************************************************************
 *							Timer functions				       *
 *****************************************************************************************/

/** 
 * \fn     os_timerCreate
 * \brief  Create timer client 
 * 
 * \param OsContext		- os context.
 * \param pRoutine			- CB function
 * \param hFuncHandle		- CB handle
 *
 * \note   
 * \return Timer client handle
 * \sa     
 */ 
TI_HANDLE os_timerCreate (TI_HANDLE OsContext, fTimerFunction pRoutine, TI_HANDLE hFuncHandle)
{
    return (TI_HANDLE)(new TimerClient (OsContext, pRoutine, hFuncHandle));
    
}

/** 
 * \fn     os_timerDestroy
 * \brief  Destroy timer client 
 * 
 * \param OsContext		- os context.
 * \param TimerHandle			- timer client handle
 * \note   
 * \return
 * \sa     
 */ 
void os_timerDestroy (TI_HANDLE OsContext, TI_HANDLE TimerHandle)
{
    if (TimerHandle) 
    {
        delete (TimerClient *)TimerHandle;
    }
}

/** 
 * \fn     os_timerStart
 * \brief  start timer 
 * 
 * \param OsContext		- os context.
 * \param TimerHandle		- timer client handle
 * \param DelayMs			
 *
 * \note   
 * \return
 * \sa     
 */ 
void os_timerStart (TI_HANDLE OsContext, TI_HANDLE TimerHandle, TI_UINT32 DelayMs)
{
    ((TimerClient *)TimerHandle)->Start(DelayMs);
}

/** 
 * \fn     os_timerStop
 * \brief  stop timer 
 * 
 * \param OsContext		- os context.
 * \param TimerHandle		- timer client handle
 * \note   
 * \return
 * \sa     
 */ 
void os_timerStop (TI_HANDLE OsContext, TI_HANDLE TimerHandle)
{
    ((TimerClient *)TimerHandle)->Stop();
}


/****************************************************************************************
 *							Protection services	API										*
 ****************************************************************************************/

/** 
 * \fn     os_protectCreate
 * \brief  
 * 
 * \note   
 * \param OsContext		- os context.
 * \return A handle of the created mutex/spinlock.
				TI_HANDLE_INVALID if there is insufficient memory available or problems
				initializing the mutex
 * \sa     
 */ 
TI_HANDLE os_protectCreate (TI_HANDLE OsContext) { return TI_OK;}


/** 
 * \fn     os_protectDestroy
 * \brief  the WVSS dose not use protection 
 * 
 * \note   
 * \param OsContext		- os context.
 * \return 
 * \sa     
 */ 
void os_protectDestroy (TI_HANDLE OsContext, TI_HANDLE ProtectContext) {}

} //extern "C"

