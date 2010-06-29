/*
 * TIWhaUtils.h
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



/** \file  TIWhaUtils.h 
 *  \brief  Utilities used by TIWha layer only
 *
 *  \see   
 */

#ifndef TI_WHA_UITLS_H
#define TI_WHA_UITLS_H

#include "TIWhaDef.h" 

/**
 * TIWhaUtils class
 */
class TIWhaUtils
{
public:  
	static EMibTemplateType WhaToTwdFrameType(WHA::TTemplateType aFrameType); 
	static ERate WhaToTwdRate (WHA::TRate aRate);
	static WHA::TRate PolicyToWhaRate (ETxRateClassId aRate);
	static TI_UINT32		WhaToMaskRate (WHA::TRate aRate);
    static TI_UINT32		WhaRateToRatePolicy(WHA::StxAutoRatePolicy* pTxAutoRatePolicy);
#ifdef HT_SUPPORT
	static TI_UINT32		HTWhaRateToRatePolicy(WHA::StxAutoRatePolicy* pTxAutoRatePolicy);
	static TI_UINT32  		HTWhaToMaskRate (WHA::TRate aRate,WHA::THtMcsSet	aMcsRates);
#endif /* HT_SUPPORT */
};


/* Align header to data: HEADER - OFFSET - DATA --> HEADER - DATA                            */
/* pOriginalFrame points to the original place. It will change to point offset bytes ahead   */
#define ALIGN_HEADER_TO_DATA_FORWARD(pOriginalFrame, uOffset, bQos) \
{   \
    TI_INT32 i;    \
    TI_UINT16 *pNewFrame = ((TI_UINT16*)pOriginalFrame) + (uOffset / sizeof(TI_UINT16)); \
    if (bQos)   \
    {   \
        *((TI_UINT16*)pNewFrame + (WLAN_HDR_LEN / sizeof(TI_UINT16))) =    \
        *((TI_UINT16*)pOriginalFrame + (WLAN_HDR_LEN / sizeof(TI_UINT16)));   \
    }   \
    for (i = ((WLAN_HDR_LEN / sizeof(TI_UINT16)) - 1) ; i >= 0 ; i--) \
    {   \
		pNewFrame[i] = *((TI_UINT16*)pOriginalFrame + i); \
    }   \
    pOriginalFrame = pNewFrame; \
}

/* Align header to data: HEADER - DATA --> HEADER - OFFSET - DATA                            */
/* pOriginalFrame points to the original place. It will change to point offset bytes before  */
#define ALIGN_HEADER_TO_DATA_BACKWARD(pOriginalFrame, uOffset, bQos) \
{   \
    TI_INT32 i;    \
    TI_UINT16 *pNewFrame = ((TI_UINT16*)pOriginalFrame) - (uOffset / sizeof(TI_UINT16));  \
    for (i = 0 ; i < WLAN_HDR_LEN / sizeof(TI_UINT16) ; i++)    \
    {   \
        pNewFrame[i] = *((TI_UINT16*)pOriginalFrame + i); \
    }   \
    if (bQos)   \
    {   \
        *((TI_UINT16*)pNewFrame + (WLAN_HDR_LEN / sizeof(TI_UINT16))) =    \
        *((TI_UINT16*)pOriginalFrame + (WLAN_HDR_LEN / sizeof(TI_UINT16)));   \
    }   \
    pOriginalFrame = pNewFrame; \
}


#define CONVERT_TU_2_MICRO(tu)               (tu) * 1024

/* 
* Small macro to convert Dbm units into Dbm/10 units. This macro is important
* in order to avoid over-flow of Dbm units bigger than 25
*/

#define DBM2DBMDIV10(uTxPower) \
	((uTxPower) > (MAX_TX_POWER / DBM_TO_TX_POWER_FACTOR) ? \
		MAX_TX_POWER : (uTxPower) * DBM_TO_TX_POWER_FACTOR)		




#endif /* TI_WHA_UITLS_H */

