/*
 * sdioadapter.cpp
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

 
/** \file   SdioAdapter.c 
 *  \brief  The SDIO driver adapter. Platform dependent. 
 * 
 * An adaptation layer between the lower SDIO driver (in BSP) and the upper SdioBusDrv.
 * Used for issuing all SDIO transaction types towards the lower SDIO-driver.
 * Makes the decision whether to use Sync or Async transaction, and reflects it to the caller
 *     by the return value and calling its callback in case of Async.
 *  
 *  \see    SdioAdapter.h, SdioDrv.c & h
 */

#include "TxnDefs.h"
extern "C" 
{
#include "SdioAdapter.h"
}

#include "SdioDrv.h"
#include <stdio.h>
#include "SDioClient.h"

void* wsdioAdapt_Open(void* hOs)
{
	return (void*)SDioClient::Create(); 
	
}

void* wSdioAdapt_Close(void* hOs)
{
    return KErrNone;
}

/**
  * \fn     wsdioAdapt_ConnectBus 
  * \brief   
  * 
  * Initialize HW.
  * 
  * \note   
  * \param  
  * \return KErrNone if success otherwise error.
  * \sa     
  */
int wsdioAdapt_ConnectBus(TAny*		   hSdio,	 
						  TAny*        fCbFunc,
                          TAny*        hCbArg,
                          unsigned int  uBlkSizeShift,
                          unsigned int  uSdioThreadPriority)
{
	SDioClient* pSdioClient = reinterpret_cast<SDioClient*>(hSdio);
   
	pSdioClient->sdioAdapt_ConnectBus(fCbFunc, hCbArg, uBlkSizeShift, uSdioThreadPriority);
	return KErrNone;
}

/**
  * \fn     wsdioAdapt_DisconnectBus 
  * \brief   
  * 
  * DeInitialize HW.
  * 
  * \note   
  * \param  
  * \return KErrNone if success otherwise error.
  * \sa     
  */
int wsdioAdapt_DisconnectBus (TAny*		   hSdio)
{
	SDioClient* pSdioClient = reinterpret_cast<SDioClient*>(hSdio);
    pSdioClient->sdioAdapt_DisconnectBus();
	return KErrNone;
}

/**
  * \fn     wsdioAdapt_Transact 
  * \brief   
  * 
  * Transaction of multiple bytes.
  * 
  * \note   
  * \param  
  * \return TXN_STATUS_OK if success, 
  *         TXN_STATUS_ERROR if error.
  * \sa     
  */
ETxnStatus wsdioAdapt_Transact (TAny*		   hSdio,	
							    unsigned int  uFuncId,
                                unsigned int  uHwAddr,
                                void *        pHostAddr,
                                unsigned int  uLength,
                                unsigned int  bDirection,
                                unsigned int  bBlkMode,
                                unsigned int  bFixedAddr,
                                unsigned int  bMore)
{
	ETxnStatus iStatus = TXN_STATUS_OK;
	SDioClient* pSdioClient = reinterpret_cast<SDioClient*>(hSdio);
    iStatus = pSdioClient->sdioAdapt_Transact(uFuncId, uHwAddr, pHostAddr, uLength, bDirection, bBlkMode, bFixedAddr, bMore );

	if(iStatus == TXN_STATUS_ERROR)
		return TXN_STATUS_ERROR;
	else
		return iStatus;
}
         
/**
  * \fn     wsdioAdapt_TransactBytes 
  * \brief   
  * 
  * Transaction of bytes.
  * 
  * \note   
  * \param  
  * \return TXN_STATUS_OK if success, 
  *         TXN_STATUS_ERROR if error.
  * \sa     
  */		 
ETxnStatus wsdioAdapt_TransactBytes (TAny*		   hSdio,	
									unsigned int  uFuncId,
                                    unsigned int  uHwAddr,
                                    void *        pHostAddr,
                                    unsigned int  uLength,
                                    unsigned int  bDirection,
                                    unsigned int  bMore)
{
	int iStatus = TXN_STATUS_OK;

	SDioClient* pSdioClient = reinterpret_cast<SDioClient*>(hSdio);
	iStatus = pSdioClient->sdioAdapt_TransactBytes(uFuncId, uHwAddr, pHostAddr, uLength, bDirection, bMore);

	if(iStatus == TXN_STATUS_ERROR)
		return TXN_STATUS_ERROR;
	else
		return TXN_STATUS_OK;
}
