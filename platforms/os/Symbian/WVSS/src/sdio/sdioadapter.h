/*
 * sdioadapter.h
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


/** \file   SdioAdapter.h 
 *  \brief  SDIO adapter module API definition                                  
 *
 *  \see    SdioAdapter.c
 */

#ifndef __SDIO_ADAPT_API_H__
#define __SDIO_ADAPT_API_H__

#include "TxnDefs.h"
#include "e32def.h"

void* wsdioAdapt_Open(void* hOs);

void* wSdioAdapt_Close(void* hOs);

int wsdioAdapt_ConnectBus(TAny*		    hSdio,	 
						  TAny*         fCbFunc,
                          TAny*         hCbArg,
                          unsigned int  uBlkSizeShift,
                          unsigned int  uSdioThreadPriority);

int wsdioAdapt_DisconnectBus (TAny*		   hSdio);

ETxnStatus wsdioAdapt_Transact (TAny*		  hSdio,	
							    unsigned int  uFuncId,
                                unsigned int  uHwAddr,
                                void *        pHostAddr,
                                unsigned int  uLength,
                                unsigned int  bDirection,
                                unsigned int  bBlkMode,
                                unsigned int  bFixedAddr,
                                unsigned int  bMore);

ETxnStatus wsdioAdapt_TransactBytes (TAny*		  hSdio,	
									unsigned int  uFuncId,
                                    unsigned int  uHwAddr,
                                    void *        pHostAddr,
                                    unsigned int  uLength,
                                    unsigned int  bDirection,
                                    unsigned int  bMore);


#endif /*__SDIO_ADAPT_API_H__*/
