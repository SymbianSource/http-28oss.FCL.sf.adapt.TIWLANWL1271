/*
 * GenSM.h
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

/** \file GenSM.h
 *  \brief Generic state machine declarations
 *
 *  \see GenSM.c
 */


#ifndef __GENSM_H__
#define __GENSM_H__

#include "tidef.h"

/* action function type definition */
typedef void (*TGenSM_action) (void *pData);


/* State/Event cell */
typedef  struct
{
    TI_UINT32       uNextState; /**< next state in transition */
    TGenSM_action   fAction;    /**< action function */
} TGenSM_actionCell;



/* 
 * matrix type 
 * Although the state-machine matrix is actually a two-dimensional array, it is treated as a single 
 * dimension array, since the size of each dimeansion is only known in run-time
 */
typedef TGenSM_actionCell *TGenSM_matrix;

TI_HANDLE   genSM_Create (TI_HANDLE hOS);
void        genSM_Unload (TI_HANDLE hGenSM);
void        genSM_Init (TI_HANDLE hGenSM, TI_HANDLE hReport);
void        genSM_SetDefaults (TI_HANDLE hGenSM, TI_UINT32 uStateNum, TI_UINT32 uEventNum,
                        TGenSM_matrix pMatrix, TI_UINT32 uInitialState, TI_INT8 *pGenSMName, 
                        TI_INT8 **pStateDesc, TI_INT8 **pEventDesc, TI_UINT32 uModuleLogIndex);
void        genSM_Event (TI_HANDLE hGenSM, TI_UINT32 uEvent, void *pData);
TI_UINT32   genSM_GetCurrentState (TI_HANDLE hGenSM);

#endif /* __GENSM_H__ */

