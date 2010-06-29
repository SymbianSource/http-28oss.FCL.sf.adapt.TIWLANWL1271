/*
 * TIWhaPLTDef.h
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


/** \file  TIWhaPLTDef.h 
 *  \brief  New API's - Should be located in Symbian ???
 *
 *  \see   
 */

#ifndef TIWhaPLT_DEF_H
#define TIWhaPLT_DEF_H


#define MAC_ADD_SIZE	24

#define TX_TYPE         1
#define TX_TYPE_INDEX	24
#define TX_LEN_INDEX	25
#define TX_LEN_MS       0x1
#define TX_LEN_LS       0x99
#define TX_VALUE_INDEX	27

#define RX_TYPE         2
#define RX_LEN          0x13
#define RX_TYPE_INDEX	436
#define RX_LEN_INDEX	437
#define RX_VALUE_INDEX	439

#define NVS_TYPE        0xAA
#define NVS_TYPE_INDEX  458
#define NVS_LEN         468


typedef TestCmdID_enum ETestCmdID;

struct SPLTResponse
{
	WHA::TStatus         aStatus;
	ETestCmdID        aPlt;   
	TUint32         aLength;
	void            *aData;  
};

struct TMemoryAccess
{
	TI_UINT32	addr;      			// target (HW) address
	TI_UINT32 	length;   			// length of transaction (in bytes) up to 256 bytes
	TI_UINT8	buf8[256];		 
};

struct TPltTester
{
	ETestCmdID eTestCmd;
	void	   *pTestCmdParams;
};


#endif /* TIWhaPLT_DEF_H */

