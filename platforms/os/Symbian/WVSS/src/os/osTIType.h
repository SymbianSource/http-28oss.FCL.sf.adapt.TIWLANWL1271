/*
 * osTIType.h
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


/*
 * inc/osTIType.h
 *
 * This module contains eSTA-DK types definitions
 *
 * Copyright (C) 2006 Texas Instruments, Inc.
 *
 * Acknowledgements:
 *   None
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed “as is” WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#ifndef __OSTITYPE_H__
#define __OSTITYPE_H__


typedef signed char         TI_INT8;
typedef unsigned char       TI_UINT8;
typedef short               TI_INT16;
typedef unsigned short      TI_UINT16;
typedef signed int          TI_INT32;
typedef unsigned int        TI_UINT32;
typedef long long           TI_INT64;
typedef unsigned long long  TI_UINT64;

//typedef TI_UINT32           TI_BOOL;
typedef char                TI_CHAR;
/*
typedef TI_INT8;
typedef TI_UINT8;
typedef TI_INT16;
typedef TI_UINT16;
typedef TI_INT32;
typedef TI_UINT32;
typedef TI_INT64;
typedef TI_UINT64;*/

#ifndef _SYMBIAN_ 
#define __TI_FILE__ 	__S40_FILE__ 
#endif

#define TI_CONST64(x)       (x##LL)

#define INLINE 

#ifndef VOID
#define VOID        void
#define PVOID       void*
#endif

#define TI_LIKELY           
#define TI_UNLIKELY         


#endif /* __OSTITYPE_H__*/


