/*
 * wlanhwinittypes.h
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


/**********************************************************************************************************************

  FILENAME:       wlanhwinittypes.cpp
*/

#ifndef WLANHWINITTYPES_H
#define WLANHWINITTYPES_H


//  INCLUDES
#ifdef __PACKED
#undef __PACKED
#endif

#define __PACKED

/**
* Length of the MAC address
*/
const TUint8 KMacAddrLength = 6;

/**
* The one and only MAC address struct
*/
#pragma pack( 1 )
struct TMacAddr
    {
    /** the MAC address */
    TUint8 iMacAddress[KMacAddrLength];
    } __PACKED; // 6 bytes

/**
* Broadcast MAC Address.
*/
const TMacAddr KBroadcastMacAddr = {{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }};

/**
* MAC address that all zeros
*/
const TMacAddr KZeroMacAddr = {{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }};


#endif      // WLANHWINITTYPES_H   
            
// End of File
