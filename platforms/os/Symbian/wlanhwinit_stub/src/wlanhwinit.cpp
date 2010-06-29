/*
 * wlanhwinit.cpp
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

/** \file  wlanhwinit.c 
 *  \brief TI WLAN Hardware init Driver
 *
 */

#include <wlanhwinit.h>
#include "WiLink6_nvs.h"
#include "wilink6_firmware.h"
#include "wlanhwinitmain.h"
#include "radio_ini.h"
#define __FILE_ID__								FILE_ID_135

// ============================ MEMBER FUNCTIONS ===============================

CWlanHwInit::CWlanHwInit() :
    iMain( NULL )
    {
    }

void CWlanHwInit::ConstructL()
    {
		/* create hwInitMain private class implemantion*/
		iMain = CWlanHwInitMain::NewL();
		
    }

EXPORT_C CWlanHwInit* CWlanHwInit::NewL()
    {
    CWlanHwInit* self = new( ELeave ) CWlanHwInit;
    CleanupStack::PushL( self );
    self->ConstructL();
    CleanupStack::Pop();

    return self;
    }
    
EXPORT_C CWlanHwInit::~CWlanHwInit()
    {
    }

// -----------------------------------------------------------------------------
// CWlanHwInit::GetHwInitData
// -----------------------------------------------------------------------------
//
EXPORT_C void CWlanHwInit::GetHwInitData(
    const TUint8** aInitData,
    TUint& aInitLength,
    const TUint8** aFwData,
    TUint& aFwLength )
    {
    
	/* Call the CHWInitMain  Class sicne all private members and method can only be stored there */
	 iMain->GetHwInitData( aInitData, aInitLength, aFwData, aFwLength );


    }

// -----------------------------------------------------------------------------
// CWlanHwInit::GetMacAddress
// -----------------------------------------------------------------------------
//
EXPORT_C TInt CWlanHwInit::GetMacAddress(
    TMacAddr& aMacAddress )
    {
	  return iMain->GetMacAddress(aMacAddress);
    }



// -----------------------------------------------------------------------------
// CWlanHwInit::GetHwTestInitData
// -----------------------------------------------------------------------------
//
EXPORT_C void CWlanHwInit::GetHwTestInitData(
    const TUint8** aInitData,
    TUint& aInitLength,
    const TUint8** aFwData,
    TUint& aFwLength )
    {
    *aInitData = NULL;
    aInitLength = 0;
    *aFwData = NULL;
    aFwLength = 0;    
    }

// -----------------------------------------------------------------------------
// CWlanHwInit::GetHwTestData
// -----------------------------------------------------------------------------
//
EXPORT_C TInt CWlanHwInit::GetHwTestData(
    TUint /*aId*/,
    TDes8& /*aData*/ )
    {
    return KErrNotSupported;
    }

// -----------------------------------------------------------------------------
// CWlanHwInit::SetHwTestData
// -----------------------------------------------------------------------------
//
EXPORT_C TInt CWlanHwInit::SetHwTestData(
    TUint /*aId*/,
    TDesC8& /*aData*/ )
    {
    return KErrNotSupported;
    }
