/*
 * sdioclient.h
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

#ifndef __SDIO_CLIENT_H__
#define __SDIO_CLIENT_H__

#include "e32def.h"

#include <sdio.h>
#include <pbusmedia.h>

#include "wlanosa.h"
#include "wlanosaext.h"
#include "wlanspia.h"

#define		FULL_ASYNC_MODE


typedef void (*TTxnDoneCb1)(void *hCbHandle, TUint32 status);

class SDioClient: public WlanSpia
{

public:

    static SDioClient* Create(MWlanOsaExt &aOsaExt);

    static SDioClient* Create();

    static void Destroy();

    int sdioAdapt_ConnectBus (TAny*         fCbFunc,
                              TAny*         hCbArg,
                              unsigned int  uBlkSizeShift,
                              unsigned int  uSdioThreadPriority);

    int sdioAdapt_DisconnectBus (void);

    ETxnStatus sdioAdapt_Transact (unsigned int  uFuncId,
                                   unsigned int  uHwAddr,
                                   void *        pHostAddr,
                                   unsigned int  uLength,
                                   unsigned int  bDirection,
                                   unsigned int  bBlkMode,
                                   unsigned int  bFixedAddr,
                                   unsigned int  bMore);

    ETxnStatus sdioAdapt_TransactBytes (unsigned int  uFuncId,
                                        unsigned int  uHwAddr,
                                        void *        pHostAddr,
                                        unsigned int  uLength,
                                        unsigned int  bDirection,
                                        unsigned int  bMore);

    static void SessionEndCallback(TAny *aSelfP);


private:
    void *              hSdioOs;
    TMMCCallBack        iSessionEndCallback;
    TRequestStatus*     iReqStatus;
    DThread*            iClient;
    DSDIOSocket*        iSocketP;
    DSDIOStack*         iStackP;
    TSDIOCard*          iCardP;
    TTxnDoneCb1         afCbFunc;
    TAny*               ahCbArg;
    static SDioClient*  mSdioClient;

protected:

    SDioClient();

    ~SDioClient()
    {
    }

    MWlanOsaExt*        mWlanOsaExt;

public:

    /* Create dummy implamantaion for SPIA pure virtual function 
       Remove all SPIA dependency when SDIOA will be published.
       */
    void Configure( const TConfig& aConfig )
    {
    }
    SPIA::TStatus Request( const TRequest& aRequest )
    {
        return SPIA::ESuccess;
    }
    SPIA::TStatus Cancel( TRequestId aRequestId )
    {
        return SPIA::ESuccess;
    }
};

#endif //__SDIO_CLIENT_H__
