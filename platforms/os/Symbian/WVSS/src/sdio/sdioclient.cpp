/*
 * sdioclient.cpp
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

#include "TxnDefs.h"
#include <stdlib.h>
#include <regifc.h>

#include "osApi.h"
#include "wlanosa.h"
#include "wlanosaext.h"

#include "SDioClient.h"
#include "sdiodrv.h"

/************************************************************************
 * Defines
 ************************************************************************/

/* To debug SdioClient, define some prints here */
#define     TRACE(msg) 
#ifndef _SYMBIAN_
#define     ERROR_TRACE(msg)
#else
#define     ERROR_TRACE(msg) Kern::Printf(msg);
#endif

/* Sync/Async Threshold */
#ifdef FULL_ASYNC_MODE
#define SYNC_ASYNC_LENGTH_THRESH	0     /* Use Async for all transactions */
#else
#define SYNC_ASYNC_LENGTH_THRESH	360   /* Use Async for transactions longer than this threshold (in bytes) */
#endif

SDioClient* SDioClient::mSdioClient = NULL;

 /**
  * \fn     SdioClient::SdioClient 
  * \brief  Constructor 
  * 
  * 
  * 
  * \note   
  * \param  
  * \return handle to Sdioclient. NULL on failure.
  * \sa     
  */
SDioClient::SDioClient() : WlanSpia(),
                           iSessionEndCallback(SDioClient::SessionEndCallback, this)
                           
{
    iSocketP = static_cast< DSDIOSocket* >(DPBusSocket::SocketFromId(0));
    iStackP = static_cast< DSDIOStack* >(iSocketP->Stack(0));
    iCardP = static_cast< TSDIOCard* >(iStackP->CardP(0));


	TRACE(("iSocketP 0x%x  iStackP 0x%x  iCardP 0x%x", iSocketP, iStackP, iCardP))

}


SDioClient* SDioClient::Create(MWlanOsaExt &aOsaExt)
{
    if (mSdioClient == NULL) {
        mSdioClient = new SDioClient();
    }

    mSdioClient->mWlanOsaExt = &aOsaExt;
    
    return mSdioClient;
}


SDioClient* SDioClient::Create()
{
    if (mSdioClient == NULL) {
        mSdioClient = new SDioClient();
    }
  
    return mSdioClient;
}

void SDioClient::Destroy()
{
    if (mSdioClient) {
        delete mSdioClient;
        mSdioClient = NULL;
    }
}

/**
  * \fn     SdioClient::SessionEndCallback 
  * \brief   
  * 
  * CallBack.
  * 
  * \note   
  * \param  
  * \return No return value.
  * \sa     
  */
void SDioClient::SessionEndCallback(TAny* ptr)
{
    SDioClient *hClient = (SDioClient *)ptr;

    mSdioClient->mWlanOsaExt->MutexAcquire();

    if (hClient->afCbFunc) 
    {
        hClient->afCbFunc(hClient->ahCbArg,0);
    }
    else
    {
        ERROR_TRACE(("SDioClient::SessionEndCallback - NoCallback"))
    }

    mSdioClient->mWlanOsaExt->MutexRelease();
}

/**
  * \fn     SdioClient::sdioAdapt_ConnectBus 
  * \brief   
  * 
  * Initialize HW.
  * 
  * \note   
  * \param  
  * \return KErrNone if success otherwise error.
  * \sa     
  */
int SDioClient::sdioAdapt_ConnectBus (TAny*        fCbFunc,
									  TAny*        hCbArg,
									  unsigned int  uBlkSizeShift,
									  unsigned int  uSdioThreadPriority)
{
        afCbFunc = (TTxnDoneCb1)fCbFunc;
        ahCbArg = hCbArg;

		iSocketP->PowerUp();
             
		TUint32 regValue = 4;
		TUint8 reply;
		TUint8 loopCounter = 0;
		int iStatus = KErrNone;
		TUint8 aReadDataP;

        iCardP->CommonRegisterInterface()->SetSync();

        do
		{
			iStatus = (iCardP->CommonRegisterInterface())->Write8(0x02, regValue, &aReadDataP );
			if(iStatus != KErrNone)
			ERROR_TRACE(("SDioClient::sdioAdapt_ConnectBus  Error 1"))

            iStatus = (iCardP->CommonRegisterInterface())->Read8(0x02,&reply);
			if(iStatus != KErrNone)
			ERROR_TRACE(("SDioClient::sdioAdapt_ConnectBus  Error 2"))
            
            loopCounter++;
			
		}while (reply!=regValue && loopCounter < 5);


        // Set function number 2
		(iCardP->CommonRegisterInterface())->SetFunctionNumber(2);
			
	return iStatus;
}

/**
  * \fn     SdioClient::sdioAdapt_DisconnectBus 
  * \brief   
  * 
  * DeInitialize HW.
  * 
  * \note   
  * \param  
  * \return KErrNone if success otherwise error.
  * \sa     
  */
int SDioClient::sdioAdapt_DisconnectBus (void)
{
	int iStatus = KErrNone;
	return iStatus;
}

/**
  * \fn     SdioClient::sdioAdapt_Transact 
  * \brief   
  * 
  * Transaction of multiple bytes.
  * 
  * \note   
  * \param  
  * \return TXN_STATUS_COMPLETE if transaction copleted, 
  *         TXN_STATUS_PENDING  if transaction is pending,
  *         TXN_STATUS_ERROR if error.
  * \sa     
  */
ETxnStatus SDioClient::sdioAdapt_Transact (unsigned int  uFuncId,
										   unsigned int  uHwAddr,
										   void *        pHostAddr,
										   unsigned int  uLength,
										   unsigned int  bDirection,
										   unsigned int  bBlkMode,
										   unsigned int  bFixedAddr,
										   unsigned int  bMore)
{
	int iStatus = KErrNone;
	
	TInt err = KErrNone;

    /* If transction length is below threshold, use Sync methods */
	if (uLength <= SYNC_ASYNC_LENGTH_THRESH) /* Stating with sunc calls, after this async calls will be enabled*/
    {
        iCardP->CommonRegisterInterface()->SetSync();
        if (bDirection) 
        {
			iStatus = (iCardP->CommonRegisterInterface())->ReadMultiple8(uHwAddr, (TUint8*)pHostAddr,(TUint32)uLength, bFixedAddr);
		}
        else 
        {
            iStatus = (iCardP->CommonRegisterInterface())->WriteMultiple8(uHwAddr, (TUint8*)pHostAddr, (TUint32)uLength, bFixedAddr );
        }
		/* If failed return ERROR, if succeeded return COMPLETE */
        if (iStatus != KErrNone) 
        {
            ERROR_TRACE(("SDioClient::sdioAdapt_Transact  Error"))
			return TXN_STATUS_ERROR;
        }
		// Using synchronous interface of SDIO stack. Hence complete the request here.
		Kern::RequestComplete(iClient, iReqStatus, err);
    }

	/* If transction length is above threshold, use Async methods */
    else 
    {
            if ((iCardP->CommonRegisterInterface()->SetAsync(iSessionEndCallback)) != TRUE )
            {
                ERROR_TRACE(("SDioClient::sdioAdapt_Transact - SetAsync - Fail"))
            }

        /* Call read or write Async method */
        if (bDirection) 
        {            
			iStatus = (iCardP->CommonRegisterInterface())->ReadMultiple8(uHwAddr, (TUint8*)pHostAddr,(TUint32)uLength, bFixedAddr );
        }
        else 
        {
			iStatus = (iCardP->CommonRegisterInterface())->WriteMultiple8(uHwAddr, (TUint8*)pHostAddr,(TUint32)uLength, bFixedAddr );
        }

        /* If failed return ERROR, if succeeded return PENDING */
        if (iStatus != KErrNone) 
        {
            ERROR_TRACE(("SDioClient::sdioAdapt_Transact - ASYNC - ERROR\n"))
            return TXN_STATUS_ERROR;
        }
        return TXN_STATUS_PENDING;
    }
    return TXN_STATUS_COMPLETE;
}

/**
  * \fn     SdioClient::sdioAdapt_TransactBytes 
  * \brief   
  * 
  * Transaction of bytes.
  * 
  * \note   
  * \param  
  * \return TXN_STATUS_COMPLETE if transaction copleted, 
  *         TXN_STATUS_ERROR if error.
  * \sa     
  */
ETxnStatus SDioClient::sdioAdapt_TransactBytes (unsigned int  uFuncId,
												unsigned int  uHwAddr,
												void *        pHostAddr,
												unsigned int  uLength,
												unsigned int  bDirection,
												unsigned int  bMore)
{

    int iStatus = KErrNone;
	TInt err = KErrNone;
	/* Call read or write bytes Sync method */
    if (bDirection) 
    {
		iStatus = (iCardP->CommonRegisterInterface())->Read8(uHwAddr, (TUint8*)pHostAddr);
	}
    else 
    {
        iStatus = (iCardP->CommonRegisterInterface())->Write8(uHwAddr, *((TUint8*)pHostAddr), NULL );
    }

    /* If failed return ERROR, if succeeded return COMPLETE */
    if (iStatus != KErrNone) 
    {
        ERROR_TRACE(("SDioClient::sdioAdapt_TransactBytes - ERROR\n"))
		return TXN_STATUS_ERROR;
    }

	// Using synchronous interface of SDIO stack. Hence complete the request here.
	Kern::RequestComplete(iClient, iReqStatus, err);
	
    return TXN_STATUS_COMPLETE;
}




/***************************************************************************************/
/* Create dummy implamantaion for SPIA pure virtual function 
   Remove all SPIA dependency when SDIOA will be published.
*/

WlanSpia* WlanSpia::Create( MWlanOsaExt& aOsaExt )
{
    return (SDioClient *)SDioClient::Create(aOsaExt);
}

WlanSpia* WlanSpia::Create()
{
    return (SDioClient *)SDioClient::Create();
}

void WlanSpia::Destroy( WlanSpia* aWlanSpia )
{
    SDioClient::Destroy();
}
    

