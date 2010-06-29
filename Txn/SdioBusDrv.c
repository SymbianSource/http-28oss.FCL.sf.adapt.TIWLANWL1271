/*
 * SdioBusDrv.c
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

 
/** \file   SdioBusDrv.c 
 *  \brief  The SDIO bus driver upper layer. Platform independent. 
 *          Uses the SdioAdapter API. 
 *          Introduces a generic bus-independent API upwards.
 *  
 *  \see    BusDrv.h, SdioAdapter.h, SdioAdapter.c
 */

#define __FILE_ID__  FILE_ID_122
#include "tidef.h"
#include "report.h"
#include "osApi.h"
#include "TxnDefs.h"
#include "SdioAdapter.h"
#include "BusDrv.h"


/************************************************************************
 * Defines
 ************************************************************************/
#undef  USE_BLOCK_MODE    /* Don't use SDIO block mode */

#define MAX_TXN_PARTS     MAX_XFER_BUFS * 2   /* Max number of txn parts derived from one TxnStruct */


/************************************************************************
 * Types
 ************************************************************************/

/* A single SDIO bus transaction which is a part of a complete transaction (TTxnStruct) */ 
typedef struct
{
    TI_BOOL          bBlkMode;           /* If TRUE this is a block-mode SDIO transaction */
    TI_UINT32        uLength;            /* Length in byte */
    TI_UINT32        uHwAddr;            /* The device address to write to or read from */
    void *           pHostAddr;          /* The host buffer address to write from or read into */
    TI_BOOL          bMore;              /* If TRUE, indicates the lower driver to keep awake for more transactions */
} TTxnPart; 


/* The busDrv module Object */
typedef struct _TBusDrvObj
{
    TI_HANDLE	     hOs;		   	 
    TI_HANDLE	     hReport;

	TBusDrvTxnDoneCb fTxnDoneCb;         /* The callback to call upon full transaction completion. */
	TI_HANDLE        hCbHandle;          /* The callback handle */
    TTxnStruct *     pCurrTxn;           /* The transaction currently being processed */
    ETxnStatus       eCurrTxnStatus;     /* COMPLETE, PENDING or ERROR */
    TTxnPart         aTxnParts[MAX_TXN_PARTS]; /* The actual bus transactions of current transaction */
    TI_UINT32        uCurrTxnPartsNum;   /* Number of transaction parts composing the current transaction */
    TI_UINT32        uCurrTxnPartsCount; /* Number of transaction parts already executed */
    TI_UINT32        uBlkSizeShift;      /* In block-mode:  uBlkSize = (1 << uBlkSizeShift) = 512 bytes */
    TI_UINT32        uBlkSize;           /* In block-mode:  uBlkSize = (1 << uBlkSizeShift) = 512 bytes */
    TI_UINT32        uBlkSizeMask;       /* In block-mode:  uBlkSizeMask = uBlkSize - 1 = 0x1FF*/
	TI_HANDLE	     hWsdio;
} TBusDrvObj;


/************************************************************************
 * Internal functions prototypes
 ************************************************************************/
static void busDrv_PrepareTxnParts  (TBusDrvObj *pBusDrv, TTxnStruct *pTxn);
static void busDrv_SendTxnParts     (TBusDrvObj *pBusDrv);
static void busDrv_TxnDoneCb        (TI_HANDLE hBusDrv, TI_INT32 status);
 


/************************************************************************
 *
 *   Module functions implementation
 *
 ************************************************************************/

/** 
 * \fn     busDrv_Create 
 * \brief  Create the module
 * 
 * Create and clear the bus driver's object, and the SDIO-adapter.
 * 
 * \note   
 * \param  hOs - Handle to Os Abstraction Layer
 * \return Handle of the allocated object, NULL if allocation failed 
 * \sa     busDrv_Destroy
 */ 
TI_HANDLE busDrv_Create (TI_HANDLE hOs)
{
    TI_HANDLE   hBusDrv;
    TBusDrvObj *pBusDrv;

    hBusDrv = os_memoryAlloc(hOs, sizeof(TBusDrvObj),MemoryNormal);
    if (hBusDrv == NULL)
    {
		WLAN_OS_REPORT(("SdioBusDrv.c - os_memoryAlloc - ERROR"));
        return NULL;
    }
    
    pBusDrv = (TBusDrvObj *)hBusDrv;

    os_memoryZero(hOs, hBusDrv, sizeof(TBusDrvObj));
    
    pBusDrv->hOs = hOs;
	pBusDrv->hWsdio = wsdioAdapt_Open(pBusDrv->hOs);

    return pBusDrv;
}

TI_HANDLE busDrv_Cfg (TI_HANDLE hOs)
{
	TI_HANDLE	hBusDrvCfg;
    TBusDrvCfg *pBusDrvCfg;
	
	//WLAN_OS_REPORT(("SdioBusDrv.c - busDrv_Create - BEGIN"));

    hBusDrvCfg = os_memoryAlloc(hOs, sizeof(TBusDrvCfg),MemoryNormal);
    if (hBusDrvCfg == NULL)
    {
		WLAN_OS_REPORT(("SdioBusDrv.c - os_memoryAlloc - ERROR"));
		return NULL;
    }
    
    pBusDrvCfg = (TBusDrvCfg *)hBusDrvCfg;

	return pBusDrvCfg;
}

/** 
 * \fn     busDrv_Destroy
 * \brief  Destroy the module. 
 * 
 * Close SDIO lower bus driver and free the module's object.
 * 
 * \note   
 * \param  The module's object
 * \return TI_OK on success or TI_NOK on failure 
 * \sa     busDrv_Create
 */ 
TI_STATUS busDrv_Destroy (TI_HANDLE hBusDrv)
{
    TBusDrvObj *pBusDrv = (TBusDrvObj*)hBusDrv;

    if (pBusDrv)
    {
        wSdioAdapt_Close(pBusDrv->hWsdio);
        os_memoryFree (pBusDrv->hOs, pBusDrv, sizeof(TBusDrvObj));     
    }
    return TI_OK;
}


/** 
 * \fn     busDrv_Init
 * \brief  Init bus driver 
 * 
 * Init module parameters.

 * \note   
 * \param  hBusDrv - The module's handle
 * \param  hReport - report module handle
 * \return void
 * \sa     
 */ 
void busDrv_Init (TI_HANDLE hBusDrv, TI_HANDLE hReport)
{
    TBusDrvObj *pBusDrv = (TBusDrvObj*) hBusDrv;

    pBusDrv->hReport = hReport;
}


/** 
 * \fn     busDrv_ConnectBus
 * \brief  Configure bus driver
 * 
 * Called by TxnQ.
 * Configure the bus driver with its connection configuration (such as baud-rate, bus width etc) 
 *     and establish the physical connection. 
 * Done once upon init (and not per functional driver startup). 
 * 
 * \note   
 * \param  hBusDrv    - The module's object
 * \param  pBusDrvCfg - A union used for per-bus specific configuration. 
 * \param  fCbFunc    - CB function for Async transaction completion (after all txn parts are completed).
 * \param  hCbArg     - The CB function handle
 * \return TI_OK / TI_NOK
 * \sa     
 */ 
TI_STATUS busDrv_ConnectBus (TI_HANDLE        hBusDrv, 
                             TBusDrvCfg       *pBusDrvCfg,
                             TBusDrvTxnDoneCb fCbFunc,
                             TI_HANDLE        hCbArg,
                             TBusDrvTxnDoneCb fConnectCbFunc)
{
    TBusDrvObj *pBusDrv = (TBusDrvObj*)hBusDrv;
    int         iStatus;

    /* Save the parameters (TxnQ callback for TxnDone events, and block-size) */
    pBusDrv->fTxnDoneCb    = fCbFunc;
    pBusDrv->hCbHandle     = hCbArg;
    pBusDrv->uBlkSizeShift = pBusDrvCfg->tSdioCfg.uBlkSizeShift;
    pBusDrv->uBlkSize      = 1 << pBusDrv->uBlkSizeShift;
    pBusDrv->uBlkSizeMask  = pBusDrv->uBlkSize - 1;
    /* This should cover stop send Txn parts in recovery */
    pBusDrv->uCurrTxnPartsCount = 0;
    pBusDrv->uCurrTxnPartsNum = 0;

	iStatus = wsdioAdapt_ConnectBus (pBusDrv->hWsdio, 
									 /*pBusDrv->hWsdiobusDrv_TxnDoneCb,*/
									 (TAny*)busDrv_TxnDoneCb,
									 hBusDrv,
									 pBusDrv->uBlkSizeShift,
									 0);

    /* Configure the SDIO driver parameters and handle SDIO enumeration */
//    iStatus = sdioAdapt_ConnectBus (busDrv_TxnDoneCb, hBusDrv, pBusDrv->uBlkSizeShift);

    if (iStatus == 0) 
    {
        return TI_OK;
    }
    else 
    {
        TRACE2(pBusDrv->hReport, REPORT_SEVERITY_ERROR, ": Status = 0x%x, BlkSize = %d\n", iStatus, pBusDrv->uBlkSize);
        return TI_NOK;
    }
}


/** 
 * \fn     busDrv_DisconnectBus
 * \brief  Disconnect SDIO driver
 * 
 * Called by TxnQ. Disconnect the SDIO driver.
 *  
 * \note   
 * \param  hBusDrv - The module's object
 * \return TI_OK / TI_NOK
 * \sa     
 */ 
TI_STATUS busDrv_DisconnectBus (TI_HANDLE hBusDrv)
{
    TBusDrvObj *pBusDrv = (TBusDrvObj*)hBusDrv;

    TRACE0(pBusDrv->hReport, REPORT_SEVERITY_INFORMATION, "\n");

    /* Disconnect SDIO driver */
    return wsdioAdapt_DisconnectBus (pBusDrv->hWsdio);
}


/** 
 * \fn     busDrv_Transact
 * \brief  Process transaction 
 * 
 * Called by the TxnQ module to initiate a new transaction.
 * Prepare the transaction parts (lower layer single transactions),
 *      and send them one by one to the lower layer.
 * 
 * \note   It's assumed that this function is called only when idle (i.e. previous Txn is done).
 * \param  hBusDrv - The module's object
 * \param  pTxn    - The transaction object 
 * \return COMPLETE if Txn completed in this context, PENDING if not, ERROR if failed
 * \sa     busDrv_PrepareTxnParts, busDrv_SendTxnParts
 */ 
ETxnStatus busDrv_Transact (TI_HANDLE hBusDrv, TTxnStruct *pTxn)
{
    TBusDrvObj *pBusDrv = (TBusDrvObj*)hBusDrv;

    pBusDrv->pCurrTxn           = pTxn;
    pBusDrv->uCurrTxnPartsCount = 0;
    pBusDrv->eCurrTxnStatus     = TXN_STATUS_COMPLETE;  /* The Txn is Sync as long as it continues in this context */

    /* Prepare the transaction parts in a table. */
    busDrv_PrepareTxnParts (pBusDrv, pTxn);

    /* Send the prepared transaction parts. */
    busDrv_SendTxnParts (pBusDrv);

    TRACE1(pBusDrv->hReport, REPORT_SEVERITY_INFORMATION, ": Status = %d\n", pBusDrv->eCurrTxnStatus);

    /* return transaction status - COMPLETE, PENDING or ERROR */
    return pBusDrv->eCurrTxnStatus;
}


/** 
 * \fn     busDrv_PrepareTxnParts
 * \brief  Prepare write or read transaction parts
 * 
 * Called by busDrv_Transact().
 * Prepares the actual sequence of SDIO bus transactions in a table.
 * 
 * \note   
 * \param  pBusDrv - The module's object
 * \param  pTxn    - The transaction object 
 * \return void
 * \sa     busDrv_Transact, busDrv_SendTxnParts, 
 */ 
static void busDrv_PrepareTxnParts (TBusDrvObj *pBusDrv, TTxnStruct *pTxn)
{
    TI_UINT32 uBufNum;
    TI_UINT32 uPartNum = 0;
    TI_UINT32 uRemainderLen;
    TI_UINT32 uCurrHwAddr = pTxn->uHwAddr;
    TI_BOOL   bFixedHwAddr = TXN_PARAM_GET_FIXED_ADDR(pTxn);
#ifndef  USE_BLOCK_MODE  
    TI_UINT32 uLen;  
#endif

    /* For all occupied buffers in current transaction do */
    for (uBufNum = 0; uBufNum < MAX_XFER_BUFS; uBufNum++) 
    {
        /* If no more buffers, exit the for loop */
        if (pTxn->aLen[uBufNum] == 0)
        {
            break;
        }

        /* If current buffer has a remainder, prepare its transaction part */
        uRemainderLen = pTxn->aLen[uBufNum] & pBusDrv->uBlkSizeMask;
        if (uRemainderLen > 0)
        {
            pBusDrv->aTxnParts[uPartNum].bBlkMode  = TI_FALSE;
            pBusDrv->aTxnParts[uPartNum].uLength   = uRemainderLen;
            pBusDrv->aTxnParts[uPartNum].uHwAddr   = uCurrHwAddr;
            pBusDrv->aTxnParts[uPartNum].pHostAddr = (void *)(pTxn->aBuf[uBufNum]);
            pBusDrv->aTxnParts[uPartNum].bMore     = TI_TRUE;

            /* If not fixed HW address, increment it by this part's size */
            if (!bFixedHwAddr)
            {
                uCurrHwAddr += uRemainderLen;
            }

            uPartNum++;
        }

#ifdef  USE_BLOCK_MODE     /* Block Mode */

        /* If current buffer has full SDIO blocks, prepare a block-mode transaction part */
        if (pTxn->aLen[uBufNum] >= pBusDrv->uBlkSize)
        {
            pBusDrv->aTxnParts[uPartNum].bBlkMode  = TI_TRUE;
            pBusDrv->aTxnParts[uPartNum].uLength   = pTxn->aLen[uBufNum] - uRemainderLen;
            pBusDrv->aTxnParts[uPartNum].uHwAddr   = uCurrHwAddr;
            pBusDrv->aTxnParts[uPartNum].pHostAddr = (void *)(pTxn->aBuf[uBufNum] + uRemainderLen);
            pBusDrv->aTxnParts[uPartNum].bMore     = TI_TRUE;

            /* If not fixed HW address, increment it by this part's size */
            if (!bFixedHwAddr)
            {
                uCurrHwAddr += pTxn->aLen[uBufNum] - uRemainderLen;
            }

            uPartNum++;
        }

#else   

		/* Split to blocks of block mode size only (512 bytes) */
		for (uLen = uRemainderLen; uLen < pTxn->aLen[uBufNum]; uLen += pBusDrv->uBlkSize)
		{
			pBusDrv->aTxnParts[uPartNum].bBlkMode  = TI_FALSE;
			pBusDrv->aTxnParts[uPartNum].uLength   = pBusDrv->uBlkSize;
			pBusDrv->aTxnParts[uPartNum].uHwAddr   = uCurrHwAddr;
			pBusDrv->aTxnParts[uPartNum].pHostAddr = (void *)(pTxn->aBuf[uBufNum] + uLen);
			pBusDrv->aTxnParts[uPartNum].bMore     = TI_TRUE;

			/* If not fixed HW address, increment it by this part's size */
			if (!bFixedHwAddr)
			{
				uCurrHwAddr += pBusDrv->uBlkSize;
			}

			uPartNum++;
		}

#endif

    }

    /* Set last More flag as specified for the whole Txn */
    pBusDrv->aTxnParts[uPartNum - 1].bMore = TXN_PARAM_GET_MORE(pTxn);
    pBusDrv->uCurrTxnPartsNum = uPartNum;
}


/** 
 * \fn     busDrv_SendTxnParts
 * \brief  Send prepared transaction parts
 * 
 * Called first by busDrv_Transact(), and also from TxnDone CB after Async completion.
 * Sends the prepared transaction parts in a loop.
 * If a transaction part is Async, the loop continues later in the TxnDone ISR context.
 * When all parts are done, the upper layer TxnDone CB is called.
 * 
 * \note   
 * \param  pBusDrv - The module's object
 * \return void
 * \sa     busDrv_Transact, busDrv_PrepareTxnParts
 */ 
static void busDrv_SendTxnParts (TBusDrvObj *pBusDrv)
{
    ETxnStatus  eStatus;
    TTxnPart   *pTxnPart;

    /* While there are transaction parts to send */
    while (pBusDrv->uCurrTxnPartsCount < pBusDrv->uCurrTxnPartsNum)
    {
        pTxnPart = &(pBusDrv->aTxnParts[pBusDrv->uCurrTxnPartsCount]);

        /* If single step, send ELP byte */
        if (TXN_PARAM_GET_SINGLE_STEP(pBusDrv->pCurrTxn)) 
        {
            /* Overwrite the function id with function 0 - for ELP register !!!! */
            eStatus = wsdioAdapt_TransactBytes (pBusDrv->hWsdio,
											   	TXN_FUNC_ID_CTRL,
                                               pTxnPart->uHwAddr,
                                               pTxnPart->pHostAddr,
                                               pTxnPart->uLength,
                                               TXN_PARAM_GET_DIRECTION(pBusDrv->pCurrTxn),
                                               pTxnPart->bMore);
        }
        else
        {
            eStatus = wsdioAdapt_Transact (pBusDrv->hWsdio,
										   TXN_PARAM_GET_FUNC_ID(pBusDrv->pCurrTxn),
                                          pTxnPart->uHwAddr,
                                          pTxnPart->pHostAddr,
                                          pTxnPart->uLength,
                                          TXN_PARAM_GET_DIRECTION(pBusDrv->pCurrTxn),
                                          pTxnPart->bBlkMode,
                                          ((TXN_PARAM_GET_FIXED_ADDR(pBusDrv->pCurrTxn)==1)?0:1),
                                          pTxnPart->bMore);
        }

        TRACE7(pBusDrv->hReport, REPORT_SEVERITY_INFORMATION, ": PartNum = %d, SingleStep = %d, Direction = %d, HwAddr = 0x%x, HostAddr = 0x%x, Length = %d, BlkMode = %d\n", pBusDrv->uCurrTxnPartsCount, TXN_PARAM_GET_SINGLE_STEP(pBusDrv->pCurrTxn), TXN_PARAM_GET_DIRECTION(pBusDrv->pCurrTxn), pTxnPart->uHwAddr, pTxnPart->pHostAddr, pTxnPart->uLength, pTxnPart->bBlkMode);

        /* Increment transaction parts counter */
        pBusDrv->uCurrTxnPartsCount++;

        /* If transaction is not fully Sync, continue it in next TxnDone interrupt */
        if (eStatus == TXN_STATUS_PENDING)
        {
            pBusDrv->eCurrTxnStatus = TXN_STATUS_PENDING;   
            return; 
        }

        /* If error, set error in Txn struct, call TxnDone CB if not fully sync, and exit */
        if (eStatus == TXN_STATUS_ERROR)
        {
            TXN_PARAM_SET_STATUS(pBusDrv->pCurrTxn, TXN_PARAM_STATUS_ERROR);
            if (pBusDrv->eCurrTxnStatus == TXN_STATUS_PENDING)
            {
                pBusDrv->fTxnDoneCb (pBusDrv->hCbHandle, pBusDrv->pCurrTxn);
            }
        	return;
        }
    }

    /* If we got here we sent all buffers and we don't pend transaction end */
    TRACE1(pBusDrv->hReport, REPORT_SEVERITY_INFORMATION, ": Txn finished successfully, Status = %d\n", pBusDrv->eCurrTxnStatus);

    /* Set status OK in Txn struct, and call TxnDone CB if not fully sync */
    TXN_PARAM_SET_STATUS(pBusDrv->pCurrTxn, TXN_PARAM_STATUS_OK);
    if (pBusDrv->eCurrTxnStatus == TXN_STATUS_PENDING)
    {
        pBusDrv->fTxnDoneCb (pBusDrv->hCbHandle, pBusDrv->pCurrTxn);
    }
}


/** 
 * \fn     busDrv_TxnDoneCb
 * \brief  Continue async transaction processing (CB)
 * 
 * Called back by the lower (BSP) bus-driver upon Async transaction completion (TxnDone ISR).
 * Call busDrv_SendTxnParts to continue sending the remained transaction parts.
 * 
 * \note   
 * \param  hBusDrv - The module's object
 * \param  status  - The last transaction result - 0 = OK, else Error
 * \return void
 * \sa     busDrv_SendTxnParts
 */ 
static void busDrv_TxnDoneCb (TI_HANDLE hBusDrv, int iStatus)
{
    TBusDrvObj *pBusDrv = (TBusDrvObj*)hBusDrv;

    /* If last transaction part failed, set error in Txn struct, call TxnDone CB and exit. */
    if (iStatus != 0)
    {
        TRACE1(pBusDrv->hReport, REPORT_SEVERITY_ERROR, ": Status = 0x%x\n", iStatus);

        TXN_PARAM_SET_STATUS(pBusDrv->pCurrTxn, TXN_PARAM_STATUS_ERROR);
        pBusDrv->fTxnDoneCb (pBusDrv->hCbHandle, pBusDrv->pCurrTxn);
        return;
    }

    TRACE0(pBusDrv->hReport, REPORT_SEVERITY_INFORMATION, "\n");

    /* Continue sending the remained transaction parts. */
    busDrv_SendTxnParts (pBusDrv);
}



