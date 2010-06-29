/*
 * TIWhaTxRx.cpp
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




/** \file  TIWhaTxRx.cpp 
 *  \brief  Control the data path from UMAC and back to UMAC
 *
 *  \see   
*/

#include "TIWha.h"

/*******************************************************************************
 * stuff for C file linking.                                                   *
 *******************************************************************************/

extern "C" 
{

#include "tidef.h"
#include "report.h"
#include "timer.h"
#include "TWDriver.h"
#include "version.h"
#include "osApi.h"
#include "context.h"
#include "BusDrv.h"
#include "public_types.h"
#define __FILE_ID__								FILE_ID_147
}

static const TI_UINT8 WMEQosAcToTid[MAX_NUM_OF_AC] = { 0, 2, 4, 6 };

#define WORD_SIZE                       4
#define WORD_ALIGN_MASK                 0xFFFC
#define GET_EXTRA_BYTES(status)		( (status & 0xc0) >> 6)
#define KEY_GEM_TYPE                    4
#define QOS_PADDING_SIZE                4
#define QOS_SHIFT_SIZE                  2


/********************************************************************************
*																				*
*                       MACROS and INLINE FUNCTIONS           					*
*																				*
*********************************************************************************/
/* Update packet length in the descriptor according to HW interface requirements */
static inline TI_UINT16 TranslateLengthToFw (TTxCtrlBlk *pPktCtrlBlk)
{
    TI_UINT16 uPktLen      = pPktCtrlBlk->tTxDescriptor.length;
    TI_UINT16 uLastWordPad = (4 - (uPktLen & 3)) & 0x3;  /* Find number of bytes needed to align */

    uPktLen = (uPktLen + uLastWordPad) >> 2;     /* Add alignment bytes and convert to words */
	pPktCtrlBlk->tTxDescriptor.length = ENDIAN_HANDLE_WORD(uPktLen);

    return uLastWordPad;
}

/* Translate packet timestamp to FW time, and update also lifeTime and uDriverDelay */
static inline void TranslateTimeToFw (TwdCtrl *iTwdCtrl, TTxCtrlBlk *pPktCtrlBlk, TI_UINT16 uLifeTime)
{
    TI_UINT32 uPktStartTime = pPktCtrlBlk->tTxDescriptor.startTime;  /* Contains host start time */

    /* Save host packet handling time until this point (for statistics) */
    pPktCtrlBlk->tTxPktParams.uDriverDelay = os_timeStampMs (iTwdCtrl->tOsContext.hOsa) - uPktStartTime;
                                             
    /* Translate packet timestamp to FW time and undate descriptor */
    uPktStartTime = TWD_TranslateToFwTime (iTwdCtrl->hTWD, uPktStartTime); 
	pPktCtrlBlk->tTxDescriptor.startTime = ENDIAN_HANDLE_LONG (uPktStartTime);
	pPktCtrlBlk->tTxDescriptor.lifeTime  = ENDIAN_HANDLE_WORD (uLifeTime);
}


/** 
 * \fn     RxMemFailTimerCb
 * \brief  Call again the rxXfer module to handle all pending packets.
 * \note    
 * \return  
 * \sa      
 */ 
void TIWha::RxMemFailTimerCb ()
{
	TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "RxMemFailTimerCb uRxMemFailCount %d\n",uRxMemFailCount);

    bRxMemFailTimerRunning = TI_FALSE;

    /* if uRxMemFailCount == 0 it means that Rx allocation is back to life - no need to call RxXfer */
    if (uRxMemFailCount)
    {
        TWD_rxXfer_Handle(iTwdCtrl.hTWD);
    }
}

/** 
 * \fn     RequestForBuffer
 * \brief  member function. calls LDD
 * 
 * \note    
 * \param  aLength - aligned length of packet.
 * \return  
 * \sa      
 */ 
ERxBufferStatus TIWha::RequestForBufferCb (void **pWbuf, TI_UINT16 aLength, TI_UINT32 uEncryptionFlag)
{
    /* Total security overhead to be asked for in RequestForBuffer ()  */
    TI_UINT32 uExtraSecurityLength = 0; 
    /* Offset to be used when returning the buffer pointer. It will be used shifting the header backwards */
    TI_UINT32 uSecurityOffset	   = 0;	
    ERxBufferStatus eBufferStatus;
    TI_UINT8* pRequestedBuf = NULL;

    /*
     * We should manipulate the packet according to the security mode. See ReceivePacket() for
     * more details. 
     * We have 2 issues to deal with:
     * 1) Add additional length to the buffer request.
    	 * 2) Change the pointer such that it will point forward by the size that should be added after the header.
     *   	It is done in order to pull back the header later in GWSI_AdaptCB_ReceivePacket() to support GWSI spec.
     */

    
    switch (uEncryptionFlag) 
    {
        case KEY_NULL:		
        	uExtraSecurityLength = 0;
        	uSecurityOffset		 = 0;
            break;
    
        case KEY_WEP:	
            uExtraSecurityLength = IV_FIELD_SIZE + ICV_FIELD_SIZE;
        	uSecurityOffset		 = IV_FIELD_SIZE;
            break;
    
        case KEY_TKIP:
        	uExtraSecurityLength = IV_FIELD_SIZE + EIV_FIELD_SIZE + MIC_FIELD_SIZE + ICV_FIELD_SIZE;
        	uSecurityOffset		 = IV_FIELD_SIZE + EIV_FIELD_SIZE;
            break;
    
        case KEY_AES:		
        	uExtraSecurityLength = AES_AFTER_HEADER_FIELD_SIZE + MIC_FIELD_SIZE;
        	uSecurityOffset		 = AES_AFTER_HEADER_FIELD_SIZE;
            break;

        #ifdef GEM_SUPPORT
            case KEY_GEM_TYPE:
                uExtraSecurityLength = GEM_AFTER_HEADER_FIELD_SIZE + GEM_MIC_FIELD_SIZE;
            	uSecurityOffset		 = GEM_AFTER_HEADER_FIELD_SIZE + GEM_BUFFER_ALIGNMENT;
                break;
        #endif

        default:
            TRACE2(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "RequestForBuffer: uEncryptionFlag = %d uExtraSecurityLength = %d\n", uEncryptionFlag, uExtraSecurityLength);
            break;
    }
    

    if (iRxPacketsAllocated < MAX_WHA_RX_BUFFERS)
    {
        /* 
         * We are requesting extra bytes for security, but decreasing WSPI_PAD_LEN_READ since the spec indicates
         *  that the Rx offset will be added any way 
         */
        if( bErrorIndication == TI_FALSE)
        {    
            pRequestedBuf = (TI_UINT8*) WhaCb()->RequestForBuffer(aLength - sizeof(RxIfDescriptor_t) + uExtraSecurityLength + QOS_PADDING_SIZE + PAYLOAD_ALIGN_PAD_BYTES);
        }
        #if TI_DBG
            else
            {
                WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
            }
        #endif	
        
        if ( pRequestedBuf == NULL )
        { 
            #ifdef PLT_TESTER
                return RX_BUF_ALLOC_OUT_OF_MEM;
            #endif
                                  
            uRxMemFailCount++;
			/* In case no timer is running - start a new timer to call RxXfer later */
			if (!bRxMemFailTimerRunning) 
			{
                #ifdef TI_DBG
                if (uRxMemFailCount == 1)
                {
                    /* Print error only for the first time */
                    TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR, "Start hRxMemFailTimer for the first time\n");
                }
                #endif
                TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "Start hRxMemFailTimer uRxMemFailCount %d\n",uRxMemFailCount);
                /* In case we fail too many times, just mark as OUT_OF_MEM so the Rx packet will be dropped */
                if (uRxMemFailCount > MAX_CONSECUTIVE_RX_MEM_FAIL)
                {
                    return RX_BUF_ALLOC_OUT_OF_MEM;
                }

				bRxMemFailTimerRunning = TI_TRUE;
                os_timerStart(&iTwdCtrl.tOsContext,hRxMemFailTimer,RX_ALLOC_FAIL_TIME_OUT);
			}

            /* In this case rxXfer will exit the loop and wait for someone to trigger the SM again */      
			return RX_BUF_ALLOC_PENDING;
        }

        /* Increase the number of allocated buffers */
        iRxPacketsAllocated++;
    
#ifdef TI_DBG
        /* Just have a print in case we are out of the bad situation of RX_BUF_ALLOC_OUT_OF_MEM */
        if (uRxMemFailCount)
        {
					TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR, "Rx allocation succeeded after %d times\n",uRxMemFailCount);
        }
#endif /* TI_DBG */

        /* Rx allocation succeeded so set counter to zero */
        uRxMemFailCount = 0;

        TRACE2(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "RequestForBuffer: uEncryptionFlag = %d uExtraSecurityLength = %d\n", uEncryptionFlag, uExtraSecurityLength);

        TRACE3(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "RequestForBuffer: len = %d OriginalPointer = 0x%x, allocatedBuffer#= %d \n", aLength, *pWbuf, iRxPacketsAllocated);

        TRACE2(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "RequestForBuffer: Pointer2Buffer= 0x%x, Buffer= 0x%x \n",pWbuf, *pWbuf);

        /* The packet will be read as if it started uSecurityOffset bytes ahead and ReceivePacket() will take it back */
        /* pWbuf points to the actual start of the data */
        pRequestedBuf += uSecurityOffset + WSPI_PAD_LEN_READ;
        
        *pWbuf = (void*)pRequestedBuf;
        
        eBufferStatus = RX_BUF_ALLOC_COMPLETE;       
    }
    else
    {
        TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_WARNING, "RequestForBuffer: RX_BUF_ALLOC_PENDING");
        /* RxXfer will try to allocate the buffer again when other buffer will be released */ 
        eBufferStatus = RX_BUF_ALLOC_PENDING;       
    }
    
    return eBufferStatus;
}

 
/** 
 * \fn     ReceivePacket
 * \brief  member function. calls LDD
 * 
 * \note    
 * \param  aStatus - not used but retrieved from RxIfDescriptor_t.
 * \param  aBuf	   - pointer to start of RxIfDescriptor_t.
 * \return  
 * \sa      
 */ 
void TIWha::ReceivePacketCb (const void *aBuf)
{
	WHA::TStatus 		aStatus;
	TUint16				aLength;
	WHA::TRate			aRate;
	WHA::TRcpi			aRcpi;
	WHA::TChannelNumber aChannel;
	TUint32 			aFlags;
	/* move aBuf to begeinnig of MAC heder */
	const void*			aFrame = (TI_UINT8*)aBuf + sizeof(RxIfDescriptor_t);
    dot11_header_t      *pDot11Hdr = (dot11_header_t*)aFrame;
	RxIfDescriptor_t*	pRxParams = (RxIfDescriptor_t*)aBuf;
    TBool               bQos;

    
    
	switch (pRxParams->status)
	{
	case RX_FAILURE_NONE:                     
        aStatus = WHA::KSuccess;          
        break;

    case RX_FAILURE_MIC_ERROR:   
        aStatus = WHA::KMicFailure; 
        TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR , "ReceivePacket MIC_FAILURE !!! \n");
        break; 

    case RX_FAILURE_DECRYPT:     
        aStatus = WHA::KDecryptFailure;   
        TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR , "ReceivePacket DECRYPT_FAILURE !!! \n");
        break; 

    default: 
    	TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR ,"ReceivePacketCb : WHA::KFailed");
        aStatus = WHA::KFailed;
	}

    /* convert to LDD rate */
	aRate	= TIWhaUtils::PolicyToWhaRate ((ETxRateClassId)pRxParams->rate);

	/* calculate RCPI form RSSI */
	if (pRxParams->rx_level >= 0)
		aRcpi = 220;
	else
		if (pRxParams->rx_level <= -110)
			aRcpi = 0;
		else
			aRcpi = (pRxParams->rx_level + 110)*2;

	/* Channel of received frame */
	aChannel = pRxParams->channel;

    /* pRxParams->length refer to total length form MAC heder in bytes */
	aLength = pRxParams->length*WORD_SIZE - pRxParams->extraBytes - sizeof(RxIfDescriptor_t);

	/* flags description:
	   1. bits 0 - 15 are optional
	   2. bits 16 - 18 security TBD when security integrated
	   3. bit 19 for more pending frames
	   4. bit 20 packet was received during a measurement process or not
     */
    aFlags = 0;

    /*
	 * The next adaptation code is meant to align with the driver legacy operation of extra padding for security:
	 *
	 * WEP: 	TnetwDrv	 	HEADER - DATA
	 *    		Upper layer		HEADER - IV (4) - DATA - ICV (4)
	 *
	 * TKIP: 	TnetwDrv	 	HEADER - DATA 
	 *			Upper layer		HEADER - IV (4)  - EIV (4) - DATA - MIC (8) - ICV (4)
	 *   		
	 * AES: 	TnetwDrv	 	HEADER - DATA
	 *			Upper layer		HEADER - RSN (8) - DATA - MIC (8) 	
	 *
     * GEM: 	TnetwDrv	 	HEADER - DATA
	 *			Upper layer		HEADER - KeyIdx(1) - Reserved(1) - PN(16) - DATA - MIC (16) 	
     *
	 * Note: 	Adding the offset is done even if the packet status is error	
	 */
    bQos = IS_QOS_FRAME(pDot11Hdr->fc);
    TUint8 shiftBytes = WSPI_PAD_LEN_READ; 
    
    switch (pRxParams->flags & RX_DESC_ENCRYPT_MASK)
    {
        case RX_DESC_ENCRYPT_WEP:
            ALIGN_HEADER_TO_DATA_BACKWARD (aFrame, WEP_AFTER_HEADER_FIELD_SIZE, bQos)
            shiftBytes += WEP_AFTER_HEADER_FIELD_SIZE;
            /* Increase length size by 8 (IV + ICV) */
            aLength += (WEP_AFTER_HEADER_FIELD_SIZE + ICV_FIELD_SIZE);
            aFlags |= WHA_WEP;
            break;
        
        case RX_DESC_ENCRYPT_TKIP:
            ALIGN_HEADER_TO_DATA_BACKWARD (aFrame, TKIP_AFTER_HEADER_FIELD_SIZE, bQos)
            shiftBytes += TKIP_AFTER_HEADER_FIELD_SIZE;
            /* Increase length size by 20 (IV + EIV + MIC + ICV) */
            aLength += (TKIP_AFTER_HEADER_FIELD_SIZE + MIC_FIELD_SIZE + ICV_FIELD_SIZE);
            aFlags |= WHA_TKIP;
            break;
    
        case RX_DESC_ENCRYPT_AES:
            ALIGN_HEADER_TO_DATA_BACKWARD (aFrame, AES_AFTER_HEADER_FIELD_SIZE, bQos)
            shiftBytes += AES_AFTER_HEADER_FIELD_SIZE;
            /* Increase length size by 16 (AES(RSN padding) + MIC) */
            aLength += (AES_AFTER_HEADER_FIELD_SIZE + MIC_FIELD_SIZE);
            aFlags |= WHA_AES;
            break;

        #ifdef GEM_SUPPORT
            case RX_DESC_ENCRYPT_GEM:
                ALIGN_HEADER_TO_DATA_BACKWARD (aFrame, GEM_AFTER_HEADER_FIELD_SIZE, bQos)
                shiftBytes += GEM_AFTER_HEADER_FIELD_SIZE + GEM_BUFFER_ALIGNMENT;
                /* Increase length size by 34 ( GEM_KeyIdx(1) + GEM_Reserved(1) + GEM_PN(16) + GEM_MIC(16) ) */
                aLength += (GEM_AFTER_HEADER_FIELD_SIZE + GEM_MIC_FIELD_SIZE);
                aFlags |= WHA_GEM;
                break;
        #endif

        default:
            /* Do nothing */
            break;
    }

	/*if QOS frame the pointer starts 2 bytes before so the shiftbytes will be 2 mor bytes*/
	if (bQos)
	{
		shiftBytes += QOS_SHIFT_SIZE;
	}



    /* Call upper layer */
    if( bErrorIndication == TI_FALSE)
    {    
        WhaCb()->ReceivePacket (aStatus, 
							aFrame, 
							aLength, 
							aRate, 
							aRcpi, 
							aChannel,
							(TUint8*)aBuf - shiftBytes,  
							aFlags);
    }
    #if TI_DBG
        else
        {
            WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
        }
    #endif	

    /* Decrease the number of allocated buffers */
    iRxPacketsAllocated--;

    TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "end of ReceivePacket(), allocated buffer = %d\n", iRxPacketsAllocated);
}


/** 
 * \fn     SendPacket
 * \brief  member function. 
 * 
 * \note    
 * \return  
 * \sa      
 */ 
WHA::TStatus TIWha::SendPacket(
								  const void* aFrame,
								  TUint16 aLength,
								  WHA::TQueueId aQueueId,
								  TUint8 aTxRateClassId,
								  WHA::TRate aMaxTransmitRate,
								  TBool aMore,
								  WHA::TPacketId aPacketId,
								  WHA::TPowerLevel aPowerLevel,
								  TUint32 aExpiryTime,
								  void* aReserved )
{    
    TTxCtrlBlk *pPktCtrlBlk;
    void* aData = (void*)aFrame;
    WHA::TStatus whaStatus;
    TI_STATUS tiStatus;

    TRACE4(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, " aFrame = 0x%x len = %d aQueue = %d rateClass = %d\n", aFrame, aLength, aQueueId, aTxRateClassId);

    TRACE3(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, " aMaxTransmitRate = 0x%x aPacketId = 0x%x aExpiryTime = %d \n",aMaxTransmitRate, aPacketId, aExpiryTime);
    

    AlignTxForTWD (aData, aLength);

    /* 
     * Prepare packet for transmision by filling up all the fields. 
     */
    
     whaStatus = preparePktCtrlBlk ( &pPktCtrlBlk,
                                     aData,
                                     aLength,
                                     aQueueId,
                                     aTxRateClassId,
                                     aMaxTransmitRate,
                                     aMore,
                                     aPacketId,
                                     aPowerLevel,
                                     aExpiryTime);

    if ( whaStatus == WHA::KQueueFull)
    {
        TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_WARNING , "****** KQueueFull ****** \n");
        AlignTxSecurityForUMAC (TRUE);
        return WHA::KQueueFull;
    }

   TRACE2(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "aFrame=%x, Buffer= %x \n", aFrame, pPktCtrlBlk->tTxnStruct.aBuf[0]);

    /*
     *  Call the Tx-Xfer to start packet transfer to the FW and return its result.       
     */

    tiStatus  = TWD_txXfer_SendPacket ( iTwdCtrl.hTWD, pPktCtrlBlk);

   TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, " status = %d \n", tiStatus);

    switch (tiStatus)
    {
        case TXN_STATUS_COMPLETE:
            /* TxXferCb () won't be called so Align Tx packet before */
            AlignTxSecurityForUMAC (TRUE);
            /* Mark that TxXferCb won't be called */
            pPktCtrlBlk->tTxPktParams.uFlags = TX_CTRL_FLAG_XFER_DONE_ISSUED;
            return WHA::KSuccessXfer;
        case TXN_STATUS_OK:
            pPktCtrlBlk->tTxPktParams.uFlags = 0;
            if (whaStatus == WHA::KPending) {
                return WHA::KPending;
            }
            return WHA::KSuccess;
        case TXN_STATUS_PENDING:
            /* The packet is pending in spi level and not in driver level */
            pPktCtrlBlk->tTxPktParams.uFlags = 0;
            if (whaStatus == WHA::KPending) {
                return WHA::KPending;
            }
            return WHA::KSuccess;
        case TXN_STATUS_ERROR:
            /* TxXferCb () won't be called so Align Tx packet before */
            AlignTxSecurityForUMAC (TRUE);
            return WHA::KQueueFull;
        default:
            /* TxXferCb () won't be called so Align Tx packet before */
            AlignTxSecurityForUMAC (TRUE);
            return WHA::KFailed;
    }
}

/** 
 * \fn     preparePktCtrlBlk
 * \brief  prepare the Ctrl Block with the given parameters
 * 
 * \note    
 * \return   txCtrlBlkEntry_t*
 * \sa      
 */ 
WHA::TStatus TIWha::preparePktCtrlBlk ( TTxCtrlBlk **pPktCtrlBlk,
                                       const void* aFrame,
                                       TUint16 aLength,
                                       WHA::TQueueId aQueueId,
                                       TUint8 aTxRateClassId,
                                       WHA::TRate aMaxTransmitRate,
                                       TBool aMore,
                                       WHA::TPacketId aPacketId,
                                       WHA::TPowerLevel aPowerLevel,
                                       TUint32 aExpiryTime)
{
    TI_UINT16 txDescAttr; /* Tx-descriptor fields values. */
    TI_UINT16 uLastWordPad;
	TI_UINT8  uAligmentPad;
    TI_UINT32 tempaFrame;    
    *pPktCtrlBlk = NULL;
    ETxHwQueStatus eHwQueStatus;

    /* Allocates a Control-Block for the packet Tx parameters and descriptor. */
    *pPktCtrlBlk = TWD_txCtrlBlk_Alloc (iTwdCtrl.hTWD);
    
    /* If null entry (not expected to happen) return ERROR. */
    if (!*pPktCtrlBlk)
    {
        TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR, ": Tx Ctrl-Blk allocation failed!!!\n");
        return WHA::KQueueFull;
    }

    /*  Save the aPacketId for use in TxXferCb & TxCompleteCb */
    iPacketIdTable[(*pPktCtrlBlk)->tTxDescriptor.descID] = aPacketId;

    TRACE2(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, ": iPacketIdTable[%d]= %d\n", (*pPktCtrlBlk)->tTxDescriptor.descID, aPacketId);

    /*
     *  Fill in parameters of  pPktCtrlBlk                                                                                   
     */
    /* Move aFrame back in order to leave room for the tx_descriptor */
    tempaFrame = (TI_UINT32)aFrame - TX_DESCRIPTOR_SIZE;
    aFrame = (void*)tempaFrame;
    
	uAligmentPad = ((4 -(aLength & 0x3)) & 0x3);
    /* data is given with the header, this is different then full driver  */
    (*pPktCtrlBlk)->tTxnStruct.aLen[0] =  aLength + TX_DESCRIPTOR_SIZE + uAligmentPad;
    (*pPktCtrlBlk)->tTxnStruct.aBuf[0] = (TI_UINT8*)aFrame;
    /* Not relevant since we are not incharge on the memory release */
    (*pPktCtrlBlk)->tTxPktParams.uInputPktLen = 0;
    (*pPktCtrlBlk)->tTxPktParams.pInputPkt = 0;

    /* 
     *  Build packet descriptor (for FW interface).  
     */
    (*pPktCtrlBlk)->tTxDescriptor.tid = WMEQosAcToTid[aQueueId];
    (*pPktCtrlBlk)->tTxDescriptor.startTime = os_timeStampMs (iTwdCtrl.tOsContext.hOsa);
    TranslateTimeToFw (&iTwdCtrl, (*pPktCtrlBlk), 2000/*aExpiryTime*/);

   TRACE3(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, ": aExpiryTime= %d, desc.startTime= %d, desc.lifeTime= %d \n", aExpiryTime, (*pPktCtrlBlk)->tTxDescriptor.startTime, (*pPktCtrlBlk)->tTxDescriptor.lifeTime);

    /* Initialize tTxDescriptor.length with the real packet length in bytes */
    (*pPktCtrlBlk)->tTxDescriptor.length = aLength + TX_DESCRIPTOR_SIZE;
    
     /* Call HwQueue for Hw resources allocation. If not available return NULL. */
    eHwQueStatus = TWD_txHwQueue_AllocResources (iTwdCtrl.hTWD, (*pPktCtrlBlk));
                   
    if (eHwQueStatus == TX_HW_QUE_STATUS_STOP_CURRENT)
    {
        /* Free Ctrl block. */
        TWD_txCtrlBlk_Free(iTwdCtrl.hTWD, (*pPktCtrlBlk));

        TRACE0(iTwdCtrl.hReport, REPORT_SEVERITY_WARNING, ": Tx Hw resources allocation failed!!!\n");
        return WHA::KQueueFull;
    }

    /* txCtrl_TranslateLengthToFw() will translate it to the length in FW format */
    uLastWordPad = TranslateLengthToFw((*pPktCtrlBlk));

    /* Upper layer is using '1' for first policy, while TnetwDrv is using '0' for it. */
    txDescAttr = (aTxRateClassId - 1 ) << TX_ATTR_OFST_RATE_POLICY;
    
    /* if there is a security padding then the WLAN packet was stuffed with 2 extra bytes for alignment */
    if (iSecurityPadding != 0) 
    {        
        txDescAttr |= TX_ATTR_HEADER_PAD;        
    }
    
    txDescAttr |= uLastWordPad << TX_ATTR_OFST_LAST_WORD_PAD;
        
    (*pPktCtrlBlk)->tTxDescriptor.txAttr = txDescAttr; 
    
    /*
     * Copy the descriptor parameters to the packet header.    
     */
    os_memoryCopy(iTwdCtrl.tOsContext.hOsa, const_cast<void*>(aFrame), (TxIfDescriptor_t*)&(*pPktCtrlBlk)->tTxDescriptor, sizeof(TxIfDescriptor_t));
    
    return eHwQueStatus == TX_HW_QUE_STATUS_STOP_NEXT ? WHA::KPending : WHA::KSuccess;    
}

/** 
 * \fn     AlignTxForTWD
 * \brief  Aligment header to TWD structure -- Check HT and Security. 
 * 
 * \note    
 * \return  
 * \sa      
 */ 
WHA::TStatus TIWha::AlignTxForTWD (void*& aFrame,TUint16& aLength)    
{
    /* Use the header to retrieve the frame ctrl of the packet */
    dot11_header_t  *pDot11Hdr = (dot11_header_t*)aFrame; 
    TI_UINT32 tempaFrame;

    /* By default we won't handle any alignment */
    iTwdCtrl.TxAlign.uHeaderToDataOffset[iTwdCtrl.TxAlign.uCurrSend] = 0;
	#ifdef HT_SUPPORT
		if (IS_HT_FRAME(pDot11Hdr->fc))
		{
		iTwdCtrl.TxAlign.uHeaderToDataOffset[iTwdCtrl.TxAlign.uCurrSend] = HT_CONTROL_FILED_SIZE;
		aLength -= HT_CONTROL_FILED_SIZE;
		}
	#endif /* HT_SUPPORT */
    	
	/* Qos heaser is needed to determine the WLAN Header size  */
    	iTwdCtrl.TxAlign.bIsQosHeader[iTwdCtrl.TxAlign.uCurrSend] = IS_QOS_FRAME(pDot11Hdr->fc);
    
    /* zero iSecurityPadding for every new packet */
      iSecurityPadding = 0;
    
    /* Check if using security. If so we should convert the packet format as the description below */
    if (IS_WEP_ON(pDot11Hdr->fc))
    {
    	TI_UINT8* pMac;
        ECipherSuite eSecurityMode;

        pMac = &pDot11Hdr->address1[0]; /* hold the first mac address */

    	/* Use group key on Multicast/broadcast. else use pairwise */
        if (MAC_MULTICAST(pMac))
        {
            eSecurityMode = iTwdCtrl.eGroupKeyMode;
        }
        else
        {
            eSecurityMode = iTwdCtrl.ePairwiseKeyMode;
        }
         
    	/*
    	 * The next adaptation code is meant to align with the driver legacy operation of extra padding for security:
    	 *
    	 * WEP: 	Upper layer		HEADER - IV (4) - DATA - ICV (4)
    	 *   		TnetwDrv	 	HEADER - DATA
    	 *
    	 * TKIP: 	Upper layer		HEADER - IV (4)  - EIV (4) - DATA - MIC (8) - ICV (4)
    	 *   		TnetwDrv	 	HEADER - EIV (4) - DATA 
    	 *
    	 * AES: 	Upper layer		HEADER - RSN (8) - DATA - MIC (8)
    	 *   		TnetwDrv	 	HEADER - RSN (8) - DATA
    	 */
    				 	
        TRACE4(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "SendPacket(): send = %d Handle = %d, QoS = %d, encryption = %d\n",iTwdCtrl.TxAlign.uCurrSend, iTwdCtrl.TxAlign.uCurrHandle, 
            iTwdCtrl.TxAlign.bIsQosHeader[iTwdCtrl.TxAlign.uCurrSend], eSecurityMode);

        switch (eSecurityMode) 
    	{
        case TWD_CIPHER_WEP:
            /* Mark the offset between header and data */
            iTwdCtrl.TxAlign.uHeaderToDataOffset[iTwdCtrl.TxAlign.uCurrSend] += WEP_AFTER_HEADER_FIELD_SIZE;
            /* Decrease length size by 8 (IV + ICV) */
            aLength -= (IV_FIELD_SIZE + ICV_FIELD_SIZE);
            break;

    	case TWD_CIPHER_TKIP:
            /* Mark the offset between header and data */
            iTwdCtrl.TxAlign.uHeaderToDataOffset[iTwdCtrl.TxAlign.uCurrSend] += IV_FIELD_SIZE;
            /* Decrease length size by 16 (IV + MIC + ICV) */
            aLength -= (IV_FIELD_SIZE + MIC_FIELD_SIZE + ICV_FIELD_SIZE);
            break;

    	case TWD_CIPHER_AES_WRAP:
    	case TWD_CIPHER_AES_CCMP:
    		/* Decrease length size by 8 (MIC) */
    		aLength -=  MIC_FIELD_SIZE;
    		break;
        #ifdef GEM_SUPPORT
            case TWD_CIPHER_GEM:
                /* Mark the offset between header and data */
                iTwdCtrl.TxAlign.uHeaderToDataOffset[iTwdCtrl.TxAlign.uCurrSend] += GEM_AFTER_HEADER_FIELD_SIZE;
                /* Decrease length size by 34 bytes (KEY_IDX_FIELD_SIZE + RESERVED_FIELD_SIZE + PN_FIELD_SIZE + MIC) */
        		aLength -=  (GEM_AFTER_HEADER_FIELD_SIZE + GEM_MIC_FIELD_SIZE);
                /* Add padding of two bytes before wlan header, */
                /* because it will be unaligned after the header will be moved 18 bytes */
                iSecurityPadding = GEM_AFTER_HEADER_PAD;
                break;
        #endif
        
    	default:
    		/* Unknown security with Encryption ON. return ERROR */
                TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR, "SendPacket(): Encryption ON but unknown security mode = %d\n",     			 eSecurityMode);
    		return WHA::KFailed;
    	}
    }

	/* Check if the header packet should move */
    if (iTwdCtrl.TxAlign.uHeaderToDataOffset[iTwdCtrl.TxAlign.uCurrSend] > 0)
    {
        ALIGN_HEADER_TO_DATA_FORWARD (aFrame, iTwdCtrl.TxAlign.uHeaderToDataOffset[iTwdCtrl.TxAlign.uCurrSend], 
                                     iTwdCtrl.TxAlign.bIsQosHeader[iTwdCtrl.TxAlign.uCurrSend])
    }

    /* Move the frame iSecurityPadding (2 bytes) backward */    
    tempaFrame = (TI_UINT32)aFrame - iSecurityPadding;
    aFrame = (void*)tempaFrame;
    /* Increase the length by size of iSecurityPadding (2 bytes) */
    aLength+= iSecurityPadding;
    /* Mark the current packet - Note that it is needed only when we have security */
    iTwdCtrl.TxAlign.pFrame[iTwdCtrl.TxAlign.uCurrSend] = (void*)aFrame;
    /* For the next packet to be send */
    
    iTwdCtrl.TxAlign.uCurrSend++;
    iTwdCtrl.TxAlign.uCurrSend %= MAX_XFER_WAITING_PACKETS;

    return WHA::KSuccess;
}


/** 
 * \fn     AlignTxSecurityForUMAC
 * \brief  security alignment. 
 *
 * Handle packet alignment after Xfer done
 *               It is done here after the pointer was changed in TIWha::SendPacket()
 *               and at this point the pointer is not needed any more.
 * \note    
 * \return  
 * \sa      
 */ 
void TIWha::AlignTxSecurityForUMAC (TBool aQFull)
{
    TRACE5(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "AlignTxSecurityForUMAC(): send = %d Handle = %d, QoS = %d, Offset = %d pFrame = %p\n",iTwdCtrl.TxAlign.uCurrSend, iTwdCtrl.TxAlign.uCurrHandle,iTwdCtrl.TxAlign.bIsQosHeader[iTwdCtrl.TxAlign.uCurrHandle], 
        iTwdCtrl.TxAlign.uHeaderToDataOffset[iTwdCtrl.TxAlign.uCurrHandle],iTwdCtrl.TxAlign.pFrame[iTwdCtrl.TxAlign.uCurrHandle]);

    if (aQFull) {
        
        iTwdCtrl.TxAlign.uCurrSend = iTwdCtrl.TxAlign.uCurrSend > 0? iTwdCtrl.TxAlign.uCurrSend - 1 : 49;
        TRACE5(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "AlignTxSecurityForUMAC() -- ** aQFull **: send = %d Handle = %d, QoS = %d, Offset = %d pFrame = %p\n",iTwdCtrl.TxAlign.uCurrSend, iTwdCtrl.TxAlign.uCurrHandle,iTwdCtrl.TxAlign.bIsQosHeader[iTwdCtrl.TxAlign.uCurrHandle], 
            iTwdCtrl.TxAlign.uHeaderToDataOffset[iTwdCtrl.TxAlign.uCurrHandle],iTwdCtrl.TxAlign.pFrame[iTwdCtrl.TxAlign.uCurrHandle]);

        ALIGN_HEADER_TO_DATA_BACKWARD (iTwdCtrl.TxAlign.pFrame[iTwdCtrl.TxAlign.uCurrSend], 
            iTwdCtrl.TxAlign.uHeaderToDataOffset[iTwdCtrl.TxAlign.uCurrSend], 
            iTwdCtrl.TxAlign.bIsQosHeader[iTwdCtrl.TxAlign.uCurrSend])
        return;
    }

    /* Check if the packet should be aligned */
    if (iTwdCtrl.TxAlign.uHeaderToDataOffset[iTwdCtrl.TxAlign.uCurrHandle])
    {
        ALIGN_HEADER_TO_DATA_BACKWARD (iTwdCtrl.TxAlign.pFrame[iTwdCtrl.TxAlign.uCurrHandle], 
            iTwdCtrl.TxAlign.uHeaderToDataOffset[iTwdCtrl.TxAlign.uCurrHandle], 
            iTwdCtrl.TxAlign.bIsQosHeader[iTwdCtrl.TxAlign.uCurrHandle])
    }

    /* For the next packet to be handled */
    iTwdCtrl.TxAlign.uCurrHandle++;
    iTwdCtrl.TxAlign.uCurrHandle %= MAX_XFER_WAITING_PACKETS;
}


/** 
 * \fn     TxXferCb
 * \brief  
 *    Called by the TNETW driver upon Xfer-Done of transmitted packet.
 *   Calls the UMAC Xfer-Done handler.
 *
 * \note    
 * \return  
 * \sa      
 */ 
void TIWha::TxXferCb ( TTxCtrlBlk *pPktCtrlBlk)
{
    TRACE2(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION,  "TxXferCb(): packetId=0x%x flags = 0x%x\n",iPacketIdTable[pPktCtrlBlk->tTxDescriptor.descID], pPktCtrlBlk->tTxPktParams.uFlags);
        
#ifdef TI_DBG
    /* If the pointed entry is already free, print error and exit (not expected to happen). */
    if (pPktCtrlBlk->pNextFreeEntry != NULL)
    {
        TRACE3(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "TxXferCb(): Pkt already free!!, DescID=%d, flags=%d, packetId=0x%x\n",pPktCtrlBlk->tTxDescriptor.descID, pPktCtrlBlk->tTxPktParams.uFlags,iPacketIdTable[pPktCtrlBlk->tTxDescriptor.descID]);
    
        return;
    }
#endif

    /* Check If this is the second time we call TxXferCb for this packet (i.e. TxCompleteCb called it explicitly) */
    if (pPktCtrlBlk->tTxPktParams.uFlags &  TX_CTRL_FLAG_XFER_DONE_ISSUED)
    {
        
        /* TRACE2(iTwdCtrl.hReport, REPORT_SEVERITY_ERROR,  TIWLANWHA_MODULE_LOG, MSG_2262, "TxXferCb(): already called for this packet!!, DescID=%d, flags=%d, packetId=0x%x\n",pPktCtrlBlk->tTxDescriptor.descID, pPktCtrlBlk->tTxPktParams.uFlagsiPacketIdTable[pPktCtrlBlk->tTxDescriptor.descID]); */        
    }
    else /* TxXferCB called for the first time. call UMAC */
    {
        pPktCtrlBlk->tTxPktParams.uFlags |= TX_CTRL_FLAG_XFER_DONE_ISSUED;
        
        AlignTxSecurityForUMAC ();

        TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "TxXferCb(): call WhaCb()->SendPacketTransfer, descID= %d\n", pPktCtrlBlk->tTxDescriptor.descID);
        TRACE1(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "TxXferCb(): call WhaCb()->SendPacketTransfer, PacketID= %d\n", iPacketIdTable[pPktCtrlBlk->tTxDescriptor.descID]);
       
        if( bErrorIndication == TI_FALSE)
        {    
            WhaCb()->SendPacketTransfer ((WHA::TPacketId)iPacketIdTable[pPktCtrlBlk->tTxDescriptor.descID]);
        }
        #if TI_DBG
            else
            {
                WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
            }
        #endif	
    }
}

/** 
 * \fn     TxCompleteCb
 * \brief      Called upon Tx-complete of transmitted packet.
 *    Handles it as follows:
 *    1) Update the HwQueue to free queue resources.
 *    2) Call the upper driver's tx-complete handler.
 *    3) Free the packet's Control-Block if Xfer-Done already occured.
 *
 * \note    
 * \return  
 * \sa      
 */ 
void TIWha::TxCompleteCb ( TxResultDescriptor_t *pTxResultInfo, TI_UINT32 backpressure)
{
    /* Get the packet's control block pointer by the descId index. */
    TTxCtrlBlk *pPktCtrlBlk = TWD_txCtrlBlk_GetPointer(iTwdCtrl.hTWD, pTxResultInfo->descID);
    WHA::TStatus status;
    
    TRACE3(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "TxCompleteCb(): DescID=%d, flags=%d, packetId=0x%x\n", 
        pTxResultInfo->descID, pPktCtrlBlk->tTxPktParams.uFlags, 
        iPacketIdTable[pPktCtrlBlk->tTxDescriptor.descID]);

#ifdef TI_DBG
    /* If the pointed entry is already free, print error and exit (not expected to happen). */
    if (pPktCtrlBlk->pNextFreeEntry != NULL)
    {
       TRACE3(iTwdCtrl.hReport, REPORT_SEVERITY_INFORMATION, "TxCompleteCb(): Pkt already free!!, DescID=%d, flags=%d, packetId=0x%x\n", 
            pPktCtrlBlk->tTxDescriptor.descID, pPktCtrlBlk->tTxPktParams.uFlags, 
            iPacketIdTable[pPktCtrlBlk->tTxDescriptor.descID]);
       
        return;
    }
#endif


    /* Check if TxXferCb was called for this packet */
    if ((pPktCtrlBlk->tTxPktParams.uFlags & TX_CTRL_FLAG_XFER_DONE_ISSUED) == 0)
    {
        /* Xfer was not  called, so call it here. Note that once TxComplete was called, there will be no TxXfer */
        TxXferCb (pPktCtrlBlk);
    }

    /* We return Success on every tx result because of stange behaviour in the upper layer */
    /* It seems that on results other than success the OS closes the socket */
    switch (pTxResultInfo->status)
    {
        case TX_SUCCESS:
            status = WHA::KSuccess;
            break;
        case TX_RETRY_EXCEEDED:
            status = WHA::KErrorRetryExceeded;
            WLAN_OS_REPORT(("ERROR: TxCompletewith status = %d",pTxResultInfo->status));
            break;
        case TX_TIMEOUT:
            status = WHA::KErrorLifetimeExceeded;
            WLAN_OS_REPORT(("ERROR: TxCompletewith status = %d",pTxResultInfo->status));
            break;
        case TX_PEER_NOT_FOUND:
            status = WHA::KErrorNoLink;
            WLAN_OS_REPORT(("ERROR: TxCompletewith status = %d",pTxResultInfo->status));
            break;
        case TX_DISABLED:
            status = WHA::KErrorMacNotResponding;
            WLAN_OS_REPORT(("ERROR: TxCompletewith status = %d",pTxResultInfo->status));
            break;
        default:
            status = WHA::KFailed;
            WLAN_OS_REPORT(("ERROR: TxCompletewith status = %d",pTxResultInfo->status));
            break;
    }

    /* Call the UMAC tx-complete handler */
    if( bErrorIndication == TI_FALSE)
    {    
        WhaCb()->SendPacketComplete(     status,
								(WHA::TPacketId)iPacketIdTable[pPktCtrlBlk->tTxDescriptor.descID],
								TIWhaUtils::PolicyToWhaRate ((ETxRateClassId)pTxResultInfo->rate),
								pTxResultInfo->mediumDelay, /* aPacketQueueDelay */
								pTxResultInfo->fwHandlingTime, /* aMediaDelay */
			 					pTxResultInfo->ackFailures);
    }
    #if TI_DBG
        else
        {
            WLAN_OS_REPORT(("%s : CommandResponse Block call to LDD response \n",__FUNCTION__));
        }
    #endif	

    /* Free Ctrl block. */
    TWD_txCtrlBlk_Free(iTwdCtrl.hTWD, pPktCtrlBlk);
}


