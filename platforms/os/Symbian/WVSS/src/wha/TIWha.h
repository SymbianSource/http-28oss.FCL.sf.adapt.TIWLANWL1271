/*
 * TIWha.h
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



/** \file  TIWha.h 
 *  \brief  Interface between the sandbox to the BSP of Omap2430
 *
 *  \see   
 */


#ifndef TIWha_H
#define TIWha_H

/* Includes that are rquired for using MWlanDfcClient */
#include <kernel/kernel.h>
#include <wlandfc.h>
#include <wlandfcclient.h>
#include <wlanpddiface.h>

/* Other driver includes*/
#include "TIWhaDef.h" /*struct and definitions neded for the TI WLAN driver*/
#include "TWDriver.h" /*interface with the HW abstraction of the WLAN driver*/
#include "TIWhaUtils.h"
#include "TIWlanHpaCB.h"

extern void Assert( const TInt8*, TInt, TBool );


/*
 * This is a class for Failure Indication DFC implementation
 */
class TIFailureDfcClient : public MWlanDfcClient
{
    public:        

        /** 
    	* \fn     TIFailureDfcClient
    	* \brief  Constructor
    	*
        * Create a MWlanDfc object.
    	* 
    	* \note
    	* \param  	aOsa
    	* return   handle to TIFailureDfcClient class.
    	* \sa
    	*/
        TIFailureDfcClient(MWlanOsa& aOsa);

        /** 
    	* \fn     OnDfc
    	* \brief  Call TIWhaAdaptCB::FailureIndicationDFC in order to handle the failure indication
        * from a different context
    	* 
    	* \note
    	* \param   aCtx - Relevant Context
    	* return   void
    	* \sa
    	*/
        virtual void OnDfc( TInt aCtx ); 

        /* A pointer to the relevant wlan DFC object */
        MWlanDfc* pFailureDfc;
};



/*
 * This is a class for Sync bus connection DFC implementation
 */
class TIConnectDfcClient : public MWlanDfcClient
{
    public:

        /** 
    	* \fn     TIConnectDfcClient
    	* \brief  Constructor
    	*     	
        * Create a MWlanDfc object.
    	* 
    	* \note
    	* \param  	aOsa
    	* return   handle to TIConnectDfcClient class.
    	* \sa
    	*/
        TIConnectDfcClient(MWlanOsa& aOsa);


        /** 
    	* \fn     OnDfc
    	* \brief  Call TIWhaAdaptCB::ConnectBus() in order to finish the bus connection procees
    	* 
    	* \note
    	* \param  	aCtx - Relevant Context
    	* return   void
    	* \sa
    	*/
        virtual void OnDfc( TInt aCtx ); 

        /* A pointer to the relevant wlan DFC object */
        MWlanDfc* pConnectDfc;
};



/**
* This is a class for TI WLAN driver implementation
*/
class TIWha :     public WHA::Wha , public DPhysicalDevice , public MWlanPddIface
{
public:  

	/** 
	* \fn     TIWha
	* \brief  Constructor
	* 
	*  TIWha constructor
	* 
	* \note   
	* \param  	aOsa
	* \param  aHpa
	* \param  aSpia
	* return   handle to TIWha class. 
	* \sa     
	*/ 
	TIWha(MWlanOsa& aOsa, WlanHpa& aHpa, WlanSpia& aSpia);


			/** 
	* \fn     TIWha
	* \brief  Constructor
	* 
	*  TIWha constructor
	* 
	* \note   
	* 
	* return   handle to TIWha class. 
	* \sa     
	*/ 
    TIWha();

	/** 
	 * \fn     ~TIWha
	 * \brief  destructor
	 * 
	 * Destroy TIWha object
	 * 
	 * \note   
	 * \param  
	 * \return   
	 * \sa     
	 */ 
	virtual ~TIWha();


  

	/* Call constructor */
	static Wha* Create( MWlanOsa& aOsa, 
	                    WlanHpa& aHpa, 
	                    const SHwBusAccessLayer& aTransPortLayer );

	/* Call destructor */
	static void Destroy( Wha* aWha );


	/** 
	 * \fn     WhaCb
	 * \brief  attach call back to LDD form the WLAN driver
	 * 
	 * \note   	in case error indication was received - return Null. That way we avoid calls to UMAC.
	 * \param  
	 * \return pointer to the Call Back function in the WHA (LDD)  
	 * \sa     
	 */ 	
    WHA::MWhaCb* WhaCb() {return iWhaCb;}


	/** 
	* \fn     FailureIndicationCb
	* \brief  Generic Error Indicatin function to the UMAC
	* 
	* sends the UMAC Error indications as agreed.
	* 
	* \note   
	* \param EFailureEvent  - the kind of error
	* \return 
	* \sa     
	*/         
	void FailureIndicationCb ( EFailureEvent failureEvent);    
    	

    /** 
     * \fn     ConnectionTimeOut
     * \brief  change rate management parameters after the connection phase
     * 
     * \note    
     * \return  
     * \sa      
     */    
    void ConnectionTimeOut ();

    /** 
     * \fn     SetTxFailLowThreshold
     * \brief  Set new Tx Fail rate to FW
     * 
     * \note    
     * \return  
     * \sa      
     */    
    void SetTxFailLowThreshold( TI_UINT8 uTxFailLowTh);
	
   /** 
 * \fn     InitializeAfterTimer
 * \brief  start second part of initialization after timer expired
 *          
 * \note    
 * \param  
 * \return  
 * \sa      
 */ 
    void InitializeAfterTimer();
	
    /** 
     * \fn     RxMemFailTimerCb
     * \brief  call RxXfer in case memory allocation failed, so we will try again
     *          
     * \note    
     * \param  
     * \return  
     * \sa      
     */ 
    void RxMemFailTimerCb();

/*******************************************************************************
 *                                                                             *
 * WLAN Symbian HAL API command methods below.                                     *
 *                                                                             *
 *******************************************************************************/

    /** 
     * \fn     Initialize
     * \brief  Downloads the firmware code to the WLAN device
     * (reference wlanwha.h) 
     *          
     * \note    
     * \param  aData firmware data
     * \param  aLength length of the data in bytes
     * \return  
     * \sa      
     */ 
	void Initialize(
				   const void* aData, 
				   TUint32 aLength );

    /** 
     * \fn     Configure
     * \brief  Method configures the WLAN device after the WHA layer has send the EInitializeResponse event.
     * (reference wlanwha.h) 
     *          
     * \note    
     * \param  aData firmware data. The content is vendor specific.
     * \param  aWhaSettings output data that holds the capabilities
     * \return  
     * \sa      
     */ 
	void Configure(
		const WHA::SConfigureData&, 
		WHA::SSettings& aWhaSettings );

    /** 
     * \fn     Configure
     * \brief  Command will cause the WHA layer to release all OS resources and stop
	 * its operation. (reference wlanwha.h) 
     *          
     * \note    
     * \param  aSynchronous should command be executed synchronously (ETrue) or not (EFalse) 
     * \return Command was executed synchronously and no command response event is sent
     * \sa      
     */ 
 	WHA::TStatus Release( TBool aSynchronous );

    /** 
     * \fn     Scan
     * \brief  When the WLAN device receives this command, it goes into a scanning mode.
     * (reference wlanwha.h) 
     *          
     * \note    
     * \param  aMaxTransmitRate specifies the transmission rate of the probe request in case of a active scan
     * \param  aBand selects the used frequency band. 
     * \param aNumOfChannels number of channels provided in the command
	 * \param aChannels specifies the scanned channels
	 * \param aScanType specifies the scan type:0 foreground scan, 1 background scan, 2 forced background scan
	 * \param aNumOfProbeRequests number of probe requests (per SSID): sent to one (1) channel, Zero (0) means that none is send, 
	 * which means that a passive scan is to be done.  
	 * \param aNumOfSSID number of SSID provided in the scan command (this is zero (0) in broadcast scan)
	 * \param aSsid Array of the SSID to be probed in scan
     * \return 
     * \sa      
     */ 
	void Scan(  WHA::TRate                  aMaxTransmitRate, 
				WHA::TBand                  aBand,
				TUint8                      aNumOfChannels,
				const WHA::SChannels*   aChannels, 
				WHA::TScanType          aScanType,
				TUint8                      aNumOfProbeRequests,
				TBool                       aSplitScan,
				TUint8                      aNumOfSSID,
				const WHA::SSSID*       aSsid);

     /** 
     * \fn     StopScan
     * \brief  This method stops a previously started scan process in the WLAN device.
     * (reference wlanwha.h) 
     *          
     * \note    
     * \param 
     * \return
     * \sa      
     */ 
   void StopScan();

    /** 
    * \fn     Join
    * \brief  The WLAN host driver uses this method to command the WLAN device to 
	* join a BSS or an IBSS or to start an IBSS.(reference wlanwha.h) 
    *          
    * \note    
    * \param aMode specifies the operation mode of the station: 0 for IBSS, 1 for BSS
	* \param aBSSID specifies the BSSID of the BSS or IBSS to be joined or the IBSS to be started
	* \param aSSID specifies the SSID of the IBSS to join or start
	* \param aBand selects the used frequency band. Only 1 bit is used to select the band
	* \param aChannel specifies the channel number to join.
    * \param aBeaconInterval specifies the time between TBTTs in TUs
	* \param aBasicRateSet defines the BSS basic rate set
	* \param aAtimWindow ATIM window of IBSS
    * \param aPreambleType Specifies the PLCP preamble type used: 0 for long preamble, 1 for short preamble
    * \param aProbeForJoin specifies if a probe request should be send with the specified SSID when joining to the network. 
    * \return
    * \sa      
    */ 
	void Join(
			 WHA::TOperationMode aMode,
			 const WHA::TMacAddress& aBSSID,
			 const WHA::SSSID& aSSID, 
			 WHA::TBand aBand,
			 WHA::TChannelNumber aChannel,
			 TUint32 aBeaconInterval,
			 WHA::TRate aBasicRateSet,
			 TUint16 aAtimWindow,
			 WHA::TPreamble aPreambleType,
			 TBool aProbeForJoin );

    /** 
    * \fn     SetPsMode
    * \brief  The WLAN host driver uses this method to manipulate the WLAN devices 
	* 802.11 power management mode in infrastructure mode.(reference wlanwha.h) 
    *          
    * \note    
    * \param aPsMode desired 802.11 power management mode:0 - 802.11 power save disable, 1 - 802.11 power save enable
    * \return
    * \sa      
    */ 
	void SetPsMode( WHA::TPsMode aPsMode );

    /** 
    * \fn     SetBssParameters
    * \brief  The WLAN host driver uses this method to fix connection parameters 
	* after the initial connection setup (reference wlanwha.h) 
    *          
    * \note    
    * \param aDTIM specifies the DTIM interval in multiples of beacons
    * \param aAID specifies the AID received during the association process
    * \return
    * \sa      
    */ 
	void SetBssParameters(
						 TUint8 aDTIM, 
						 TUint16 aAID );

    /** 
    * \fn     Measure
    * \brief  This method starts radio measurements in the WLAN device.
    * (reference wlanwha.h) 
    *          
    * \note    
    * \param aTxPowerLevel used transmission power level during the measurement process. 
	* \param aBand selects the used frequency band. Only 1 bit is used to select the band
	* \param aChannel specifies the channel number.
    * \param aActivationDelay number of TBTTs until interval specified by the aMeasurementOffset starts.
    * \param aMeasurementOffset Time after the activation delay in TUs
	* \param aNumberOfMeasurementTypes number of measurement types to activate at parallel
	* \param aParameterSet specific measurement parameter set
    * \return
    * \sa      
    */ 
	void Measure(
				WHA::TPowerLevel aTxPowerLevel,
				WHA::TBand aBand,
				WHA::TChannelNumber aChannel,
				TUint8 aActivationDelay,
				TUint8 aMeasurementOffset,
				TUint8 aNumberOfMeasurementTypes,
				const WHA::SParameterSet* aParameterSet );

    /** 
    * \fn     StopMeasure
    * \brief  This method stops all previously started measurement processes in the WLAN device. 
    * (reference wlanwha.h) 
    *          
    * \note    
    * \param 
    * \return
    * \sa      
    */ 
	void StopMeasure();

    /** 
    * \fn     ReadMib
    * \brief  The WLAN host driver uses this method to read configuration 
	* information and statistics from the WLAN vendor specific solution. 
    * (reference wlanwha.h) 
    *          
    * \note    
    * \param aMib ID of the MIB to be accessed
    * \return
    * \sa      
    */ 
 	void ReadMib( WHA::TMib aMib );

    /** 
    * \fn     WriteMib
    * \brief  The WLAN host driver uses this method to write configuration 
	* information to the WLAN vendor specific solution
    * (reference wlanwha.h) 
    *          
    * \note    
    * \param aMib ID of the MIB to be written
	* \param aLength length of the MIB
	* \param aData pointer to the MIB data
	* \param aMore more MIBS are written after this command (ETrue) or not (EFalse) 
	* \return ESuccess: Command was executed synchronously and no command response event is sent
	*         EPending: Command is executed asynchronously and corresponding command response event 
    *                   will be sent upon command completion
    * \sa      
    */ 
	WHA::TStatus WriteMib(
						 WHA::TMib aMib,
						 TUint16 aLength,
						 const void* aData,
						 TBool aMore  );

    /** 
    * \fn     AddKey
    * \brief  This method adds a new (or replaces an old) encryption key to WLAN device
    * (reference wlanwha.h) 
    *          
    * \note    
    * \param aType type of the key to be added
	* \param aKey pointer to buffer specifying the key material, 
	* \param aEntryIndex
	* \return 
    * \sa      
    */ 
	void AddKey(
			   WHA::TKeyType aType, 
			   const void* aKey,
			   TUint8 aEntryIndex );

    /** 
    * \fn     RemoveKey
    * \brief  This method removes encryption keys from active key set. 
    * (reference wlanwha.h) 
    *          
    * \note    
    * \param aEntryIndex index of the key to remove from the set, Valid range: 0-8
    * \return 
    * \sa      
    */ 
	void RemoveKey( TUint8 aEntryIndex );

    /** 
    * \fn     ConfigureQueue
    * \brief  The WLAN host driver uses this method to configure QoS parameters of transmission queues 
    * (reference wlanwha.h) 
    *          
    * \note    
    * \param aQueueId ID for the queue
    * \param aMaxLifeTime dot11MaxTransmitMsduLifetime to be used for the specified queue. 
    * \param aPsScheme PS scheme of the specified queue: 0 Regular PS, 1 U-APSD, 2 Legacy PSPOLL,3 S-APSD [optional]
    * \param aSAPSDConfig
    * \param aQueueId ID for the queue
    * \param aAckPolicy ACK frame policy of the specified queue: 0 - normal, 1 - Tx no ACK [optional],2 - block ACK [optional]
    * \return 
    * \sa      
    */ 
	void ConfigureQueue(
					   WHA::TQueueId aQueueId,
					   TUint32 aMaxLifeTime,
					   WHA::TPsScheme aPsScheme,
					   const WHA::SSAPSDConfig& aSAPSDConfig,
					   WHA::TAckPolicy aAckPolicy,
                       TUint16 aMediumTime );

    /** 
    * \fn     ConfigureAC
    * \brief  The WLAN host driver uses this method to configure EDCA Access Category parameters.
    * (reference wlanwha.h) 
    *          
    * \note    
    * \param aCwMin CWmin (in slots) for the access class
    * \param aCwMax CWmax (in slots) for the access class
    * \param aAIFS AIFS value (in slots) for the access class
    * \param aTxOplimit Tx Op Limit (in microseconds) for the access class
    * \param aMaxReceiveLifeTime dot11MaxReceiveLifeTime to be used for the specified queue.  
    * \return 
    * \sa      
    */ 
	void ConfigureAC(
					TUint16 aCwMin[Wha::KNumOfEdcaQueues],
					TUint16 aCwMax[Wha::KNumOfEdcaQueues],
					TUint8 aAIFS[Wha::KNumOfEdcaQueues],
					TUint16 aTxOplimit[Wha::KNumOfEdcaQueues],
					TUint16 aMaxReceiveLifeTime[Wha::KNumOfEdcaQueues] );

    /** 
    * \fn     SendPacket
    * \brief  The WLAN host driver uses this method to send packets across the interface. 
    * (reference wlanwha.h) 
    *          
    * \note    
    * \param aFrame pointer to the frame (MSDU) content
    * \param aLength MSDU length.
    * \param aQueueId transmit queue as defined in ConfigureQueue
    * \param aTxRateClassId transmit rate class ID defined in txRatePolicy MIB. 
    * \param aMaxTransmitRate defines the highest transmission rate to be used   
    * \param aMore informs the WHA layer if an another packet is pending for transmission in the same context  
    * \param aPacketId packet identifier   
    * \param aPowerLevel transmission power level 
    * \param aExpiryTime the elapsed time in TUs  
    * \param aReserved reserved field  
    * \return ESuccess: Packet was accepted for delivery to WLAN device. 
    *  ESuccessXfer: Packet and all the (possible) other packets queued inside the WHA layer were transferred to the WLAN device.
    *  EQueueFull: The designated transmit queue is full and the packet was not accepted for delivery to WLAN device. 
    * \sa      
    */ 
	WHA::TStatus SendPacket(
						   const void* aFrame,
						   TUint16 aLength,
						   WHA::TQueueId aQueueId,
						   TUint8 aTxRateClassId,
						   WHA::TRate aMaxTransmitRate,
						   TBool aMore,
						   WHA::TPacketId aPacketId,
						   WHA::TPowerLevel aPowerLevel,
						   TUint32 aExpiryTime,
						   void* aReserved );

    /** 
    * \fn     Reset
    * \brief  The WLAN host driver uses this command to reset the WLAN vendor specific solution to its initial state
    * (reference wlanwha.h) 
    *          
    * \note    
    * \param   
    * \return 
    * \sa      
    */ 
	void Reset();

    /** 
    * \fn     Plt
    * \brief  The host driver calls this method to perform production line testing of the WLAN device
    * (reference wlanwha.h) 
    *          
    * \note    
    * \param aType indicates which test to perform  
    * \param aParams test specific parameters 
    * \return 
    * \sa      
    */ 
    void Plt (WHA::TPltType aType, void *aParams);
    

	/**
	 * Direct read/write operation from the device 
	 */
	WHA::TStatus WriteMem (TMemoryAccess *pMemoryAccess);
	WHA::TStatus ReadMem (TMemoryAccess  *pMemoryAccess);

	/**********************************************/
	/************ TIWha callbacks **************/
	/**********************************************/

    /* ConnectBusCb called after first connection phase (init command transaction) is finished*/
    void ConnectBusCb (TI_STATUS status); 
	/* Init HW Callback called after all software init is done  and NVS configuration was saved as defaults, will download the FW*/
	void InitHwCb ( TI_STATUS status);
	/* Init FW Callback called after Fw was downloaded will configure the FW*/
	void InitFwCb ( TI_STATUS status);
	/* Config FW Callback will set the defaults*/
	void ConfigFwCb ( TI_STATUS status);
    /* InitFailCb will be called in case of a failure in one of the init SM's */
	void InitFailCb ( TI_STATUS status);

	/**********************************************/
	/************ Complete callbacks **************/
	/**********************************************/

	/* Complete Callback */
	void ScanCompleteCb (TI_STATUS returnStatus , E80211PsStatus PSMode);

	/* Power Save Complete Callback */
	void SetPsModeCompleteCb (TUint8 PSMode, TUint8 transStatus);

	/* Measurement Complete Callback */
	void MeasureCompleteCb               ( TMeasurementReply *pMsrReply);


	/**********************************************/
	/*********** Indication callbacks *************/
	/**********************************************/
	/* RCPI Indication Callback */
	void RcpiIndicationCb                ( TUint8* buffer, TUint32 len);
	/* Lost Beacon Indication Callback */        
	void LostBssIndicationCb             ();
	/* Regain Bss Indication Callback */        
	void RegainBssIndicationCb           ();
	/* BtCoexistence Indication CallBacks */
	void btCoexSenseIndicationCb         ( TUint8* buffer, TUint32 len);
	void btCoexProtectiveIndicationCb    ( TUint8* buffer, TUint32 len);
	void btCoexAvalancheIndicationCb     ( TUint8* buffer, TUint32 len);
    /* FW statistics callback */
    void TWD_StatisticsReadCB (TI_HANDLE hTWD, TI_UINT16 MboxStatus, ACXStatistics_t* pElem);
	/*Avr RSSI Callback*/
	void TWD_AvrRssiReadCB (TI_HANDLE hCb, TI_UINT16 MboxStatus, ACXRoamingStatisticsTable_t* roamingStatistics);
	/**********************************************/
	/************ Response callbacks **************/
	/**********************************************/

	/* Generic Response Callback */
	void GenericCommandResponseCb ( TUint16 CmdType, TUint16 CmdID, TUint32 aStatus);
	/* Scan Response Callback */
	void ScanResponseCb(WHA::TStatus aStatus);
	/* Join Response Callback */
	void JoinCompleteCb ();

	/* Add Key Response Callback */
	void AddKeyResponseCb                ( WHA::TStatus aStatus);
	/* Remove Key Response Callback */
	void RemoveKeyResponseCb             ( WHA::TStatus aStatus);
	/* Stop Scan Response Callback */
	void StopScanResponseCb (WHA::TStatus aStatus);

	/* Power Save Response Callback */
	void SetPsModeResponseCb (TUint8 aStatus);

	/* Read MIB Response Callback */
	void ReadMIBResponseCb               ( TUint16 aStatus,  void *InterrogateParamsBuf);
	/* TxPower Level Read MIB CallBack */
	void ReadMIBStationIdResponse      (TUint16 aStatus, void *InterrogateParamsBuf);
	void ReadMIBstatisticsTableResponse(TUint16 aStatus,  void *InterrogateParamsBuf);
	void ReadMIBcountersTableResponse  (TUint16 aStatus,  void *InterrogateParamsBuf);
	void ReadMIBBtCoexParamsResponse   (TUint16 aStatus,  void *InterrogateParamsBuf);
	/* Read Version Callback */
	void ReadMIBVersion                (TUint16 aStatus,  void *InterrogateParamsBuf);
	/* Measure Response Callback */
	void MeasureResponseCb               ( TUint16 aStatus);
	/* Stop Measure Response Callback */
	void StopMeasureResponseCb           ( TUint16 aStatus);
	/* Command Complete Generic Callback */


	/* PLT Response Callback */
	void PltResponseCb                   ( TI_STATUS aStatus,  void *InterrogateParamsBuf);
	/* PLT RX PER callbacks */
	void PltPerGetResultResponseCb       ( TUint16 aStatus,  void *InterrogateParamsBuf);
	void PltPerStartResponseCb           ( TUint16 aStatus,  void *InterrogateParamsBuf);
	void PltPerStopResponseCb            ( TUint16 aStatus,  void *InterrogateParamsBuf);
	void PltPerClearResponseCb           ( TUint16 aStatus, void *InterrogateParamsBuf);
	/* Read Memory/Registers Response Callback */
	void ReadMemResponseCb          (TFwDebugParams* params);
	/* Write Registers Response Callback */
	void WriteMemResponseCb         (TFwDebugParams* params);

	/* PLT TX calibration */
	void PltGainGetResponseCb            ( TUint16 aStatus,  void *InterrogateParamsBuf);
	void PltGetNVSUpdateBufferResponseCb ( TUint16 aStatus,  void *InterrogateParamsBuf);

#ifdef PLT_TESTER
	TUint8 iRealPlt;
	void PltSm(void *pBuf);

#endif
	/* RX CallBacks */
    ERxBufferStatus RequestForBufferCb (void **pWbuf, TI_UINT16 aLength, TI_UINT32 uEncryptionflag);
	
	void ReceivePacketCb (const void *aFrame);


	/* Frame was written to FW */
	void TxXferCb(TTxCtrlBlk *pPktCtrlBlk);

	/* Frame was transmitted over the air (or failed to be transmitted) */
	void TxCompleteCb (TxResultDescriptor_t *pTxResultInfo, TI_UINT32 backpressure);

	/**
	* \fn     InitResponse
	* \brief ititializatin response function. In case of success - return command response
	*								In case of error - send error indication (as requested from Palau)
	*
	* /note
	* 
	* /param aTwdInitStatus - Whether Init was successful
	*   
	* /return 
	*/
	void InitResponse                  (WHA::TStatus aTwdInitStatus);


	/**
	* \fn     getMacAddress
	* \brief  return the Mac address from the TWDCtrl
	*
	* 
	* /note
	* 
	* /return 
	*/
	void * getMacAddress();


    /**
	* \fn     ConvertTxBip2Nvs
	* \brief  Fill in the Tx NVS params in iBipNvsBuffer
	*
	* 
	* /note
	* 
	* /return 
	*/
	void ConvertTxBip2Nvs(void* pBuf);


    /**
	* \fn     ConvertRxBip2Nvs
	* \brief  Fill in the Rx NVS params in iBipNvsBuffer
	*
	* 
	* /note
	* 
	* /return 
	*/
    void ConvertRxBip2Nvs(void* pBuf);

private:

    /**
	* Internal PLT to match the TWD test API
	* /param eTestCmd indicates which test to perform
	* /param pTestCmdParams test specific parameters
	* /return 
	*/

    WHA::TStatus Plt (ETestCmdID eTestCmd, void *pTestCmdParams);
    

#ifdef TI_TEST
	void PltTester(const void *aData);
	void ChangeQueue(const void *aData);
#endif /* TI_TEST */


	/*Helper function*/

	TI_STATUS CreateDriver();
	void DestroyDriver();

    	
	/**
	* \fn     InitTwdParamTable
	* \brief  initialization of TWD parameters
	*
	* since we have no ini file, we are initializing the 
	* Modules with hard coded values
	* /note
	* 
	* /return 
	*/
	void             InitTwdParamTable   ();
    /* FEM Auto Detection*/
    void             InitTwdRadioParam   ();
    void             FillRadioData();
    void             FillSmartReflexData();
    void             InitTwdPlatformGenParam();
    #ifdef TI_DBG
       void             PrintRadioData();
    #endif

    void             InitTwdRadioSmartReflexParam();


	/**
	* \fn     InitBtCoex
	* \brief  initialization of Soft Gemini parameters
	*
	* since we have no ini file, we are initializing the 
	* Modules with hard coded values
	* /note
	* 
	* /return 
	*/
	void 			InitBtCoex();

	/**
	* \fn     InitReportParamTable
	* \brief  initialization of Report parameters
	*
	* since we ave no ini file, we are initializing the 
	* Modules wit hard coded values
	* /note
	* 
	* /return 
	*/
	void             InitReportParamTable();       
	/* Open reports for debug */
	void 	OpenAllReports();

	/** 
	* \fn     RegisterCb
	* \brief  registers the TIWha callbacks
	* 
	* Register callbacks To Twd level according to the action
	* that we would like to be triggered on 
	* 
	* \note    
	* \return  status whther the registration succeed or not.
	* \sa      
	*/
	WHA::TStatus     RegisterCb                ();


	/** 
	* \fn     RegisterEvents
	* \brief  register the TIWha Events
	* 
	* Register callbacks To Twd level according to the event 
	* that we would like to be triggered on 
	* 
	* \note    
	* \return  status whther the registration succeed or not.
	* \sa      
	*/
	WHA::TStatus     RegisterEvents        ();


	/**
	* \fn     FillNWSASettings
	* \brief  Fill UMAC structure with correct settings
	*
	* since we ave no ini file, we are initializing the 
	* Modules wit hard coded values
	* /note
	* 
	* /param SNWSASettings - pointer pointer to the UMAC settings struct
	* /return status
	*/
	WHA::TStatus     FillNWSASettings      (WHA::SSettings *SNWSASettings);

	/**
	* \fn     WriteMib utility functions
	* \brief  For each WriteMib command we use one of the following functions
	*
	* /return status
	*/
	WHA::TStatus SetSlotTime (const void *aData);
	WHA::TStatus Dot11MaxReceiveLifeTime (const void *aData) ;
	WHA::TStatus SetRxFilter (const void *aData);
	WHA::TStatus SetCurrentTxPowerLevel (const void *aData);
	WHA::TStatus SetTemplateFrame (const void *aData);
	WHA::TStatus SetDot11GroupAddrTable (const void *aData);           
	WHA::TStatus SetDefaultKeyID (const void *aData); 
	WHA::TStatus Dot11RTSThreshold (const void *aData);
	WHA::TStatus CtsToSelf (const void *aData);
	WHA::TStatus SetArpIpAddrTable (const void *aData);
	WHA::TStatus SetBeaconFilterIETable (const void *aData);
	WHA::TStatus SetBeaconFilterEnable (const void *aData);
	WHA::TStatus SleepMode ( const WHA::TSleepMode&       aSleepMode, TBool       bResponse);
	WHA::TStatus WakeUpConditions (const void *aData);
	WHA::TStatus SetBeaconLostCount (const void  *aData);
	WHA::TStatus SetRcpiThreshold (const void *aData);
	WHA::TStatus TxRatePolicy (TUint32     aLength, const void *aData); 
	WHA::TStatus btCoexistenceMode(TUint32 aLength, const void *aData);
        WHA::TStatus btCoexistenceProfile(TUint32 aLength, const void *aData);
#ifdef HT_SUPPORT
	void         TConvertTwdHtCapa2SHtCapa (TTwdHtCapabilities* pTwdHtCapabilities,WHA::SHtCapabilities* pHtCapabilities);
	WHA::TStatus SetHTCapabilities (WHA::ShtCapabilities* pApCapabilities);
	WHA::TStatus SetHTInformation (WHA::ShtBssOperation* pHtInformarion);
	WHA::TStatus ConfigureBA (WHA::ShtBlockAckConfigure* pBABitMask);	
#endif
    WHA::TStatus TxAutoRatePolicy (WHA::StxAutoRatePolicy* pTxAutoRatePolicy);
	void 		 SetPowerSavePowerLevel(void *aMib);
	TI_STATUS SoftGemini_SetParams (TI_HANDLE hTWD, TSoftGeminiParams *pSgParams,ESoftGeminiEnableModes aSgMode );
	
	void prepareNextFwChunk ();

	WHA::TStatus preparePktCtrlBlk ( TTxCtrlBlk **pPktCtrlBlk,
                                              const void* aFrame,
            								  TUint16 aLength,
            								  WHA::TQueueId aQueueId,
            								  TUint8 aTxRateClassId,
            								  WHA::TRate aMaxTransmitRate,
            								  TBool aMore,
            								  WHA::TPacketId aPacketId,
            								  WHA::TPowerLevel aPowerLevel,
            								  TUint32 aExpiryTime);


    WHA::TStatus AlignTxForTWD (void*& aFrame,TUint16& aLength);
    void AlignTxSecurityForUMAC (TBool aQFull = FALSE);
	
	WHA::TStatus ConstructAddKey (	TSecurityKeys   *aSecurityKey,  /* structure to be filled               */
							    WHA::TKeyType        aType,          /* Type of the key to be added          */
							    const void      *aKey,          /* Pointer to buffer specifying the key */
							                                    /* material, according to the key type  */
							                                    /* (see specification for details).     */
							    TUint8          aEntryIndex);    /* Key entry index. Valid range: 0-8.   */

	/*Data*/
	TIWlanHpaCB*                        iTiWlanHpaCb;
	WHA::UCommandResponseParams         iUCommandResponseParams; 
	TI_UINT8							iReabMibMem[TIWha_MAX_READ_MIB_BUFFER];
	WHA::UCommandCompletionParams		iUCommandCompletionParams;
	WHA::UIndicationParams				iUIndicationParams;
	TI_BOOL								bErrorIndication;
	TI_BOOL								bCallRelease;	
	TI_BOOL								bFreeDriver;    	
	TFileInfo           				iFwFile;
    TI_UINT8*							ipNVSbuf;
	TI_UINT32							iNVSlength;
	TI_BOOL								bFailureIndication;	
	TwdCtrl                    			iTwdCtrl; 
    TAutoRadioIniParams                 iAutoRadioParams;   

    TSmartReflexConfigParams            iSmartReflexParams;
    TSmartReflexDebugParams             iSmartReflexDebugParams;

    TI_UINT8                            iSecurityPadding;
     
    /* Add the Soft Gemini Parameters to TIWha */
    TSoftGeminiParams					iSgParams;
    TI_UINT32							iRxPacketsAllocated;

#ifdef TI_TEST
	WHA::TQueueId						iQueueId;
	ACXRoamingStatisticsTable_t			roamingStatistics;
#endif /* TI_TEST */

    /* PDD create API */
public:
    TInt Install();
    void GetCaps( TDes8& aDes ) const;
    TInt Create( DBase *&aChannel, TInt aUnit, const TDesC* anInfo, const TVersion& aVer);
    TInt Validate( TInt aUnit, const TDesC8 *aInfo, const TVersion &aVer );
    TBool Attach( MWlanOsa& aWlanOsa, MWlanOsaExt& aWlanOsaExt );
    WHA::Wha& Extract();
    void GetCapabilities( SCapabilities*& aCapabilities );

    /*
	 * Hold the capability off the cashe memory
	 */
    SCapabilities*    aCapability;

public:

	TUint8								iNvsStart[24];
	TUint8								iBipNvsBuffer[MAX_TLV_LENGTH];

    TI_UINT32                           iPacketIdTable[CTRL_BLK_ENTRIES_NUM];
	TUint16								aScanResultCount;
	TUint16								SentScanResult;
	TI_STATUS							aPsMode;
	TI_STATUS							aScanStatus; 
    TI_HANDLE							hConnectionTimer;           /* Timer for special rate management during connection */
    static const TI_UINT32              uConnectionApproxTimeMs = 5000; /* After 5 seconds we will set back the original Tx fail rate */
    TI_BOOL                             bConnectionTimerRunning;
	
    TI_HANDLE                           hInitializeTimer;     /* Holds the timer for initialize */
    TI_HANDLE                           readDot11StationIdMibTmr;

    TI_HANDLE                           hRxMemFailTimer;		/* Next parameters handle Rx memory allocation failure */
	TI_BOOL								bRxMemFailTimerRunning;
    TI_UINT32                           uRxMemFailCount;                    
    static const TI_UINT32              MAX_CONSECUTIVE_RX_MEM_FAIL = 50; /* After this amount of failures we will start dropping packets */
    static const TI_UINT32              RX_ALLOC_FAIL_TIME_OUT = 10;      /* time out for requesting a buffer again */

	EFailureEvent 						ifailureEvent;

    TMib								iTxRatePolicy;

#ifdef HT_SUPPORT
	TMacAddr							iJoinedMacAddress; 
	TUint8 								iTxBlockAckUsageLast;
    TUint8 								iRxBlockAckUsageLast;
	WHA::THtMcsSet						iMcsRates;
	TUint8								iBACounertRespone;	
#endif /* HT_SUPPORT */
    TI_UINT8                            bJoined;
        
    TI_UINT8                            iConnectionCounter;
    const void*                         iData;
    TI_UINT32                           iLength;

    /* Holds the failure event number in order save it in the failure handling context switch */
    EFailureEvent                       iFailureEvent;

    /* Holds the other two optinal wlan DFC's context */
    TIFailureDfcClient*                 pFailureDfcClient;
    TIConnectDfcClient*                 pConnectDfcClient;
    TBusDrvCfg                          BusDrvCfg;

private:
    TI_BOOL                       bDriverCreated;
};

#endif      // TIWha_H   



