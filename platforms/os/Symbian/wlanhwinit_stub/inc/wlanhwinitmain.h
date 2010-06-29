/*
 * wlanhwinitmain.h
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

  FILENAME:       wlanhwinitmain.h
*/

#ifndef WLANHWINITMAIN_H
#define WLANHWINITMAIN_H

#include <e32base.h>
#include <wlanhwinit.h>
#include <e32std.h>

//#include "wlanhwinittypes.h"
#include "wlanhwinitinterface.h"


/**
* This class implements the actual HW specific initialization functionality.
*/
NONSHARABLE_CLASS( CWlanHwInitMain ): public CBase, public MWlanHwInitInterface
    {
    public:  
        /* constructor */
        static CWlanHwInitMain* NewL();

        /* Destructor */
        virtual ~CWlanHwInitMain();

       /** 
       * \fn     GetHwInitData
       * \brief  From MWlanHwInitInterface Get pointer to hardware specific initialization data
       * (reference wlanhwinit.h) 
       *          
       * \note    
       * \param aInitData Pointer to initialization data, NULL if none.  
       * \param aInitLength Length of initialization data. 
       * \param aFwData Pointer to firmware data, NULL if none.
       * \param FwLength Length of firmware data.
       * \return 
       * \sa      
       */ 
        void GetHwInitData(const TUint8** aInitData, TUint& aInitLength, const TUint8** aFwData, TUint& aFwLength);

        /** 
        * \fn     GetMacAddress
        * \brief  
        * (reference wlanhwinit.h) 
        *          
        * \note    
        * \param aMacAddress MAC address of the device.  
        * \return A Symbian error code.
        * \sa      
        */ 
        TInt GetMacAddress(TMacAddr& aMacAddress);

       /** 
        * \fn     GetHwTestInitData
        * \brief  Get pointer to hardware specific initialization data for production testing.
        * (reference wlanhwinit.h) 
        *          
        * \note    
        * \param aInitData Pointer to initialization data, NULL if none.
        * \param aInitLength Length of initialization data.
        * \param aFwData Pointer to firmware data, NULL if none.
        * \param aFwLength Length of firmware data.
        * \return
        * \sa      
        */ 
        void GetHwTestInitData(const TUint8** aInitData, TUint& aInitLength, const TUint8** aFwData, TUint& aFwLength);

        /** 
         * \fn     GetHwTestInitData
         * \brief  Get hardware specific production testing data.
         * (reference wlanhwinit.h) 
         *          
         * \note    
         * \param aId Id of the parameter to read.
         * \param aData Buffer for read data.
         * \return A Symbian error code.
         * \sa      
         */ 
        TInt GetHwTestData(TUint aId, TDes8& aData);

        /** 
         * \fn     SetHwTestData
         * \brief  Set hardware specific production testing data.
         * (reference wlanhwinit.h) 
         *          
         * \note    
         * \param aId Id of the parameter to store.
         * \param aData Data to be stored.
         * \return A Symbian error code.
         * \sa      
         */ 
        TInt SetHwTestData(TUint aId, TDesC8& aData);

    private:

        CWlanHwInitMain();

        void ConstructL();

        void GetMacAddressL(TMacAddr& aMacAddress);

        void GetTuningDataL(TDes8& aTuningData);

        void SetTuningDataL(TDesC8& aTuningData);
                     
        void FillRadioData(TUint8 *bob10mradio);

    private:    

        TUint8* ipaData;
        
        /**
        * Checks if a firmware file can be found from
        * the memory card.
        * return False if not found or error occured, True if 
        * the firmware was successfully read.
        * The firmware is loaded only once.
        */
        TBool IsMMCFirmwareFound();
 
 
      /** 
     * \fn     IsMMCNvsFound() 
     * \brief  Checks if a NVS file can be found from the memory card.
     * 
     * To success read the NVS file from a card the NVS bin file should be
     * in nvs directory in the root of the card
     * 
     * \param  None
     * \return false if not found or error occured, True if 
     * the nvs was successfully read.
     * The nvs is loaded only once.
     * \sa     IsMMCNvsFound
     */ 
        TBool IsMMCNvsFound();
 
 
        /** Buffer for possible firmware loaded from MC. */
        HBufC8* iFirmwareMC;
        
        /** Buffer for possible NVS loaded from MC. */
        HBufC8* pNvsMC;
        
    };

#endif // WLANHWINITMAIN_H
