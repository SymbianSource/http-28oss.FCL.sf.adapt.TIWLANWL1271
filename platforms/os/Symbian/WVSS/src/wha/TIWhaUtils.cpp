/*
 * TIWhaUtils.cpp
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


/** \file  TIWhaUtils.cpp 
 *  \brief  Utilities used by TIWha layer only
 *
 *  \see   
*/

#include "TIWhaUtils.h"
#define __FILE_ID__								FILE_ID_148

/** 
* \fn     WhaToTwdFrameType
*  
* \return   
* \sa     
*/ 
EMibTemplateType TIWhaUtils::WhaToTwdFrameType(WHA::TTemplateType aFrameType) 
{
    switch (aFrameType)
    {
        case WHA::KBeaconTemplate: return TEMPLATE_TYPE_BEACON;
        case WHA::KProbeRequestTemplate: return TEMPLATE_TYPE_PROBE_REQUEST;
        case WHA::KNullDataTemplate: return TEMPLATE_TYPE_NULL_FRAME;
        case WHA::KProbeResponseTemplate: return TEMPLATE_TYPE_PROBE_RESPONSE;
        case WHA::KQosNullDataTemplate: return TEMPLATE_TYPE_QOS_NULL_FRAME;
        case WHA::KPSPollTemplate: return TEMPLATE_TYPE_PS_POLL;
        default:
            WLAN_OS_REPORT (("ERROR: %s aFrameType = %d\n",__FUNCTION__,aFrameType)); 
            return  TEMPLATE_TYPE_BEACON;
    }
}

/** 
* \fn     WhaToTwdRate
*  
* \return   
* \sa     
*/ 
ERate TIWhaUtils::WhaToTwdRate (WHA::TRate aRate)
{
    /*Important Note: aRate = 0 - means don't care. Use 1M rate same as the default rate.
	UMAC sets probe req template with rate = 0, as it doesn't care for probe req rate*/
    switch (aRate)
    {
		case 0:
        case WHA::KRate1Mbits:   return  DRV_RATE_1M; 
        case WHA::KRate2Mbits:   return  DRV_RATE_2M;  
        case WHA::KRate5_5Mbits: return  DRV_RATE_5_5M;
        case WHA::KRate6Mbits:   return  DRV_RATE_6M; 
        case WHA::KRate9Mbits:   return  DRV_RATE_9M; 
        case WHA::KRate11Mbits:  return  DRV_RATE_11M;  
        case WHA::KRate12Mbits:  return  DRV_RATE_12M;  
        case WHA::KRate18Mbits:  return  DRV_RATE_18M;  
        case WHA::KRate22Mbits:  return  DRV_RATE_22M;  
        case WHA::KRate24Mbits:  return  DRV_RATE_24M;  
        case WHA::KRate36Mbits:  return  DRV_RATE_36M;  
        case WHA::KRate48Mbits:  return  DRV_RATE_48M;  
        case WHA::KRate54Mbits:  return  DRV_RATE_54M;  
        default:      
            WLAN_OS_REPORT (("ERROR: %s aRate = %d\n",__FUNCTION__,aRate)); 
            return  DRV_RATE_1M;
    }
}

/** 
* \fn     PolicyToWhaRate
*  
* \param ePolicyRate 
* \return  TRate 
* \sa     
*/ 
WHA::TRate TIWhaUtils::PolicyToWhaRate (ETxRateClassId ePolicyRate)
{
    switch (ePolicyRate)
    {
#ifdef HT_SUPPORT
        case     txPolicyMcs7:      return  WHA::KRate1Mbits;
        case     txPolicyMcs6:      return  WHA::KRate1Mbits;
        case     txPolicyMcs5:      return  WHA::KRate1Mbits;
        case     txPolicyMcs4:      return  WHA::KRate1Mbits;
        case     txPolicyMcs3:      return  WHA::KRate1Mbits;
        case     txPolicyMcs2:      return  WHA::KRate1Mbits;
        case     txPolicyMcs1:      return  WHA::KRate1Mbits;
        case     txPolicyMcs0:      return  WHA::KRate1Mbits;
#endif /* HT_SUPPORT */
        case     txPolicy54:        return  WHA::KRate54Mbits;  
        case     txPolicy48:        return  WHA::KRate48Mbits;  
        case     txPolicy36:        return  WHA::KRate36Mbits;  
        case     txPolicy24:        return  WHA::KRate24Mbits;  
        case     txPolicy22:        return  WHA::KRate22Mbits;  
        case     txPolicy18:        return  WHA::KRate18Mbits;  
        case     txPolicy12:        return  WHA::KRate12Mbits;  
        case     txPolicy11:        return  WHA::KRate11Mbits;  
        case     txPolicy9:     return  WHA::KRate9Mbits; 
        case     txPolicy6:     return  WHA::KRate6Mbits; 
        case     txPolicy5_5:       return  WHA::KRate5_5Mbits;
        case     txPolicy2:     return  WHA::KRate2Mbits;  
        case     txPolicy1:     return  WHA::KRate1Mbits;

        default:      
            WLAN_OS_REPORT (("ERROR: %s aRate = %d\n",__FUNCTION__,ePolicyRate)); 
            return  WHA::KRate1Mbits;
    }
}

/** 
 * \fn     WhaToMaskRate
 * \brief  Rate mask to convert the TIWha rates
 * 
 *  
 * 
 * \note    
 * \param  TIWhaRate 
  * \return  rateNask_e - corosponding aRate
 * \sa      
 */ 
TI_UINT32  TIWhaUtils::WhaToMaskRate (WHA::TRate aRate)
{

    TI_UINT32 rateMask =0;
    
        
    (aRate& WHA::KRate1Mbits )  ? rateMask |= DRV_RATE_MASK_1_BARKER :   NULL;
    (aRate& WHA::KRate2Mbits )  ? rateMask |= DRV_RATE_MASK_2_BARKER :   NULL;
    (aRate& WHA::KRate5_5Mbits )? rateMask |= DRV_RATE_MASK_5_5_CCK  :   NULL;
    (aRate& WHA::KRate6Mbits )  ? rateMask |= DRV_RATE_MASK_6_OFDM   :   NULL;
    (aRate& WHA::KRate9Mbits )  ? rateMask |= DRV_RATE_MASK_9_OFDM   :   NULL;
    (aRate& WHA::KRate11Mbits ) ? rateMask |= DRV_RATE_MASK_11_CCK   :   NULL;
    (aRate& WHA::KRate12Mbits ) ? rateMask |= DRV_RATE_MASK_12_OFDM  :   NULL;
    (aRate& WHA::KRate18Mbits ) ? rateMask |= DRV_RATE_MASK_18_OFDM  :   NULL;
    (aRate& WHA::KRate22Mbits ) ? rateMask |= DRV_RATE_MASK_22_PBCC  :   NULL;
    (aRate& WHA::KRate24Mbits ) ? rateMask |= DRV_RATE_MASK_24_OFDM  :   NULL;
    (aRate& WHA::KRate33Mbits ) ? rateMask |= DRV_RATE_MASK_1_BARKER :   NULL;
    (aRate& WHA::KRate36Mbits ) ? rateMask |= DRV_RATE_MASK_36_OFDM  :   NULL;
    (aRate& WHA::KRate48Mbits ) ? rateMask |= DRV_RATE_MASK_48_OFDM  :   NULL;
    (aRate& WHA::KRate54Mbits ) ? rateMask |= DRV_RATE_MASK_54_OFDM  :   NULL;

    if (rateMask == 0)
	{
                 WLAN_OS_REPORT (("ERROR: %s aRate = %d\n",__FUNCTION__,aRate)); 
        rateMask|= DRV_RATE_MASK_1_BARKER; // as default
}

    return rateMask;
 }


/** 
 * \fn     HTWhaToMaskRate
 * \brief  HT Rate mask to convert the TIWha rates
 * 
 *  
 * 
 * \note    
 * \param  TIWhaRate 
 * \param  THtMcsSet
  * \return  rateNask_e - corosponding aRate
 * \sa      
 */ 
#ifdef HT_SUPPORT
TI_UINT32  TIWhaUtils::HTWhaToMaskRate (WHA::TRate aRate,WHA::THtMcsSet	aMcsRates)
{
	TI_UINT32 rateMask =0;
    
        
    (aRate& WHA::KRate1Mbits )  ? rateMask |= DRV_RATE_MASK_1_BARKER :   NULL;
    (aRate& WHA::KRate2Mbits )  ? rateMask |= DRV_RATE_MASK_2_BARKER :   NULL;
    (aRate& WHA::KRate5_5Mbits )? rateMask |= DRV_RATE_MASK_5_5_CCK  :   NULL;
    (aRate& WHA::KRate6Mbits )  ? rateMask |= DRV_RATE_MASK_6_OFDM   :   NULL;
    (aRate& WHA::KRate9Mbits )  ? rateMask |= DRV_RATE_MASK_9_OFDM   :   NULL;
    (aRate& WHA::KRate11Mbits ) ? rateMask |= DRV_RATE_MASK_11_CCK   :   NULL;
    (aRate& WHA::KRate12Mbits ) ? rateMask |= DRV_RATE_MASK_12_OFDM  :   NULL;
    (aRate& WHA::KRate18Mbits ) ? rateMask |= DRV_RATE_MASK_18_OFDM  :   NULL;
    (aRate& WHA::KRate22Mbits ) ? rateMask |= DRV_RATE_MASK_22_PBCC  :   NULL;
    (aRate& WHA::KRate24Mbits ) ? rateMask |= DRV_RATE_MASK_24_OFDM  :   NULL;
    (aRate& WHA::KRate33Mbits ) ? rateMask |= DRV_RATE_MASK_1_BARKER :   NULL;
    (aRate& WHA::KRate36Mbits ) ? rateMask |= DRV_RATE_MASK_36_OFDM  :   NULL;
    (aRate& WHA::KRate48Mbits ) ? rateMask |= DRV_RATE_MASK_48_OFDM  :   NULL;
    (aRate& WHA::KRate54Mbits ) ? rateMask |= DRV_RATE_MASK_54_OFDM  :   NULL;

	/* MCS Rates */
	(aMcsRates[0] & WHA_MCS_0)	? rateMask |= DRV_RATE_MASK_MCS_0_OFDM : NULL;
	(aMcsRates[0] & WHA_MCS_1)	? rateMask |= DRV_RATE_MASK_MCS_1_OFDM : NULL;
	(aMcsRates[0] & WHA_MCS_2)	? rateMask |= DRV_RATE_MASK_MCS_2_OFDM : NULL;
	(aMcsRates[0] & WHA_MCS_3)	? rateMask |= DRV_RATE_MASK_MCS_3_OFDM : NULL;
	(aMcsRates[0] & WHA_MCS_4)	? rateMask |= DRV_RATE_MASK_MCS_4_OFDM : NULL;
	(aMcsRates[0] & WHA_MCS_5)	? rateMask |= DRV_RATE_MASK_MCS_5_OFDM : NULL;
	(aMcsRates[0] & WHA_MCS_6)	? rateMask |= DRV_RATE_MASK_MCS_6_OFDM : NULL;
	(aMcsRates[0] & WHA_MCS_7)	? rateMask |= DRV_RATE_MASK_MCS_7_OFDM : NULL;

	if (rateMask == 0)
	{
        WLAN_OS_REPORT (("ERROR: %s aRate = %d\n",__FUNCTION__,aRate)); 
        rateMask|= DRV_RATE_MASK_1_BARKER; // as default
	}
	return rateMask;
}
#endif /* HT_SUPPORT */


TI_UINT32 TIWhaUtils::WhaRateToRatePolicy(WHA::StxAutoRatePolicy* pTxAutoRatePolicy)
{
	TI_UINT32	aRateMask = 0;
	/* B/G Rates */
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate1Mbits) 	? aRateMask |= HW_BIT_RATE_1MBPS	: NULL;
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate2Mbits) 	? aRateMask |= HW_BIT_RATE_2MBPS	: NULL;
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate5_5Mbits)	? aRateMask |= HW_BIT_RATE_5_5MBPS	: NULL;
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate6Mbits) 	? aRateMask |= HW_BIT_RATE_6MBPS	: NULL;
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate9Mbits) 	? aRateMask |= HW_BIT_RATE_9MBPS	: NULL;
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate11Mbits) 	? aRateMask |= HW_BIT_RATE_11MBPS	: NULL;
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate12Mbits) 	? aRateMask |= HW_BIT_RATE_12MBPS	: NULL;
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate18Mbits) 	? aRateMask |= HW_BIT_RATE_18MBPS	: NULL;
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate22Mbits) 	? aRateMask |= HW_BIT_RATE_22MBPS	: NULL;
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate24Mbits) 	? aRateMask |= HW_BIT_RATE_24MBPS	: NULL;
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate36Mbits) 	? aRateMask |= HW_BIT_RATE_36MBPS	: NULL;
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate48Mbits) 	? aRateMask |= HW_BIT_RATE_48MBPS	: NULL;
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate54Mbits) 	? aRateMask |= HW_BIT_RATE_54MBPS	: NULL;
    
	return aRateMask;
}

#ifdef HT_SUPPORT
TI_UINT32 TIWhaUtils::HTWhaRateToRatePolicy(WHA::StxAutoRatePolicy* pTxAutoRatePolicy)
{
	TI_UINT32	aRateMask = 0;
	/* B/G Rates */
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate1Mbits) 	? aRateMask |= HW_BIT_RATE_1MBPS	: NULL;
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate2Mbits) 	? aRateMask |= HW_BIT_RATE_2MBPS	: NULL;
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate5_5Mbits)	? aRateMask |= HW_BIT_RATE_5_5MBPS	: NULL;
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate6Mbits) 	? aRateMask |= HW_BIT_RATE_6MBPS	: NULL;
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate9Mbits) 	? aRateMask |= HW_BIT_RATE_9MBPS	: NULL;
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate11Mbits) 	? aRateMask |= HW_BIT_RATE_11MBPS	: NULL;
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate12Mbits) 	? aRateMask |= HW_BIT_RATE_12MBPS	: NULL;
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate18Mbits) 	? aRateMask |= HW_BIT_RATE_18MBPS	: NULL;
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate22Mbits) 	? aRateMask |= HW_BIT_RATE_22MBPS	: NULL;
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate24Mbits) 	? aRateMask |= HW_BIT_RATE_24MBPS	: NULL;
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate36Mbits) 	? aRateMask |= HW_BIT_RATE_36MBPS	: NULL;
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate48Mbits) 	? aRateMask |= HW_BIT_RATE_48MBPS	: NULL;
	(pTxAutoRatePolicy->iBAndGRates & WHA::KRate54Mbits) 	? aRateMask |= HW_BIT_RATE_54MBPS	: NULL;

	/* 11n Rates */
	(pTxAutoRatePolicy->iMcsSet[0] & WHA_MCS_0) 			? aRateMask |= HW_BIT_RATE_MCS_0	: NULL;
	(pTxAutoRatePolicy->iMcsSet[0] & WHA_MCS_1) 			? aRateMask |= HW_BIT_RATE_MCS_1	: NULL;
	(pTxAutoRatePolicy->iMcsSet[0] & WHA_MCS_2) 			? aRateMask |= HW_BIT_RATE_MCS_2	: NULL;
	(pTxAutoRatePolicy->iMcsSet[0] & WHA_MCS_3) 	   	 	? aRateMask |= HW_BIT_RATE_MCS_3	: NULL;
	(pTxAutoRatePolicy->iMcsSet[0] & WHA_MCS_4) 	   	 	? aRateMask |= HW_BIT_RATE_MCS_4	: NULL;
	(pTxAutoRatePolicy->iMcsSet[0] & WHA_MCS_5) 			? aRateMask |= HW_BIT_RATE_MCS_5	: NULL;
	(pTxAutoRatePolicy->iMcsSet[0] & WHA_MCS_6) 			? aRateMask |= HW_BIT_RATE_MCS_6	: NULL;
	(pTxAutoRatePolicy->iMcsSet[0] & WHA_MCS_7) 			? aRateMask |= HW_BIT_RATE_MCS_7	: NULL;

	return aRateMask;
}
#endif /* HT_SUPPORT */
