/**
 * Copyright (c) 2016 - 2018 Nordic Semiconductor ASA and Luxoft Global Operations Gmbh.
 * 
 * All Rights Reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
#ifndef PHY_PLME_CCA_H_INCLUDED
#define PHY_PLME_CCA_H_INCLUDED

#include <stdint.h>
#include "phy_common.h"

/** @file
 * This file contains declarations of Clear Channel Assessment PHY routines and necessary types.
 *
 * @defgroup phy_cca PHY CCA API
 * @{
 * @brief Module to declare PHY CCA API
 */

/**@brief   PLME-CCA.confirm parameters
 *
 * @details The PLME-CCA.confirm primitive is generated by
 *          the initiating PLME and issued to its next higher layer
 *          in response to an PLME-CCA.request primitive.
 *          In accordance with IEEE Std 802.15.4-2006, section 6.2.2.2.1
 */
typedef struct
{
    /** One of PHY_TRX_OFF, PHY_BUSY or PHY_IDLE. */
    phy_enum_t status;
} plme_cca_conf_t;


/**@brief   PLME-CCA.request
 *
 * @details Request for clear channel assessment.
 *          In accordance with IEEE Std 802.15.4-2006, section 6.2.2.1
 */
void plme_cca_req(void);


/**@brief   PLME-CCA.confirm callback function, implemented by the next higher layer.
 *
 * @details The PLME-CCA.confirm primitive is generated by the PLME and issued
 *          to its next higher layer in response to an PLME-CCA.request primitive.
 *          In accordance with IEEE Std 802.15.4-2006, section 6.2.2.2
 *
 * @param[out] conf    Pointer to PLME-CCA.confirm parameters
 */
void plme_cca_conf(plme_cca_conf_t * conf);


/**@brief   Direct (synchronous) PLME-CCA.request
 *
 * @details Optional. Not covered by a standard.
 *
 * @return  One of PHY_TRX_OFF, PHY_BUSY or PHY_IDLE,
 *          or implementation defined error code in case of
 *          unavailability of access to system resources.
 */
phy_enum_t plme_cca(void);

/** @} */

#endif // PHY_PLME_CCA_H_INCLUDED
