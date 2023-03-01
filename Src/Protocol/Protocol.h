/*
 * Protocol.h
 *
 *  Created on: 12 Feb 2020
 *      Author: Arion
 */

#ifndef PROTOCOL_PROTOCOL_H_
#define PROTOCOL_PROTOCOL_H_

#include "../Build/Build.h"


#ifdef PROTOCOL_20W18
#include "Protocol20W18.h"
#endif /* PROTOCOL_20W18 */

#ifdef PROTOCOL_21W3
#include "Protocol21W3.h"
#endif /* PROTOCOL_20W18 */

#ifdef PROTOCOL_RADIO_22W12
#include "Protocol22W12_DWM.h"
#endif /* PROTOCOL_DWM_22W12 */

#ifdef PROTOCOL_22W29
#include "Protocol22W29.h"
#endif /* PROTOCOL_DWM_22W29 */

#ifdef PROTOCOL_23
#include "Protocol23.h"
#endif

#endif /* PROTOCOL_PROTOCOL_H_ */
