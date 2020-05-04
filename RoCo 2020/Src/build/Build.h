/*
 * Build.h
 *
 *  Created on: 3 May 2020
 *      Author: Arion
 */

#ifndef BUILD_BUILD_H_
#define BUILD_BUILD_H_


#define PROTOCOL_20W18
#define BUILD_FOR_CONTROL_STATION

#ifdef BUILD_FOR_CONTROL_STATION

#elif BUILD_FOR_COMPUTER_VISION
#elif BUILD_FOR_STANDARD_SYSTEM
#elif BUILD_FOR_DISPATCHER_CHIP
#else
#error "Please specify a build target"
#endif

#define BUILD_WITH_NETWORK_IO
#define BUILD_WITH_NETWORK_CLIENT_IO
#define BUILD_WITH_LOOPBACK_BUS


#endif /* BUILD_BUILD_H_ */
