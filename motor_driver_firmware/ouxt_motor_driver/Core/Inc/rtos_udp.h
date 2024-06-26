/*
 * rtos_udp.h
 *
 *  Created on: Jun 9, 2024
 *      Author: masaya
 */

//@sa https://qiita.com/hirekatsu0523/items/df651e7da8dee32e7891

#ifndef INC_RTOS_UDP_H_
#define INC_RTOS_UDP_H_

#include "lwip.h"
#include "lwip/sockets.h"

struct receive_data {
    double motor_speed;
};

struct send_data {
	double motor_speed;
};

struct receive_data GetROSData();
void SendF7Data(struct send_data*);
void UDPDefineTasks();

#endif /* INC_RTOS_UDP_H_ */
