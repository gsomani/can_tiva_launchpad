#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>	
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_can.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
	
void read_rx_msg(int msg_obj,int *id,bool *remote,char *len,char *message);
void can_set_receive_object(int msg_obj,int id,int id_mask);
void can_send_message(int msg_obj,int id,bool remote,int len,char *message);
void CAN0_init(uint32_t bitrate,int rx_obj);