#include "can0.h"	
void read_rx_msg(int msg_obj,int *id,bool *remote,char *len,char *message){
        tCANMsgObject msg;
	msg.pui8MsgData = message;
 	CANMessageGet(CAN0_BASE, msg_obj, &msg, true); 
	*id=msg.ui32MsgID;
	*len=msg.ui32MsgLen;
	if(msg.ui32Flags & MSG_OBJ_REMOTE_FRAME) *remote=true;
	else *remote=false;
}

void can_set_receive_object(int msg_obj,int id,int id_mask){
	tCANMsgObject msg;	
	msg.ui32MsgID = id;
	msg.ui32MsgIDMask = id_mask;
	msg.ui32Flags = MSG_OBJ_USE_ID_FILTER;
	CANMessageSet(CAN0_BASE, msg_obj, &msg, MSG_OBJ_TYPE_RX);
}

void can_send_message(int msg_obj,int id,bool remote,int len,char *message){
	tCANMsgObject msg;	
	msg.ui32MsgID = id;
	msg.ui32MsgLen = len;	
	if(remote) CANMessageSet(CAN0_BASE, msg_obj, &msg, MSG_OBJ_TYPE_TX_REMOTE); 
	else { msg.pui8MsgData = message; CANMessageSet(CAN0_BASE, msg_obj, &msg, MSG_OBJ_TYPE_TX);}
}

void CAN0_init(uint32_t bitrate,int rx_obj){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // enable CAN0 GPIO peripheral
	GPIOPinConfigure(GPIO_PE4_CAN0RX);
	GPIOPinConfigure(GPIO_PE5_CAN0TX);
	GPIOPinTypeCAN(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
	CANInit(CAN0_BASE);
	CANBitRateSet(CAN0_BASE, SysCtlClockGet(), bitrate);
	CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
	IntEnable(INT_CAN0);
	CANEnable(CAN0_BASE);
	can_set_receive_object(rx_obj,0,0);
}