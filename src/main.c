#include <string.h>
#include "can0.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#define baud 115200
#define max_bitrate 1000000
#ifdef I2C_RTC							        
#include "i2c_rtc.h"
#endif

char stream[200];	
bool rx_remote,tx_remote,generate=false;
char t=0,rx_len,rx_message[8],tx_len,tx_message[8];int rx_id,tx_id,rtr_id=1,rx_obj=1,tx_obj=2;uint32_t bitrate=500000,tx_count=0,rx_count=0;
	
void DisableInterrupts(void)
{
    __asm ("CPSID  I\n");
}

void EnableInterrupts(void)
{
    __asm  ("CPSIE  I\n");
}

void GPIOPortF_Init(void)
{
    SYSCTL_RCGC2_R |= 0x00000020;   /* 1) activate clock for PortF */
    GPIO_PORTF_LOCK_R = 0x4C4F434B; /* 2) unlock GPIO PortF */
    GPIO_PORTF_CR_R = 0x1F;         /* allow changes to PF4-0 */
    GPIO_PORTF_AMSEL_R = 0x00;      /* 3) disable analog on PF */
    GPIO_PORTF_PCTL_R = 0x00000000; /* 4) PCTL GPIO on PF4-0 */
    GPIO_PORTF_DIR_R = 0x0E;        /* 5) PF4,PF0 in, PF3-1 out */
    GPIO_PORTF_AFSEL_R = 0x00;      /* 6) disable alt funct on PF7-0 */
    GPIO_PORTF_PUR_R = 0x11;        /* enable pull-up on PF0 and PF4 */
    GPIO_PORTF_DEN_R = 0x1F;        /* 7) enable digital I/O on PF4-0 */



    GPIO_PORTF_IS_R &= ~0x10;       /*  PF4 is edge-sensitive */
    GPIO_PORTF_IBE_R &= ~0x10;      /*  PF4 is not both edges */
    GPIO_PORTF_IEV_R &= ~0x10;      /*  PF4 falling edge event */
    GPIO_PORTF_ICR_R = 0x10;        /*  Clear flag4 */
    GPIO_PORTF_IM_R |= 0x10;        /*  arm interrupt on PF4 */
    NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF1FFFFF) | 0x00A00000; /*  priority 5 */
    NVIC_EN0_R = 0x40000000;        /*  Enable interrupt 30 in NVIC */
}
					        
void send_date_time_request(){
	tx_id=rtr_id;tx_remote=true;tx_len=6;
	can_send_message(tx_obj,tx_id,tx_remote,tx_len,tx_message);	
}

void GPIOPortF_Handler(void)
{
    volatile int readback;

    GPIO_PORTF_ICR_R = 0x10;        /* clear PF4 int */
    send_date_time_request();
    readback = GPIO_PORTF_ICR_R;    /* a read to force clearing of interrupt flag */
}

void init_systick_time_update(){
   NVIC_ST_RELOAD_R = (SysCtlClockGet()>>2)-1;  /* reload with number of clocks per second */
   NVIC_ST_CTRL_R = 7;             /* enable SysTick interrupt, use system clock */ 
}

void print_msg(int id,bool remote,char len,char *message){
        UARTprintf("%x\t[%x]",id,len);
	if(remote) UARTprintf(" remote request");
	else 	
		for(int j=0; j<len;j++)
				UARTprintf(" %x",message[j]);
	UARTprintf("\r\n");
}

#ifdef I2C_RTC						        
void send_date_time(){
	int i;		
	for(i=0;i<3;i++){
		tx_message[i]=date[i];
		tx_message[i+3]=time[2-i];
	}
	tx_id=rtr_id;tx_remote=false;tx_len=6;
	can_send_message(tx_obj,tx_id,tx_remote,tx_len,tx_message);	
}
#endif



void SysTick_Handler(void)
{
		#ifdef I2C_RTC						        
		t=(t+1)&3;
		read_cur_time();
		lcd_display_date_time();
		if(generate)
			if(!t && !(time[0]&0xF)) send_date_time();
		#endif				
}

// CAN0 interrupt handler
void CAN0IntHandler(void) {

	unsigned long status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE); // read interrupt status
	if(status == CAN_INT_INTID_STATUS) {
		 // controller status interrupt		
		status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL); // read back error bits
	
	if(status & CAN_STS_RXOK ) { // message object 1
		CANIntClear(CAN0_BASE, rx_obj); // clear interrupt
		read_rx_msg(rx_obj,&rx_id,&rx_remote,&rx_len,rx_message);
		UARTprintf("received\t");
		print_msg(rx_id,rx_remote,rx_len,rx_message);
		#ifdef I2C_RTC			
		if(rx_len==6 && rx_remote && rx_id==rtr_id) send_date_time();
		#endif				
		rx_count++;		
	}
	if(status & CAN_STS_TXOK) { // message object 2
		CANIntClear(CAN0_BASE, tx_obj); // clear interrupt
		UARTprintf("sent\t");
		print_msg(tx_id,tx_remote,tx_len,tx_message);
		tx_count++;
		}
	} 
}

void UART_init(){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // enable UART0 GPIO peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTStdioConfig(0, baud, SysCtlClockGet()); 
}

char valid_hex_char(char c)
{
	if((c>='0' && c<='9')||(c>='A' && c<='F')||(c>='a' && c<='f')) return 1;
	return 0;
}

bool read_frame(char frame[],int *id,bool *remote,char *len,char *msg){
	int i,j,l;char cur_byte[3];cur_byte[2]=*id=*len=0;*remote=false;*msg=0;
	if(!valid_hex_char(frame[0]) || !valid_hex_char(frame[1]) || !valid_hex_char(frame[2])|| frame[3]!='#' || frame[0]>'7') return 0;
	frame[3]=0;		
	*id=strtol(frame,NULL,16);
	if(frame[4]=='R' || frame[4]=='r'){
		*remote=true;
		if(frame[5]) *len=frame[5]-'0';
		else *len=0;
		if(*len<0 || *len >8 || frame[6]) return 0;
	}
	else
	{
		l=strlen(frame+4);
		if(l>16 || (l&1)) return 0;
		*len=l/2;

		for(i=4,j=0;i<l+4;i+=2,j++){
			if(!valid_hex_char(frame[i]) || !valid_hex_char(frame[i+1])) return 0;
			cur_byte[0]=frame[i];cur_byte[1]=frame[i+1];			
			msg[j]=strtol(cur_byte,NULL,16);
		}
		msg[*len]=0;		 
	}
	return 1;
}

bool valid_dec(char rate[],uint32_t *req_rate){
	int i;uint32_t cur;
	for(i=0;rate[i];i++)
		if(rate[i]<'0' || rate[i]>'9') return false;
	cur = strtol(rate,NULL,10);
	*req_rate=cur;	
	return true;				
}

int main(void) {

	bool valid_frame,valid_number,cmd;uint32_t req_rate;
	char command[4][9]={"send","bitrate","generate","stop"},*tok;
	
	DisableInterrupts();
	#ifdef I2C_RTC
	initialise_lcd_time();
	#endif
	GPIOPortF_Init();			
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);	
	init_systick_time_update();
	CAN0_init(bitrate,rx_obj);
	UART_init();
	EnableInterrupts();

	UARTprintf("System Ready\r\n\n");			

	char escape=0,cur;int cur_idx=0,count=0;

while( 1 ) {	

while( UARTCharsAvail(UART0_BASE) ){
           cur = UARTCharGet(UART0_BASE);
	   if(escape>0) {
		escape++;
	   if(escape==3) {
			escape=0;
			if(cur=='C' && cur_idx<200) {
			UARTCharPut( UART0_BASE, cur);if(cur_idx>=count) stream[cur_idx]=' ';cur_idx++;} 
			else if (cur=='D') {UARTCharPut( UART0_BASE, cur);if(cur_idx>0) cur_idx--;} 
		continue;		
		}
		UARTCharPut( UART0_BASE, cur);
		continue;
	   }
	   if(cur_idx<200 || cur =='\r') UARTCharPut( UART0_BASE, cur);	   
	   if(cur=='\r') {UARTCharPut( UART0_BASE, '\n');break;}	
	   if(cur=='\e') 
			{escape=1;continue;} 	 
	   if(cur=='\b') {if(cur_idx>0) cur_idx--;continue;}		
           stream[cur_idx]=cur; cur_idx++;if(cur_idx>count) count=cur_idx;	
    }
if(cur!='\r') continue;	
stream[count]=0;

if(count>0) 
{
cmd=false;count=cur_idx=0;

tok=strtok(stream," \t");
        if(!strcmp(tok,command[0])) {
			cmd=true;
			tok=strtok(NULL," \t");
		        valid_frame=read_frame(tok,&tx_id,&tx_remote,&tx_len,tx_message);
			if(!valid_frame) {cmd=false;UARTprintf("Invalid CAN Frame\r\n");}
			tok=strtok(NULL," \t");	
			if(tok!=NULL) {cmd=false;UARTprintf("Invalid command format\r\n");}
			if(cmd) can_send_message(tx_obj,tx_id,tx_remote,tx_len,tx_message);
		} 
	else if(!strcmp(tok,command[1])){
			cmd=true;
			tok=strtok(NULL," \t");
			if(!tok) UARTprintf("bitrate = %i\n\r\n",bitrate);
			else 
				{	valid_number=valid_dec(tok,&req_rate);
					if(!valid_number) {cmd=false;UARTprintf("Invalid bitrate\r\n");}
					if(req_rate>max_bitrate) {cmd=false;UARTprintf("bitrate can't exceed 1000000.\r\n");}	
					tok=strtok(NULL," \t");	
					if(tok!=NULL) {cmd=false;UARTprintf("Invalid command format\r\n");}
				 	if(cmd) {bitrate=req_rate;CANBitRateSet(CAN0_BASE, SysCtlClockGet(), bitrate);}
				}
		}
	else if(!strcmp(tok,command[2])){
					cmd=true;
					tok=strtok(NULL," \t");	
					if(tok!=NULL) {cmd=false;UARTprintf("Invalid command format\r\n");}
				 	if(cmd)	generate=true;
			}
	else if(!strcmp(tok,command[3])){
					cmd=true;
					tok=strtok(NULL," \t");	
					if(tok!=NULL) {cmd=false;UARTprintf("Invalid command format\r\n");}
				 	if(cmd)	generate=false;
			}
	else UARTprintf("Invalid command\r\n");
	UARTprintf("\r\n");
}   
}
	return 0;
}
