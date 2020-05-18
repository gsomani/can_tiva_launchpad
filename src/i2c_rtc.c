#include<stdint.h>
#include"inc/tm4c123gh6pm.h"
#include"i2c_rtc.h" 
#define LCD_DISPLAY 1

#define     LCD_RS 0x40     /* PORTA BIT6 mask */
#define     LCD_EN 0x80     /* PORTA BIT7 mask */

#define LCD_DATA_PORT    (*((volatile uint32_t *)0x400053FC))    /* PORTB7 - PORTB0 */
#define LCD_CMD_PORT     (*((volatile uint32_t *)0x40004300))    /* PA7-PA6 */

char cur_date[8],cur_time[8];

void delayMs(int n)
{
int i, j;
for(i = 0 ; i < n; i++)
for(j = 0; j < 3180; j++) {}
}

void delayUs(int n)
{
    int i, j;

    for( i = 0 ; i < n; i++ )
        for( j = 0; j < 3; j++ ) {} /* do nothing for 1 us */
}

void LCD_OutCmd(unsigned char command)
{
    LCD_DATA_PORT = command;
    LCD_CMD_PORT = 0;           /* E=0, R/W=0, RS=0 */
    delayUs(6);                 /* wait 6us */
    LCD_CMD_PORT = LCD_EN;      /* E=1, R/W=0, RS=0 */
    delayUs(6);                 /* wait 6us */
    LCD_CMD_PORT = 0;           /* E=0, R/W=0, RS=0 */
    delayUs(40);                /* wait 40us */
}

/* Clear the LCD */
void LCD_Clear(void)
{
    LCD_OutCmd(0x01);           /* Clear Display */
    delayUs(1600);              /* wait 1.6ms */
    LCD_OutCmd(0x02);           /* Cursor to home */
    delayUs(1600);              /* wait 1.6ms */
}

void LCD_OutChar(char data)
{
    LCD_DATA_PORT = data;
    LCD_CMD_PORT = LCD_RS;      /* E=0, R/W=0, RS=1 */
    delayUs(6);                 /* wait 6us */
    LCD_CMD_PORT = LCD_EN | LCD_RS; /* E=1, R/W=0, RS=1 */
    delayUs(6);                 /* wait 6us */
    LCD_CMD_PORT = LCD_RS;      /* E=0, R/W=0, RS=1 */
    delayUs(40);                /* wait 40us */
}

void init_lcd()
{
    	SYSCTL_RCGCGPIO_R |= 0x00000003;
    	GPIO_PORTB_DIR_R |= 0xFF;
    	GPIO_PORTB_DEN_R |= 0xFF;
    	GPIO_PORTA_DIR_R |= 0xC0;
    	GPIO_PORTA_DEN_R |= 0xC0;
    	delayMs(60); 
	LCD_OutCmd(0x38);
	delayMs(5);		
	LCD_OutCmd(0x38);
	delayUs(100);
	LCD_OutCmd(0x38);       
        LCD_OutCmd(0x8);
	LCD_Clear();        
	LCD_OutCmd(0xE);
        LCD_OutCmd(0xF);
	LCD_OutCmd(0xC);	
}

static int I2C_wait_till_done(void)
{
    while(I2C3_MCS_R & 1)
        ;                           /* wait until I2C master is not busy */
    return I2C3_MCS_R & 0xE;        /* return I2C error code */
}
 
/* Write one byte only */
/* byte write: S-(saddr+w)-ACK-maddr-ACK-data-ACK-P */
char I2C3_byteWrite(int slaveAddr, char memAddr, char data)
{
    char error;
 
    /* send slave address and starting address */
    I2C3_MSA_R = slaveAddr << 1;
    I2C3_MDR_R = memAddr;
    I2C3_MCS_R = 3;                 /* S-(saddr+w)-ACK-maddr-ACK */
    error = I2C_wait_till_done();   /* wait until write is complete */
    if(error) return error;

    /* send data */
    I2C3_MDR_R = data;
    I2C3_MCS_R = 5;                 /* -data-ACK-P */
    error = I2C_wait_till_done();   /* wait until write is complete */
    while( I2C3_MCS_R & 0x40 )
        ;                           /* wait until bus is not busy */
    error = I2C3_MCS_R & 0xE;
    if(error) return error;
 
    return 0;                       /* no error */
}
 
/* Read memory */
/* read: S-(saddr+w)-ACK-maddr-ACK-R-(saddr+r)-ACK-data-ACK-data-ACK-...-data-NACK-P */
char I2C3_read(int slaveAddr, char memAddr, int byteCount, char* data)
{
    char error;
 
    if (byteCount <= 0)
        return -1;                  /* no read was performed */
 
    /* send slave address and starting address */
    I2C3_MSA_R = slaveAddr << 1;
    I2C3_MDR_R = memAddr;
    I2C3_MCS_R = 3;                 /* S-(saddr+w)-ACK-maddr-ACK */
    error = I2C_wait_till_done();
    if(error)
        return error;

    /* to change bus from write to read, send restart with slave addr */
    I2C3_MSA_R = (slaveAddr << 1) + 1;   /* restart: -R-(saddr+r)-ACK */
 
    if( byteCount == 1 )            /* if last byte, don't ack */
        I2C3_MCS_R = 7;             /* -data-NACK-P */
    else                            /* else ack */
        I2C3_MCS_R = 0xB;           /* -data-ACK- */
    error = I2C_wait_till_done();
    if(error) return error;
 
    *data++ = I2C3_MDR_R;           /* store the data received */
 
    if( --byteCount == 0 ) {        /* if single byte read, done */
        while( I2C3_MCS_R & 0x40 )
            ;                       /* wait until bus is not busy */
        return 0;                   /* no error */
    }
 
    /* read the rest of the bytes */
    while( byteCount > 1 ) {
        I2C3_MCS_R = 9;              /* -data-ACK- */
        error = I2C_wait_till_done();
        if(error) return error;
        byteCount--;
        *data++ = I2C3_MDR_R;        /* store data received */
    }
 
    I2C3_MCS_R = 5;                 /* -data-NACK-P */
    error = I2C_wait_till_done();
    *data = I2C3_MDR_R;             /* store data received */
    while( I2C3_MCS_R & 0x40 )
        ;                           /* wait until bus is not busy */
 
    return 0;                       /* no error */
}
 
void read_cur_time()
{
	char err;
	err = I2C3_read(SLAVE_ADDR, 0, 3, time);
	err = I2C3_read(SLAVE_ADDR, 4, 3, date);
	cur_date[0]=(date[0]>>4)+'0';cur_date[1]=(date[0] & 0xF)+'0';
	cur_date[3]=(date[1]>>4)+'0';cur_date[4]=(date[1] & 0xF)+'0';
	cur_date[6]=(date[2]>>4)+'0';cur_date[7]=(date[2] & 0xF)+'0';
	cur_time[0]=(time[2]>>4)+'0';cur_time[1]=(time[2] & 0xF)+'0';
	cur_time[3]=(time[1]>>4)+'0';cur_time[4]=(time[1] & 0xF)+'0';
	cur_time[6]=(time[0]>>4)+'0';cur_time[7]=(time[0] & 0xF)+'0';
}

void reset_date_time()
{
	char err;int i;	
	for(i=6;i>=4;i--) err = I2C3_byteWrite(SLAVE_ADDR, i, 0);	
	for(i=2;i>=0;i--) err = I2C3_byteWrite(SLAVE_ADDR, i, 0);
	cur_date[2]=cur_date[5]='/';
	cur_time[2]=cur_time[5]=':';cur_time[8]=cur_date[8]=0;
}

void lcd_display_date_time()
{
	int i;
	LCD_Clear();	
	for (i=0;i<8;i++) LCD_OutChar(cur_date[i]);
	LCD_OutCmd(0xC0);
	for (i=0;i<8;i++) LCD_OutChar(cur_time[i]);
}

void I2C3_init(void)
{
    SYSCTL_RCGCI2C_R |= 0x08;      
    SYSCTL_RCGCGPIO_R |= 0x08;      
 
    /* PORTA 7, 6 for I2C3 */
    GPIO_PORTD_AFSEL_R |= 0x03;     
    GPIO_PORTD_PCTL_R &= ~0xFF; 
    GPIO_PORTD_PCTL_R |= 0x33;
    GPIO_PORTD_DEN_R |= 0x03;       
    GPIO_PORTD_ODR_R |= 0x02;       
 
    I2C3_MCR_R = 0x10;              
    I2C3_MTPR_R = 7;                
}

void initialise_lcd_time(){
init_lcd();
I2C3_init();
reset_date_time();
}
