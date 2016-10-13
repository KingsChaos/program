/*
 * Elmo.c
 * Created: 2016/8/18 
 * Author: yangjunju
 * Use: Mecanum-Car
 */ 

#include <avr/io.h>
#include "clksys_driver.h"
#include "clksys_driver.c"
#include "usart_driver.h"
#include <util/delay.h>
#include "TC_driver.c"
#include <math.h>
#include "TC_driver.h"

#define CLOCK 32000000
#define WHEEL_DIST 571.52 //wheel distance 
#define WHEEL_RADIAL 498 // shaft distance
#define RA 78.25
#define PI 3.1415926
#define THETA PI/4
#define WHEEL_DIAMETER 156.5
#define REDUCTION_ID 50  //reducer
#define ENCODER_ID 512   //encoder
#define RATIO_ID 102400     //reducer * encoder  * 4

//odom
float xpositionf  = 0;
float ypositionf  = 0;
float robot_theta = 0;
float x_pos=0.0,y_pos=0.0,w_pos=0.0;
 
long int x_=0,y_=0,plus_1=0,plus_3=0,plus_4=0,plus_1_last=0,plus_3_last=0,plus_4_last=0,plus_2=0,plus1_stamp=0,plus3_stamp=0,plus4_stamp=0,plus2_stamp=0,plus_2_last=0,v_1=0,v_3=0,v_4=0,v1_stamp=0,v2_stamp=0,v3_stamp=0,v4_stamp=0,v_2=0,plusv1=0,plusv2=0,plusv3=0,plusv4=0,vel_x = 0,vel_y = 0,vel_w = 0;
long int pr1=0,pr2 = 0, pr3 = 0, pr4 = 0;
unsigned char rev[13],odom1[20],odom2[20],odom3[20],odom4[20];
float x_f=0,y_f=0,t_f=0;
long int inc1=0,inc2=0,inc3=0,inc4=0;
unsigned char it=0,jt=0,mt=0,nt=0;
int t_=0,v_=0,w_=0,count=0,ii=0,jj=0,kk=0,mm=0,ll=0,flag1=1,flag2=1,flag3=1,flag4=1,flag5=1,flag6=1,flag7=1,flag8=1,num1=0,num2=0,num3=0,num4=0,max_vel=1500,max_rvel=60;
bool flg=true,flg2=true,flg1=false;
long int leftplusc = 0, rightplusc = 0;
int vthread = 10;
int vel=0,rvel=0,v1=0,v2=0,v3=0,v4=0,velx=0,vely=0,max_velx = 500,max_vely = 500, min_velx = -500, min_vely = - 500,max_velz = 30,min_velz = -30;
int posx=0, posy=0,posz=0;
///·¢ËÍ×Ö·û
void uart1_putc(unsigned char c)
{ 
    while(!(USARTC0.STATUS & USART_DREIF_bm));
    USARTC0.DATA = c;
}
void uart2_putc(unsigned char c)
{ 
    while(!(USARTC1.STATUS & USART_DREIF_bm));
    USARTC1.DATA = c;
}
void uart3_putc(unsigned char c)
{ 
    while(!(USARTD0.STATUS & USART_DREIF_bm));
    USARTD0.DATA = c;
}
void uart4_putc(unsigned char c)
{ 
    while(!(USARTD1.STATUS & USART_DREIF_bm));
    USARTD1.DATA = c;
}
/*void uart5_putc(unsigned char c)
{ 
    while(!(USARTE0.STATUS & USART_DREIF_bm));
    USARTE0.DATA = c;
}*/
/*void uart6_putc(unsigned char c)
{ 
    while(!(USARTE1.STATUS & USART_DREIF_bm));
    USARTE1.DATA = c;
}*/
/*void uart7_putc(unsigned char c)
{ 
    while(!(USARTF0.STATUS & USART_DREIF_bm));
    USARTF0.DATA = c;
}*/
void uart8_putc(unsigned char c)
{ 
    while(!(USARTF1.STATUS & USART_DREIF_bm));
    USARTF1.DATA = c;
}
///·¢ËÍ×Ö·û´®
void uart1_puts(const char* str)
{
	while(*str)
	  uart1_putc(*str++);
}
void uart2_puts(const char* str)
{
	while(*str)
	  uart2_putc(*str++);
}
void uart3_puts(const char* str)
{
	while(*str)
	  uart3_putc(*str++);
}
void uart4_puts(const char* str)
{
	while(*str)
	  uart4_putc(*str++);
}
/*void uart5_puts(const char* str)
{
	while(*str)
	  uart5_putc(*str++);
}*/
/*void uart6_puts(const char* str)
{
	while(*str)
	  uart6_putc(*str++);
}*/
/*void uart7_puts(const char* str)
{
	while(*str)
	  uart7_putc(*str++);
}*/
void uart8_puts(const char* str)
{
	while(*str)
	  uart8_putc(*str++);
}
///·¢ËÍÊ®½øÖÆÕýÕûÊý
void uart1_putw_udec(unsigned long int w)
{
    unsigned long int num = 1000000000;
    unsigned char started = 0;
    while(num > 0)
    {
        unsigned char b = w / num;
        if(b > 0 || started || num == 1)
        {
            uart1_putc('0' + b);
            started = 1;
        }
        w -= b * num;

        num /= 10;
    }
}
void uart2_putw_udec(unsigned long int w)
{
    unsigned long int num = 1000000000;
    unsigned char started = 0;
    while(num > 0)
    {
        unsigned char b = w / num;
        if(b > 0 || started || num == 1)
        {
            uart2_putc('0' + b);
            started = 1;
        }
        w -= b * num;

        num /= 10;
    }
}
void uart3_putw_udec(unsigned long int w)
{
    unsigned long int num = 1000000000;
    unsigned char started = 0;
    while(num > 0)
    {
        unsigned char b = w / num;
        if(b > 0 || started || num == 1)
        {
            uart3_putc('0' + b);
            started = 1;
        }
        w -= b * num;

        num /= 10;
    }
}
void uart4_putw_udec(unsigned long int w)
{
    unsigned long int num = 1000000000;
    unsigned char started = 0;
    while(num > 0)
    {
        unsigned char b = w / num;
        if(b > 0 || started || num == 1)
        {
            uart4_putc('0' + b);
            started = 1;
        }
        w -= b * num;

        num /= 10;
    }
}
/*void uart5_putw_udec(unsigned long int w)
{
    unsigned long int num = 1000000000;
    unsigned char started = 0;
    while(num > 0)
    {
        unsigned char b = w / num;
        if(b > 0 || started || num == 1)
        {
            uart5_putc('0' + b);
            started = 1;
        }
        w -= b * num;

        num /= 10;
    }
}*/
/*void uart6_putw_udec(unsigned long int w)
{
    unsigned long int num = 1000000000;
    unsigned char started = 0;
    while(num > 0)
    {
        unsigned char b = w / num;
        if(b > 0 || started || num == 1)
        {
            uart6_putc('0' + b);
            started = 1;
        }
        w -= b * num;

        num /= 10;
    }
}*/
/*void uart7_putw_udec(unsigned long int w)
{
    unsigned long int num = 1000000000;
    unsigned char started = 0;
    while(num > 0)
    {
        unsigned char b = w / num;
        if(b > 0 || started || num == 1)
        {
            uart7_putc('0' + b);
            started = 1;
        }
        w -= b * num;

        num /= 10;
    }
}*/
void uart8_putw_udec(unsigned long int w)
{
    unsigned long int num = 1000000000;
    unsigned char started = 0;
    while(num > 0)
    {
        unsigned char b = w / num;
        if(b > 0 || started || num == 1)
        {
            uart8_putc('0' + b);
            started = 1;
        }
        w -= b * num;

        num /= 10;
    }
}
///·¢ËÍ´ø·ûºÅµÄÊ®½øÖÆÕûÊý
void uart1_putw_dec(long int w)
{
 if(w>=0)
   
	uart1_putw_udec(w);
	
 else
   {
    long int m;
	 m=-1*w;
     uart1_putc('-');
	 uart1_putw_udec(m);}	 
}
void uart2_putw_dec(long int w)
{
 if(w>=0)
   
	uart2_putw_udec(w);
	
 else
   {
    long int m;
	m=-1*w;
     uart2_putc('-');
	 uart2_putw_udec(m);}	 
}
void uart3_putw_dec(long int w)
{
 if(w>=0)
   
	uart3_putw_udec(w);
	
 else
   {
    long int m;
	 m=-1*w;
     uart3_putc('-');
	 uart3_putw_udec(m);}	 
}
void uart4_putw_dec(long int w)
{
 if(w>=0)
   
	uart4_putw_udec(w);
	
 else
   {
    long int m;
	 m=-1*w;
     uart4_putc('-');
	 uart4_putw_udec(m);}	 
}
/*void uart5_putw_dec(long int w)
{
 if(w>=0)
   
	uart5_putw_udec(w);
	
 else
   {
    long int m;
	m=-1*w;
     uart5_putc('-');
	 uart5_putw_udec(m);}	 
}*/
/*void uart6_putw_dec(long int w)
{
 if(w>=0)
   
	uart6_putw_udec(w);
	
 else
   {
    long int m;
	m=-1*w;
     uart6_putc('-');
	 uart6_putw_udec(m);}	 
}*/
/*void uart7_putw_dec(long int w)
{
 if(w>=0)
   
	uart7_putw_udec(w);
	
 else
   {
    long int m;
	m=-1*w;
     uart7_putc('-');
	 uart7_putw_udec(m);}	 
}*/
void uart8_putw_dec(long int w)
{
 if(w>=0)
   
	uart8_putw_udec(w);
	
 else
   {
    long int m;
	m=-1*w;
     uart8_putc('-');
	 uart8_putw_udec(m);}	 
}
///´®¿Ú³õÊ¼»¯£¨²¨ÌØÂÊ£©
void UART_INIT (long int baud1,long int baud2,long int baud3,long int baud4,long int baud8)
{
    // Want 9600 baud. Have a 32 MHz clock. BSCALE = 0
    // BSEL = ( 32000000 / (2^2 * 16*9600)) -1 = 51
    // Fbaud = 32000000 / (2^2 * 16 * (51+1))  = 9615 bits/sec (9600 -0.1%)
	PORTC.DIRSET = PIN3_bm;
	PORTC.OUTSET = PIN3_bm;
	PORTC.DIRCLR = PIN2_bm;
	PORTC.OUTSET = PIN2_bm;
	USART_Format_Set(&USARTC0, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
    USART_Baudrate_Set(&USARTC0,(int)(CLOCK/32/baud1), 1);
	USART_Rx_Enable(&USARTC0);
	USART_Tx_Enable(&USARTC0);
	
	PORTC.DIRSET = PIN7_bm;
	PORTC.OUTSET = PIN7_bm;
	PORTC.DIRCLR = PIN6_bm;
	PORTC.OUTSET = PIN6_bm;
	USART_Format_Set(&USARTC1, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
    USART_Baudrate_Set(&USARTC1,(int)(CLOCK/32/baud2), 1);
	USART_Rx_Enable(&USARTC1);
	USART_Tx_Enable(&USARTC1);
	
	PORTD.DIRSET = PIN3_bm;
	PORTD.OUTSET = PIN3_bm;
	PORTD.DIRCLR = PIN2_bm;
	PORTD.OUTSET = PIN2_bm;
	USART_Format_Set(&USARTD0, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
    USART_Baudrate_Set(&USARTD0, (int)(CLOCK/32/baud3) , 1);
	USART_Rx_Enable(&USARTD0);
	USART_Tx_Enable(&USARTD0);
	
	PORTD.DIRSET = PIN7_bm;
	PORTD.OUTSET = PIN7_bm;
	PORTD.DIRCLR = PIN6_bm;
	PORTD.OUTSET = PIN6_bm;
	USART_Format_Set(&USARTD1, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_Baudrate_Set(&USARTD1,(int)(CLOCK/32/baud4), 1);
	USART_Rx_Enable(&USARTD1);
	USART_Tx_Enable(&USARTD1);
	
	/*PORTE.DIRSET = PIN3_bm;
	PORTE.OUTSET = PIN3_bm;
	PORTE.DIRCLR = PIN2_bm;
	PORTE.OUTSET = PIN2_bm;
	USART_Format_Set(&USARTE0, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_Baudrate_Set(&USARTE0,(int)(CLOCK/32/baud5), 1);
	USART_Rx_Enable(&USARTE0);
	USART_Tx_Enable(&USARTE0);*/

	/*PORTE.DIRSET = PIN7_bm;
	PORTE.OUTSET = PIN7_bm;
	PORTE.DIRCLR = PIN6_bm;
	PORTE.OUTSET = PIN6_bm;
	USART_Format_Set(&USARTE1, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_Baudrate_Set(&USARTE1,(int)(CLOCK/32/baud6), 1);
	USART_Rx_Enable(&USARTE1);
	USART_Tx_Enable(&USARTE1);*/
	
	/*PORTF.DIRSET = PIN3_bm;
	PORTF.OUTSET = PIN3_bm;
	PORTF.DIRCLR = PIN2_bm;
	PORTF.OUTSET = PIN2_bm;
	USART_Format_Set(&USARTF0, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_Baudrate_Set(&USARTF0,(int)(CLOCK/32/baud7), 1);
	USART_Rx_Enable(&USARTF0);
	USART_Tx_Enable(&USARTF0);*/

	PORTF.DIRSET = PIN7_bm;
	PORTF.OUTSET = PIN7_bm;
	PORTF.DIRCLR = PIN6_bm;
	PORTF.OUTSET = PIN6_bm;
	USART_Format_Set(&USARTF1, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
    USART_Baudrate_Set(&USARTF1,(int)(CLOCK/32/baud8), 1);
	USART_Rx_Enable(&USARTF1);
	USART_Tx_Enable(&USARTF1);
}
///½ÓÊÕÖÐ¶Ï³õÊ¼»¯
void RXC_INI_INIT()
{
  
  USART_RxdInterruptLevel_Set(&USARTC0,USART_RXCINTLVL_HI_gc);
  USART_RxdInterruptLevel_Set(&USARTC1,USART_RXCINTLVL_HI_gc);
  USART_RxdInterruptLevel_Set(&USARTD0,USART_RXCINTLVL_HI_gc);
  USART_RxdInterruptLevel_Set(&USARTD1,USART_RXCINTLVL_HI_gc);
 // USART_RxdInterruptLevel_Set(&USARTE0,USART_RXCINTLVL_HI_gc);
 // USART_RxdInterruptLevel_Set(&USARTE1,USART_RXCINTLVL_HI_gc);
 // USART_RxdInterruptLevel_Set(&USARTF0,USART_RXCINTLVL_HI_gc);
  USART_RxdInterruptLevel_Set(&USARTF1,USART_RXCINTLVL_HI_gc);

  PMIC.CTRL|=PMIC_HILVLEN_bm;       
  sei();                  
}
///¶¨Ê±Æ÷³õÊ¼»¯
void TIMER0_INIT(void)
{
	//ÖÐ¶ÏÊ±¼äÎª50ms
  TC_SetPeriod(&TCC0,1562);     //ÉèÖÃ¶¨Ê±Æ÷¼ÆÊý·¶Î§
  TC_SetCount( &TCC0,0);        //ÉèÖÃ¶¨Ê±Æ÷´Ó0¿ªÊ¼¼ÆÊý
  TC0_SetOverflowIntLevel(&TCC0,TC_OVFINTLVL_LO_gc); //ÉèÖÃ¶¨Ê±Æ÷Òç³öÖÐ¶ÏÎªµÍ¼¶±ð
  TC0_ConfigClockSource(&TCC0,TC_CLKSEL_DIV1024_gc); //ÉèÖÃ¶¨Ê±Æ÷µÄÊ±ÖÓÆµÂÊÎªÏµÍ³Ê±ÖÓÆµÂÊµÄ1/1024
  PMIC.CTRL|=PMIC_LOLVLEN_bm;   //ÔÊÐíµÍ¼¶±ðÖÐ¶Ï
  sei();                        //Æô¶¯È«¾ÖÖÐ¶Ï
}

///¼ÆËãÀï³Ì¼Æº¯Êý
void ELMO_ChassisGetPosition()
{
	//float wheel_halfdist = WHEEL_DIST/2.0;   //?????����?��?��?(�̣�??:mm) d/2
	float ratio_mmtocode = RATIO_ID/1.0/PI/WHEEL_DIAMETER;

	float vxpositionf  = 0;
	float vypositionf  = 0;
	float rpositionf   = 0;
	
	float r_vel = 0;
	float r_pos = 0;
	
	//long int w1, w2, w3, w4;
	long int vel1=0,vel2=0,vel3=0,vel4=0;
		
	
	vel1=v_1/ratio_mmtocode;
	vel2=v_2/ratio_mmtocode;
	vel3=v_3/ratio_mmtocode;
	vel4=v_4/ratio_mmtocode;
	
	//get vel of bot
	vxpositionf= (vel1+vel2+vel3+vel4)/4;
	vypositionf= (vel1-vel2+vel3-vel4)/4;
	rpositionf = (vel1-vel2-vel3+vel4) / (WHEEL_DIST + WHEEL_RADIAL)/4 * 10;

	//rad 2 degree
	r_vel = rpositionf * 180 / PI + 0.5;

	//////////2016-10-09
	/*
	//get vel of bot
	vxpositionf= (+vel1+vel2+vel3+vel4)/4;
	vypositionf= (- vel1+vel2-vel3+vel4)/4;
	rpositionf  = (   vel1-vel2-vel3+vel4)/(WHEEL_DIST+WHEEL_RADIAL);
	*/
	
	//get position from encoder;
	//delta_x delta_y delta_w;
	x_pos = RA * tan(THETA) * (+inc1+inc2+inc3+inc4) / RATIO_ID * 2 * PI/4;
	y_pos = RA * tan(THETA) * ( inc1-inc2+inc3-inc4) / RATIO_ID * 2 * PI/4;
	w_pos = 10 * RA * tan(THETA) * ( inc1-inc2-inc3+inc4) / RATIO_ID * 2 * PI/(WHEEL_DIST/2.0+WHEEL_RADIAL/2.0*1/tan(THETA))/4;

	
	//get position;
	xpositionf  = xpositionf  + x_pos*sin(robot_theta/10)+y_pos*cos(robot_theta/10);
	ypositionf  = ypositionf  + x_pos*cos(robot_theta/10)-y_pos*sin(robot_theta/10);
	robot_theta = robot_theta + w_pos;

	//rad 2 degree
	r_pos = robot_theta * 180 / PI;
	
	///////2016-10-09
	/*
	//get position from encoder;
	xpositionf    = xpositionf   + RA  * tan(THETA) * (-inc1+inc2  -inc3+inc4) / RATIO_ID * 2 * PI/4;
	ypositionf    = ypositionf   + RA  * tan(THETA) * (  inc1+inc2+inc3+inc4) / RATIO_ID * 2 * PI/4;
	robot_theta = robot_theta + RA * tan(THETA) * (  inc1 -inc2- inc3+inc4) / RATIO_ID * 2 * PI / ( WHEEL_DIST/2.0+WHEEL_RADIAL/2.0*1/tan(THETA) );
*/
    
	/*xpositionf    = x_f + RA / 4.0 * tan(THETA) * (-inc1+inc2-inc3+inc4) / RATIO_ID * 2 * PI;
	ypositionf    = y_f + RA / 4.0 * tan(THETA) * (inc1+inc2+inc3+inc4) / RATIO_ID * 2 * PI;
	robot_theta = t_f  + RA / 4.0 * tan(THETA) * (inc1-inc2-inc3+inc4) / RATIO_ID * 2 * PI / ( WHEEL_DIST/2.0+WHEEL_RADIAL/2.0*1/tan(THETA) );

	vxpositionf = ( -w1 + w2 - w3 + w4 ) * WHEEL_DIAMETER / 8;
	vypositionf = (  w1 + w2 + w3 + w4 ) * WHEEL_DIAMETER / 8;
	rpositionf   = (  w1 -  w2 - w3 + w4 ) * WHEEL_DIAMETER / 8 / WHEEL_DIST / WHEEL_RADIAL;*/

    if(r_pos > 1800)                //???����??��?a180~-180
		r_pos = r_pos - 3600;

	if(r_pos <= -1800)
		r_pos = r_pos + 3600; 
		
	x_ = (long int)(xpositionf+0.5);
	y_ = (long int)(ypositionf+0.5);
	t_ =(int)(r_pos+0.5);
	
	vel_x = (long int)(vxpositionf);
	vel_y = (long int)(vypositionf);
	vel_w = (long int)(r_vel);
}

bool pos2convert(int posx,int posy,int posz)
{
		float coefficient22 = (WHEEL_RADIAL/2.0*tan(THETA)+WHEEL_DIST/2.0) * posz/10 /180.0*PI;

		pr1=(long int)(posx+posy+coefficient22)*RATIO_ID/WHEEL_DIAMETER/PI;
		pr2=(long int)(posx-posy-coefficient22)*RATIO_ID/WHEEL_DIAMETER/PI;
		pr3=(long int)(posx+posy-coefficient22)*RATIO_ID/WHEEL_DIAMETER/PI;
		pr4=(long int)(posx-posy+coefficient22)*RATIO_ID/WHEEL_DIAMETER/PI;
		
	//////////2016-10-09
	/*if( posy ==0 && posz ==0)
	{
		pr1=(long int)(posx/PI/WHEEL_DIAMETER*RATIO_ID);
		pr2=pr1;
		pr3=pr2;
		pr4=pr3;
	}
	else
	{
		float coefficient1 = posy*tan(THETA);
		float coefficient2 = ( WHEEL_RADIAL / 2.0 * tan( THETA ) + WHEEL_DIST / 2.0 ) * posz * PI / 180.0;
		pr1=(long int)(posx-coefficient1+coefficient2 )*RATIO_ID/WHEEL_DIAMETER;
		pr2=(long int)(posx+coefficient1-coefficient2 )*RATIO_ID/WHEEL_DIAMETER;
		pr3=(long int)(posx-coefficient1-coefficient2 )*RATIO_ID/WHEEL_DIAMETER;
		pr4=(long int)(posx+coefficient1+coefficient2 )*RATIO_ID/WHEEL_DIAMETER;
	}
	if(posx == 0 && posy == 0)
	{
		pr1 = pr1 / 7;
		pr2 = pr2 / 7;
		pr3 = pr3 / 7;
		pr4 = pr4 / 7;
	}*/
	
	return true;
}

///ËÙ¶È×ª»»º¯Êý
bool convert(int vx,int vy,int w)//degree
{
		float coefficient2  = (WHEEL_RADIAL/2.0*tan(THETA)+WHEEL_DIST/2.0) * w/10 /180.0*PI;

		plusv1=(long int)(vx+vy+coefficient2 )*RATIO_ID/WHEEL_DIAMETER;
		plusv2=(long int)(vx-vy-coefficient2 )*RATIO_ID/WHEEL_DIAMETER;
		plusv3=(long int)(vx+vy-coefficient2 )*RATIO_ID/WHEEL_DIAMETER;
		plusv4=(long int)(vx-vy+coefficient2 )*RATIO_ID/WHEEL_DIAMETER;
	/////////////////////////////2016-10-09
	/*if(w ==0 && vy ==0)
	{
		plusv1=(long int)(vx  /PI/WHEEL_DIAMETER*RATIO_ID);
		plusv2=plusv1;
		plusv3=plusv2;
		plusv4=plusv3;
	}
	else
	{
		float coefficient1 = vy*tan(THETA);
		float coefficient2 = (WHEEL_RADIAL/2.0*tan(THETA)+WHEEL_DIST/2.0)*w/10*PI/180.0;
		plusv1=(long int)(vx-coefficient1+coefficient2 )*RATIO_ID/WHEEL_DIAMETER;
		plusv2=(long int)(vx+coefficient1-coefficient2 )*RATIO_ID/WHEEL_DIAMETER;
		plusv3=(long int)(vx-coefficient1-coefficient2 )*RATIO_ID/WHEEL_DIAMETER;
		plusv4=(long int)(vx+coefficient1+coefficient2 )*RATIO_ID/WHEEL_DIAMETER;
	}*/
	return true;
}
///·¢ËÍÖ¸Áîº¯Êý
void sendCMD(unsigned char* buf,unsigned char cmd)
{
	 int flag=1/*,max_pv=0,max_ac=0,max_dc=0*/;
	 //long int sp=0,ac=0,dc=0;
	 //int dist=0,deg=0,degv=0,v11=0,v22=0;
	 //float ddeg=0;
	
	 unsigned char str1=0,str2=0,str3=0/*,str4=0,str5=0,str6=0,str7=0,str8=0*/;
	 switch(cmd){
		 
		/* case 0x11://ichange max vel to 0.6m/s (120000); defalut: 600000 
	 	uart1_puts("MO=0\n");
		uart2_puts("MO=0\n");
		uart3_puts("MO=0\n");
		uart4_puts("MO=0\n");
		_delay_ms(10);
		
			 uart1_puts("UM=5\n");
			 uart2_puts("UM=5\n");
			 uart3_puts("UM=5\n");
			 uart4_puts("UM=5\n");
			 _delay_ms(10);
		
		 uart1_puts("SP=120000\n");
		uart2_puts("SP=120000\n");
		uart3_puts("SP=120000\n");
		uart4_puts("SP=120000\n");
		_delay_ms(10);
	 	break;
		 
		 case 0x12://ichange max vel to 0.15m/s (30000); defalut: 600000
	 	uart1_puts("MO=0\n");
		uart2_puts("MO=0\n");
		uart3_puts("MO=0\n");
		uart4_puts("MO=0\n");
		_delay_ms(10);
		
			 uart1_puts("UM=5\n");
			 uart2_puts("UM=5\n");
			 uart3_puts("UM=5\n");
			 uart4_puts("UM=5\n");
			 _delay_ms(10);
		
		 uart1_puts("SP=30000\n");
		uart2_puts("SP=30000\n");
		uart3_puts("SP=30000\n");
		uart4_puts("SP=30000\n");
		_delay_ms(10);
	 	break;*/


	 	case 0x13:
	 	uart1_puts("ST\n");
		uart2_puts("ST\n");
		uart3_puts("ST\n");
		uart4_puts("ST\n");
	 	break;


	 	//case 0x14
	 	case 0x14:
		 str1=*(buf+3);
		 str2=*(buf+4);
		 str3=*(buf+5);
		 if (str1&0x80)
		 {
			 flag=-1;
			 str1 = (str1 & 0x7f);
		 }
		 else
		 {
			 flag=1;
		 }
		 posx = ( ( ( str1<<16 )+str2<<8 )+str3 )*flag;

			
		 str1=*(buf+6);
		 str2=*(buf+7);
		 str3=*(buf+8);
		 if (str1&0x80)
		 {
			 flag=-1;
			 str1 = (str1 & 0x7f);
		 }
		 else
		 {
			 flag=1;
		 }
		 posy = ( ( ( str1<<16 )+str2<<8 )+str3 )*flag;
		 
			
		 str1=*(buf+9);
		 str2=*(buf+10);
		 if (str1&0x80)
		 {
			 flag=-1;
			 str1 = (str1 & 0x7f);
		 }
		 else
		 {
			 flag=1;
		 }
		 posz = ((str1<<8)+str2)*flag;

		//turning control;
		/* if( posz > 1800 )
		 	posz = posz - 3600;

		 if( posz < 1800 )
		 	posz = posz + 3600;*/


		 if ( pos2convert(posx,posy,posz) )
		 {
			 if(posx==0 && posy	== 0)
			 { 
			 uart1_puts("MO=0\n");
			 uart2_puts("MO=0\n");
			 uart3_puts("MO=0\n");
			 uart4_puts("MO=0\n");
			 _delay_ms(10);

			 uart1_puts("UM=5\n");
			 uart2_puts("UM=5\n");
			 uart3_puts("UM=5\n");
			 uart4_puts("UM=5\n");
			 _delay_ms(10);
			 
			 uart1_puts("SP=30000\n");
			 uart2_puts("SP=30000\n");
			 uart3_puts("SP=30000\n");
			 uart4_puts("SP=30000\n");
			 _delay_ms(10);

			 uart1_puts("MO=1\n");
			 uart2_puts("MO=1\n");
			 uart3_puts("MO=1\n");
			 uart4_puts("MO=1\n");
			 _delay_ms(10);

			 uart1_puts("PR=");
			 uart2_puts("PR=");
			 uart3_puts("PR=");
			 uart4_puts("PR=");
			 _delay_ms(10);

			 uart1_putw_dec(pr1);
			 uart2_putw_dec(pr2);
			 uart3_putw_dec(pr3);
			 uart4_putw_dec(pr4);
			 uart1_putc('\n');
			 uart2_putc('\n');
			 uart3_putc('\n');
			 uart4_putc('\n');
			 _delay_ms(10);

			 uart1_puts("BG\n");
			 uart2_puts("BG\n");
			 uart3_puts("BG\n");
			 uart4_puts("BG\n");
			 }
			 else
			 {
			  uart1_puts("MO=0\n");
			 uart2_puts("MO=0\n");
			 uart3_puts("MO=0\n");
			 uart4_puts("MO=0\n");
			 _delay_ms(10);

			 uart1_puts("UM=5\n");
			 uart2_puts("UM=5\n");
			 uart3_puts("UM=5\n");
			 uart4_puts("UM=5\n");
			 _delay_ms(10);
			 
			  uart1_puts("SP=60000\n");
			 uart2_puts("SP=60000\n");
			 uart3_puts("SP=60000\n");
			 uart4_puts("SP=60000\n");
			 _delay_ms(10);

			 uart1_puts("MO=1\n");
			 uart2_puts("MO=1\n");
			 uart3_puts("MO=1\n");
			 uart4_puts("MO=1\n");
			 _delay_ms(10);

			 uart1_puts("PR=");
			 uart2_puts("PR=");
			 uart3_puts("PR=");
			 uart4_puts("PR=");
			 _delay_ms(10);

			 uart1_putw_dec(pr1);
			 uart2_putw_dec(pr2);
			 uart3_putw_dec(pr3);
			 uart4_putw_dec(pr4);
			 uart1_putc('\n');
			 uart2_putc('\n');
			 uart3_putc('\n');
			 uart4_putc('\n');
			 _delay_ms(10);

			 uart1_puts("BG\n");
			 uart2_puts("BG\n");
			 uart3_puts("BG\n");
			 uart4_puts("BG\n");
			 }				 
		 }
		 break;


		 //case 0x15
		 case 0x15:
		 str1=*(buf+3);
		 str2=*(buf+4);
		 if (str1&0x80)
		 {
			 flag=-1;
			 str1 = (str1 & 0x7f);
		 }
		 else
		 {
			 flag=1;
		 }
		 velx = ((str1<<8)+str2)*flag;
		 if(velx > max_velx)
		    velx = max_velx;
		 if(velx < min_velx)
		    velx = min_velx;
			
		 str1=*(buf+5);
		 str2=*(buf+6);
		 if (str1&0x80)
		 {
			 flag=-1;
			 str1 = (str1 & 0x7f);
		 }
		 else
		 {
			 flag=1;
		 }
		 vely = ((str1<<8)+str2)*flag;
		 if(vely > max_vely)
		    vely = max_vely;
		 if(vely < min_vely)
		    vely = min_vely;
			
		 str1=*(buf+7);
		 str2=*(buf+8);
		 if (str1&0x80)
		 {
			 flag=-1;
			 str1 = (str1 & 0x7f);
		 }
		 else
		 {
			 flag=1;
		 }
		 rvel = ((str1<<8)+str2)*flag;
		 if(rvel > max_velz)
		    rvel = max_velz;
		 if(rvel < min_velz)
		    rvel = min_velz;
		 if (convert(velx,vely,rvel))
		 {
			 uart1_puts("MO=1\n");
			 uart2_puts("MO=1\n");
			 uart3_puts("MO=1\n");
			 uart4_puts("MO=1\n");
			 _delay_ms(10);
			 uart1_puts("JV=");
			 uart2_puts("JV=");
			 uart3_puts("JV=");
			 uart4_puts("JV=");
			 _delay_ms(10);
			 uart1_putw_dec(plusv1);
			 uart2_putw_dec(plusv2);
			 uart3_putw_dec(plusv3);
			 uart4_putw_dec(plusv4);
			 uart1_putc('\n');
			 uart2_putc('\n');
			 uart3_putc('\n');
			 uart4_putc('\n');
			 _delay_ms(10);
			 uart1_puts("BG\n");
			 uart2_puts("BG\n");
			 uart3_puts("BG\n");
			 uart4_puts("BG\n");
		 }
		 break;
		default:
		 break;
	 }		 
}
	
int main(void)
{
	CLKSYS_Enable( OSC_RC32MEN_bm );
	do {} while ( CLKSYS_IsReady( OSC_RC32MRDY_bm ) == 0 );
	CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_RC32M_gc );
	CLKSYS_Disable( OSC_RC2MEN_bm );
	UART_INIT(19200,19200,19200,19200,115200);//JSC 19200,19200,19200,19200,115200
	RXC_INI_INIT();
	TIMER0_INIT();
    while(1);
}

ISR(TCC0_OVF_vect)
{
	count++;
	TC_SetCount(&TCC0,0);
	unsigned char data[22];
	  if (flg)
	  {
		 uart1_puts("VX\n");
	     uart2_puts("VX\n");
		 uart3_puts("VX\n");
	     uart4_puts("VX\n");
		 flg=false;
	  }
	  else
	  {
	     uart1_puts("PX\n");
		 uart2_puts("PX\n");
		 uart3_puts("PX\n");
		 uart4_puts("PX\n");
		 flg=true;
	  } 
	   if(count >= 2)
	  {    
	    ELMO_ChassisGetPosition();
		count=0;
		long int temp=0;
		int temp2=0;
		data[0]=0xfa;
		data[1]=0xfb;
		
		//x-pos
	    if (y_< 0)
	   	{
			temp=-y_;
			data[2] = ((temp>>16) | 0x80);	
	    	}
		else
		{
			temp=y_;
			data[2] = temp >>16;
		}
		data[3] = temp >>8;
		data[4] = temp;
		
		//y-pos
	    if (x_< 0)
	    	{
			temp=-x_;
			data[5] = ((temp>>16) | 0x80);	
	    	}
		else
		{
			temp=x_;
			data[5] = temp >>16;
		}
		data[6] = temp >>8;		
		data[7] = temp;
		
		//th-pos
	    if (t_< 0)
	    	{
			temp2=-t_;
			data[8] = ((temp2>>8) | 0x80);	
	    	}
		else
		{
			temp2=t_;
			data[8] = temp2 >>8;
		}
		data[9] = temp2;
		
		//x-vel
	    if (vel_x< 0)
	    {
			temp2=-vel_x;
			data[10] = ((temp2>>8) | 0x80);	
	    }
		else
		{
			temp2=vel_x;
			data[10] = temp2 >>8;
		}	   
		data[11] = temp2 ;
		
		//y-vel
	    if (vel_y< 0)
	    {
			temp2=-vel_y;
			data[12] = ((temp2>>8) | 0x80);	
	    }
		else
		{
			temp2=vel_y;
			data[12] = temp2 >>8;
		}	
		data[13] = temp2;
		
		//w-th
		if (vel_w< 0)
	    {
			temp2=-vel_w;
			data[14] = ((temp2>>8) | 0x80);	
	    }
		else
		{
			temp2=vel_w;
			data[14] = temp2 >>8;
		}	
		data[15] = temp2;

		data[16] = 0;
		data[17] = 0;
	
		data[18]=data[2]^data[4]^data[6]^data[8]^data[10]^data[12]^data[14]^data[16];
		data[19]=data[3]^data[5]^data[7]^data[9]^data[11]^data[13]^data[15]^data[17];
		
		data[20] = 0xfa;
		data[21] = 0xfb;
	
		for (int i=0;i<22;i++)
		{
			uart8_putc(data[i]);
		}
	}
}	


ISR(USARTC0_RXC_vect)
{
	  unsigned char buf;
      	  buf=USART_GetChar(&USARTC0);
	  odom1[jj++]=buf;
	  if (jj == 1)
	  {
		  if (odom1[0] !=0x50)
		    if(odom1[0] !=0x56)
			  jj = 0;  
	  }
	  if (jj == 2)
	  {
		  if (odom1[1] !=0x58)
			  jj = 0;
	  } 
	  num1=1000;
	  if(odom1[jj-1] == 0x3b)
	     num1=jj-1;
	  if(jj>=num1)
	  {
		   jj=0;	
		   int i=0;
		   
           		  while (i <= num1)
		   {
			   if(odom1[0] == 0x50) 
		       {
				   if(odom1[i]>=0x30 && odom1[i]<=0x39)
		  	         plus1_stamp=plus1_stamp*10 + odom1[i]-0x30;
		           if(odom1[i] == 0x2d)
			         flag1=-1;
			       i++;
			   }
			   else if (odom1[0] == 0x56)
			   {
				   if(odom1[i]>=0x30 && odom1[i]<=0x39)
				      v1_stamp=v1_stamp*10 + odom1[i]-0x30;
				   if(odom1[i] == 0x2d)
			          flag2=-1;
				   i++;
			   }
			}
			if (odom1[0] == 0x50)
			{
				plus_1_last=plus_1;
				plus_1=plus1_stamp*flag1;
				inc1=plus_1-plus_1_last;
	            			plus1_stamp=0;
	            			flag1=1;
			}
			else if (odom1[0] == 0x56)
			{
				  v_1=v1_stamp*flag2;
				  v1_stamp=0;
				  flag2=1;  

			}
	  }							
}
ISR(USARTC1_RXC_vect)
{
	  unsigned char buf;
      	  buf=USART_GetChar(&USARTC1);
	  odom2[kk++]=buf;
	  if (kk == 1)
	  {
		  if (odom2[0] !=0x50)
		    if(odom2[0] !=0x56)
			  kk = 0;  
	  }
	  if (kk == 2)
	  {
		  if (odom2[1] !=0x58)
			  kk = 0;
	  } 
	  num2=1000;
	  if(odom2[kk-1] == 0x3b)
	     num2=kk-1;
	  if(kk>=num2)
	  {
		   kk=0;	
		   int i=0;
		   
           while (i <= num2)
		   {
			   if(odom2[0] == 0x50) 
		       {
				   if(odom2[i]>=0x30 && odom2[i]<=0x39)
		  	         plus2_stamp=plus2_stamp*10 + odom2[i]-0x30;
		           if(odom2[i] == 0x2d)
			         flag3=-1;
			       i++;
			   }
			   else if (odom2[0] == 0x56)
			   {
				   if(odom2[i]>=0x30 && odom2[i]<=0x39)
				      v2_stamp=v2_stamp*10 + odom2[i]-0x30;
				   if(odom2[i] == 0x2d)
			          flag4=-1;
				   i++;
			   }
			}
			if (odom2[0] == 0x50)
			{
				plus_2_last=plus_2;
				plus_2=plus2_stamp*flag3;
				inc2=plus_2-plus_2_last;
	            			plus2_stamp=0;
	            			flag3=1;
			}
			else if (odom2[0] == 0x56)
			{
				  v_2=v2_stamp*flag4;
				  v2_stamp=0;
				  flag4=1;  

			}
	  }	
}
ISR(USARTD0_RXC_vect)
{
	  unsigned char buf;
     	  buf=USART_GetChar(&USARTD0);
	  odom3[ll++]=buf;
	  if (ll == 1)
	  {
		  if (odom3[0] !=0x50)
		    if(odom3[0] !=0x56)
			  ll = 0;  
	  }
	  if (ll == 2)
	  {
		  if (odom3[1] !=0x58)
			  ll = 0;
	  } 
	  num3=1000;
	  if(odom3[ll-1] == 0x3b)
	     num3=ll-1;
	  if(ll>=num3)
	  {
		   ll=0;	
		   int i=0;
		   
           while (i <= num3)
		   {
			   if(odom3[0] == 0x50) 
		       {
				   if(odom3[i]>=0x30 && odom3[i]<=0x39)
		  	         plus3_stamp=plus3_stamp*10 + odom3[i]-0x30;
		           if(odom3[i] == 0x2d)
			         flag5=-1;
			       i++;
			   }
			   else if (odom3[0] == 0x56)
			   {
				   if(odom3[i]>=0x30 && odom3[i]<=0x39)
				      v3_stamp=v3_stamp*10 + odom3[i]-0x30;
				   if(odom3[i] == 0x2d)
			          flag6=-1;
				   i++;
			   }
			}
			if (odom3[0] == 0x50)
			{
				plus_3_last=plus_3;
				plus_3=plus3_stamp*flag5;
				inc3=plus_3-plus_3_last;
	            plus3_stamp=0;
	            flag5=1;

			}
			else if (odom3[0] == 0x56)
			{
				  v_3=v3_stamp*flag6;
				  v3_stamp=0;
				  flag6=1;  

			}
	  }							
}
ISR(USARTD1_RXC_vect)
{
	  unsigned char buf;
       	  buf=USART_GetChar(&USARTD1);
	  odom4[mm++]=buf;
	  if (mm == 1)
	  {
		  if (odom4[0] !=0x50)
		    if(odom4[0] !=0x56)
			  mm = 0;  
	  }
	  if (mm == 2)
	  {
		  if (odom4[1] !=0x58)
			  mm = 0;
	  } 
	  num4=1000;
	  if(odom4[mm-1] == 0x3b)
	     num4=mm-1;
	  if(mm>=num4)
	  {
		   mm=0;	
		   int i=0;
		   
           while (i <= num4)
		   {
			   if(odom4[0] == 0x50) 
		       {
				   if(odom4[i]>=0x30 && odom4[i]<=0x39)
		  	         plus4_stamp=plus4_stamp*10 + odom4[i]-0x30;
		           if(odom4[i] == 0x2d)
			         flag7=-1;
			       i++;
			   }
			   else if (odom4[0] == 0x56)
			   {
				   if(odom4[i]>=0x30 && odom4[i]<=0x39)
				      v4_stamp=v4_stamp*10 + odom4[i]-0x30;
				   if(odom4[i] == 0x2d)
			          flag8=-1;
				   i++;
			   }
			}
			if (odom4[0] == 0x50)
			{
				plus_4_last=plus_4;
				plus_4=plus4_stamp*flag7;
				inc4=plus_4-plus_4_last;
	            plus4_stamp=0;
	            flag7=1;

			}
			else if (odom4[0] == 0x56)
			{
				  v_4=v4_stamp*flag8;
				  v4_stamp=0;
				  flag8=1;  

			}
	  }							
}
ISR(USARTF1_RXC_vect)
{
	  unsigned char buf;
	  buf=USART_GetChar(&USARTF1);
	  rev[ii++]=buf;
	  if(ii == 1)
	  {
	     if (rev[0] != 0xfa)
		     ii=0;     		  
	  }
	  if (ii == 2)
	  {
		  if(rev[1] !=0xfb)
		     ii=0;
	  }
	  if (ii >= 13)
	  {
		  ii=0;
		  if((rev[11] == rev[3]^rev[5]^rev[7]^rev[9]) && (rev[12] == rev[4]^rev[6]^rev[8]^rev[10]))
		  {
			  sendCMD(rev,rev[2]);
		  }
	  }
}