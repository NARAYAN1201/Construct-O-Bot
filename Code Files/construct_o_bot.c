#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include<stdio.h>
#include<conio.h>
#include <math.h> //included to support power function
#include "lcd.h"
#define trsh 50
#define INFINITY 9999

//declarations of the functions

void dijkstra(int G[16][16],int n,int startnode,int endnode);
void h(int G[16][16],int house);
int arena(int start,int end);
void eyantra();

//defining global variables 

int floor_array[5]={1,0,0,1,0};//for taking the priority for high rise house (1) and low level(0)
int house_total_requirement[5] = {2,2,2,1,2}; //represent no . of requirement of boxes
int which_material[10]={0,0,0,0,0,0,0,0,2,1};//represent the address of blocks 
int left,right,up,down;//directions variables defined globally for everywhere accessible
int current_node = 1; //for checking up the current status of bot based on position

int err =0;
unsigned char ADC_Conversion(unsigned char);
unsigned char LWS,RWS,MWS;
unsigned char sharp1,sharp2, distance, adc_reading;
unsigned int value1,value2,pickd;
volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle in degrees for turning
//function for setting the directions
void set_direction()
{
    left =0; //taking left ,right ,up and down directions
    right = 0;
    up =0;
    down =1;//by default we are taking downward direction
}
//ADC pin configuration
void adc_pin_config (void)
{
	DDRF = 0x00; //set PORTF direction as input
	PORTF = 0x00; //set PORTF pins floating
	DDRK = 0x00; //set PORTK direction as input
	PORTK = 0x00; //set PORTK pins floating
}
//Configure PORTB 5 pin for servo motor 1 operation
void servo1_pin_config (void)
{
	DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
	PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
	DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
	PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}
void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F; //set direction of the PORTA 3 to PORTA 0 pins as output
	PORTA = PORTA & 0xF0; // set initial value of the PORTA 3 to PORTA 0 pins to logic 0
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM
}
//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}
void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}

//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}
//TIMER1 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz 
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}


// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function to Initialize PORTS
void port_init()
{
	motion_pin_config();
	servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
	servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation
	lcd_port_config();
	adc_pin_config();
	left_encoder_pin_config();
	right_encoder_pin_config();
}

//Function to Initialize ADC
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}



//This Function accepts the Channel Number and returns the corresponding Analog Value
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	_delay_ms(2);
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}


// This Function calculates the actual distance in millimeters(mm) from the input
// analog value of Sharp Sensor.
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}
//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 			// removing upper nibbel as it is not needed
	PortARestore = PORTA; 			// reading the PORTA's original status
	PortARestore &= 0xF0; 			// setting lower direction nibbel to 0
	PortARestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTA status
	PORTA = PortARestore; 			// setting the command to the port
}


void forwardd (void) //both wheels forward
{
	motion_set(0x06);
}

void back (void) //both wheels backward
{
	motion_set(0x09);
}

void leftt (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void rightt (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
	motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
	motion_set(0x08);
}

void stop (void) //hard stop
{
	motion_set(0x00);
}
void readsensors()
{
	LWS = ADC_Conversion(1);
	MWS = ADC_Conversion(2);
	RWS = ADC_Conversion(3);
	lcd_print(2, 10, LWS, 3);                       //Prints Value of White Line Sensor1
	lcd_print(2, 6, MWS, 3);                       //Prints Value of White Line Sensor2
	lcd_print(2, 2, RWS, 3);                       //Prints Value of White Line Sensor3
}
void forward()
{
    unsigned int node=1;
	unsigned int count = 0;
	while(count<node)
	{
		readsensors();
		if(LWS<trsh && MWS>trsh && RWS<trsh)//forward
		{
			forwardd();
			velocity(80,77);
			err = 0;                 //initializing previous error
		}
		else if(LWS>trsh && MWS<trsh && RWS<trsh)//left
		{
			soft_left();
			velocity(70,80);
			err = 1;                 //initializing previous error
		}
		else if(LWS<trsh && MWS<trsh && RWS>trsh)//right
		{
			soft_right();
			velocity(80,70);
			err = 2;                 //initializing previous error
		}
		else if(LWS<trsh && MWS<trsh && RWS<trsh && err == 0)//if the line is thin then error is used to  
		{                                                    //repsoition the bot and the are considered as the  
			wall_data();                                 //previous error.
			forwardd();
			velocity(80,77);
			err = 3;                 //initializing another error
		}
		else if(LWS<trsh && MWS<trsh && RWS<trsh && err == 1)//for previous error
		{
			wall_data();
			soft_right();
			velocity(80,70);
			err = 4;                 //initializing another error
		}
		else if(LWS<trsh && MWS<trsh && RWS<trsh && err == 2)//for previous error
		{
			wall_data();
			soft_left();
			velocity(70,80);
			err = 5;                  //initializing another error
		}
		else if((LWS>trsh && MWS>trsh) || (MWS>trsh && RWS>trsh) || (LWS>trsh && MWS>trsh && RWS>trsh))
		{
			count++;
			stop();
			forwardd();
			velocity(90,90);
			_delay_ms(450);
			stop();
			_delay_ms(2000);
		}
		else if(LWS<trsh && MWS<trsh && RWS<trsh && err==3)//if another error occured without being reposition 
		{                                                  //in the right place then to change the movement of the bot 
			wall_data();                               //towards opposite direction to bring it back to stable position.
			forwardd();
			velocity(80,77);
		}
		else if(LWS<trsh && MWS<trsh && RWS<trsh && err==4)//another error
		{
			wall_data();
			soft_left();
			velocity(70,80);
		}
		else if(LWS<trsh && MWS<trsh && RWS<trsh && err==5)//another error
		{
			wall_data();
			soft_right();
			velocity(80,70);
		}
		else if(LWS>trsh&&MWS<trsh&&RWS>trsh)
		{
			whiteline(node);
		}
	}
}

void leftside(void)
{
	while(1)
	{
		readsensors();
		if(LWS>trsh)
		{
			stop();
			break;
		}
		else
		{
			leftt();
			velocity(75,75);
		}
	}
}

void rightside(void)
{
	while(1)
	{
		readsensors();
		if(RWS>trsh)
		{
			stop();
			break;
		}
		else
		{
			rightt();
			velocity(75,75);
		}
	}
}
//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 0.5650; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 0.302; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	stop(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM)
{
	forwardd();
	linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
	back();
	linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	leftt(); //Turn left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}



void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	rightt();  //Turn right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
	
}


void soft_left_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left(); //Turn soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right();  //Turn soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_left_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left_2(); //Turn reverse soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right_2();  //Turn reverse soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}
void wall_data()
{
sharp1 = ADC_Conversion(9);
sharp2 = ADC_Conversion(10);
value2=	Sharp_GP2D12_estimation(sharp2);				//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
value1 = Sharp_GP2D12_estimation(sharp1);				//Stores Distance calsulated in a variable "value".
while((value1<250)&&(value2<250))
{
	sharp1 = ADC_Conversion(9);
	sharp2 = ADC_Conversion(10);
	value2=	Sharp_GP2D12_estimation(sharp2);				//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
	value1 = Sharp_GP2D12_estimation(sharp1);				//Stores Distance calsulated in a variable "value".
	lcd_print(1,10,value2,3); 						//Prints Value Of Distanc in MM measured by Sharp Sensor.
	lcd_print(1,6,value1,3);
	if(value1>163 && value2>163)
	{
		forwardd();
		velocity(90,87);
	}
	else if(value1<163 && value2>163)
	{
		soft_left(); //for slight change in the direction of the movement of the bot
		velocity(75,87);
	}
	else if(value1>163 && value2<163)
	{
		soft_right();  //for slight change in the direction of the movement of the bot
		velocity(90,75);
	}
	else
	{
		forwardd();
		velocity(90,87);
	}
}
}
void whiteline(unsigned int node1)
{
	unsigned int count = 0;
	while(count<node1)
	{
		readsensors();
		if(LWS>trsh && MWS<trsh && RWS>trsh)//forward
		{
			forwardd();
			velocity(80,77);
			err = 0;
		}
		else if(LWS<trsh&&MWS>trsh&&RWS<trsh)
		{
			return;
		}
		else if(LWS>trsh && MWS>trsh && RWS<trsh)//right
		{
			forwardd();
			velocity(80,65);
			err = 1;
		}
		else if(LWS<trsh && MWS>trsh && RWS>trsh)//left
		{
			forwardd();
			velocity(65,80);
			err = 2;
		}
		else if(LWS>trsh && MWS>trsh && RWS>trsh && err == 0)
		{
			forwardd();
			velocity(80,77);
			err = 3;
		}
		else if(LWS>trsh && MWS>trsh && RWS>trsh && err == 1)
		{
			forwardd();
			velocity(65,80);
			err = 4;
		}
		else if(LWS>trsh && MWS>trsh && RWS>trsh && err == 2)
		{
			forwardd();
			velocity(80,65);
			err = 5;
		}
		else if((LWS<trsh && MWS<trsh) || (MWS<trsh && RWS<trsh) || (LWS<trsh && RWS<trsh) || (LWS<trsh && MWS<trsh && RWS<trsh))
		{
			count++;
			stop();
			_delay_ms(2000);
			pick();
		}
		else if(LWS>trsh && MWS>trsh && RWS>trsh && err==3)
		{
			forwardd();
			velocity(80,77);
		}
		else if(LWS>trsh && MWS>trsh && RWS>trsh && err==4)
		{
			forwardd();
			velocity(80,65);
		}else if(LWS>trsh && MWS>trsh && RWS>trsh && err==5)
		{
			forwardd();
			velocity(65,80);
		}
	}
}
void pick()
{
velocity(150,150);
stop();
_delay_ms(500);
back_mm(60);

for(int i =175;i>2;i--)
{
	servo_1(i);
	_delay_ms(15);
}
forward_mm(90);
_delay_ms(2000);
for(int i =5;i<=180;i++)
{
	servo_2(i);
	_delay_ms(10);
}
_delay_ms(1000);
for(int i =1;i<=170;i++)
{
	servo_1(i);
	_delay_ms(15);
}
}
void place()
{
	for(int i =175;i>60;i--)
	{
		servo_1(i);
		_delay_ms(15);
	}
	forward_mm(90);
	_delay_ms(2000);
	for(int i =180;i<=100;i--)
	{
		servo_2(i);
		_delay_ms(10);
	}
	_delay_ms(1000);
	for(int i =1;i<=170;i++)
	{
		servo_1(i);
		_delay_ms(15);
	}
}
void servo_1(unsigned char degrees)
{
	float PositionPanServo = 0;
	PositionPanServo = ((float)degrees / 1.86) + 35.0;
	OCR1AH = 0x00;
	OCR1AL = (unsigned char) PositionPanServo;
}


//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
	float PositionTiltServo = 0;
	PositionTiltServo = ((float)degrees / 1.86) + 35.0;
	OCR1BH = 0x00;
	OCR1BL = (unsigned char) PositionTiltServo;
}

//servo_free functions unlocks the servo motors from the any angle
//and make them free by giving 100% duty cycle at the PWM. This function can be used to
//reduce the power consumption of the motor if it is holding load against the gravity.

void servo_1_free (void) //makes servo 1 free rotating
{
	OCR1AH = 0x03;
	OCR1AL = 0xFF; //Servo 1 off
}

void servo_2_free (void) //makes servo 2 free rotating
{
	OCR1BH = 0x03;
	OCR1BL = 0xFF; //Servo 2 off
}

void init_devices (void)
{
	cli(); //Clears the global interrupts
	port_init();
	timer5_init();
	timer1_init();
	adc_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei(); //Enables the global interrupts
}

//path regarding/tasking functions help to decide house . it will decide ,where we have to place the box  
void house_decision(int G[16][16],int house)//in house variable we are sending the no. of house
{
         if(house ==1) //condition for house one
           {
                dijkstra(G,16,current_node,5);
                current_node=5;//changing the currrent status
                if(up==1)
                  {
                    leftside();
                    left =1;
                    up =0;    //for changing the previous diection ,adding 0  to previous direction and  1 to new direction
                  }
                if(down == 1)
                  {
                     rightside();//rotating along rightside
                     down =0;
                     left=1;
                  }

            }
          else if(house ==2)//condition for house 2
            {
                    dijkstra(G,16,current_node,6);
                    current_node=6;
                    if(up==1)
                     {
                          up=0;
                          right=1;//for changing the direction
                          rightside();
                     }
                     if(down ==1)
                       {
                          down=0;
                          right =1;
                          leftside();
                       }
             }
            else if(house ==3)//condition for house 3
             {
                    dijkstra(G,16,current_node,9);
                    current_node=9;
                    if(up==1)
                    {
                      leftside();//rotate along left side
                      left =1;
                      up =0;
                    }
                    if(down == 1)
                    {
                       rightside();
                       down =0;
                       left=1;
                    }
              }
            else if(house ==4)//condition for house 4
              {
                    dijkstra(G,16,current_node,10);
                    current_node=10;//changing the current node position 
                    if(up==1)
                      {
                          up=0;
                          right=1;
                          rightside();//rightside function to rotate along right side

                      }
                    if(down ==1)
                      {
                          down=0;
                          right =1;
                          leftside();//leftside fuction to rotate along left side
                      }
                }
            else//condition for house 5 (taking default becauuse no any house is left more)
                 {
                     dijkstra(G,16,current_node,14);//calling this function we are getting correct path
                     current_node=14;
                     if(left ==1)
                     {
                         left =0;
                         leftside();
                         down =1;
                     }
                     if(right ==1)
                     {
                         right = 0;
                         rightside();
                         down=1;//changing the direction 
                     }
                  }
                //upper all the conditions just send us to specific position where we have to perform placing
                 place();//place function to perform placing 

}


//defining the function arena to get direction of end from the start
int arena(int start,int end) //
{
    int i,j;
    //specifiy the special cases in arena based on specials conections 1.wall connection and 2. Zig-Zag connection
    if(start ==5 && end == 6)
        return  2;
    if(start == 6 && end ==5)
        return 1;
    if(start == 9 && end == 10)
        return 2;
    if(start == 10 && end ==9)
        return 1;
    int arr[7][3]=
    {

        {
            13,14,15 //14 is an intermidatery node between 13 and 15
        },
        {
            12,-1,11
        },
        {
            9,-1,10 //if there is no any intermidate node betwe 2 nodes then we add it as -1
        },
        {
            8,-1,7
        },
        {
            5,-1,6
        },
        {

            4,-1,3
        },
        {

            0,1,2
        }
    };
for(i =0;i<7;i++)
  for( j=0;j<3;j++)
  {

    if(arr[i][j]==start) //first we find node with whom we have to make connection
    {

           if(arr[i-1][j]==end && i-1!=-1)
             return 3;//3 for up direction
           else if(arr[i+1][j]==end && i+1 !=7)
             return 4; //4 for downward direction 
           else if(arr[i][j-1]==end && j-1 !=-1)
             return 1;//1 for left
           else
             return 2;//2 for right
    }
  }

}

//Dijkstra algorithm for giving the correct path between the given two nodes startnode and endnode
void dijkstra(int G[16][16],int n,int startnode,int endnode)
{
    int val,co=0,y;   //this is whole dijkstra algorithm for finding the minimum distance between the given nodes
    int arr[16]={214};
    int cost[16][16],distance[16],pred[16];
	int visited[16],count,mindistance,nextnode,i,j;
    for(i=0;i<n;i++)
    
		for(j=0;j<n;j++)   
			if(G[i][j]==0)
				cost[i][j]=INFINITY;
			else
				cost[i][j]=G[i][j];
    for(i=0;i<n;i++)
	 {
		distance[i]=cost[startnode][i];
		pred[i]=startnode;
		visited[i]=0;
	 }
    distance[startnode]=0;//used to find distance
	visited[startnode]=1;//for defining wheathe we have visited to the node otr not
	count=1;
    while(count<n-1)
	{
		mindistance=INFINITY;//defining maximum distacne first then we will set min distance 


		for(i=0;i<n;i++)
			if(distance[i]<mindistance&&!visited[i])
			{
				mindistance=distance[i];
				nextnode=i;
			}


			visited[nextnode]=1;
			for(i=0;i<n;i++)
				if(!visited[i])
					if(mindistance+cost[nextnode][i]<distance[i])
					{
						distance[i]=mindistance+cost[nextnode][i];
						pred[i]=nextnode;
					}
		count++;
	 }

        i= endnode;
		if(i!=startnode)
		{

			arr[co]=i;
			co++;
			j=i;
			do
			{
				j=pred[j];
                arr[co]=j;
                co++;

			}
			while(j!=startnode);
	    }
        for( y=co;y<16;y++)
            arr[y]=-1;
	    for( i=15;i>0;i--)
        {
        if(arr[i]!=-1)
        {
        val = arena(arr[i],arr[i-1]);
    //condition 1
    if(val==1)
        {

          if(right ==1)
          {
        	left= 1;
        	right =0;
        	leftside();
        	leftside();
            forward();
          }
          else if(up ==1)
          {
        	left =1;
        	up =0;
        	leftside();
            forward();
          }
		  else if (down==1)
		  {
			left =1;
			down =0;
			rightside();
            forward();
          }
		  else
		  forward();

       }

      //condition 2
    else if(val==2)
    {

            if(right ==1)
          	forward();
            else if(up ==1)
            {
              up=0;
        	  right =1;
        	  rightside();
              forward();
		    }
		    else if (down==1)
		    {
			  down=0;
			  right = 1;
			  leftside();
              forward();
		    }
            else
		    {
		        left=0;
		     	right =1;
		      	rightside();
			    rightside();
                forward();
            }
        }

   //condition 3
    else if(val==3)
    {

            if(left ==1)
            {
              left =0;
        	  up =1;
        	  rightside();
              forward();
            }
		    else if (right ==1)
		    {
		        right =0;
		     	up =1;
		      	leftside();
                forward();
            }
		    else if (down ==1)
            {
                down =0;
		     	up =1;
			    leftside();
			    leftside();
                forward();
		    }
		    else
		     	forward();
    }

    //condition 4
    else
    {

        if(up==1)
          {
            up=0;
        	down =1;
        	leftside();
        	leftside();
            forward();
		  }
		else  if (left ==1)
		  {
		    left =0;
			down =1;
			leftside();
            forward();
		  }

		else if (right ==1)
		  {
		    right =0;
			down =1;
			rightside();
            forward();
		  }
		else
		  	forward();
    }
         }
      }
}

void eyantra()//function used to define conection between the nodes if connection is present then it will 1 otherwise it will 0
{
    int i,house,rem;
    int G[16][16]={
	    {

	        0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0//
	    },
	    {

	        1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0
	    },
	    {

	        0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0
	    },
	    {

	        0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0
	    },
	    {

	        1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0
	    },
	    {

	        0,0,0,0,1,0,2,0,1,0,0,0,0,0,0,0
	    },
	    {

	        0,0,0,1,0,2,0,1,0,0,0,0,0,0,0,0
	    },
	    {

	        0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0
	    },
	    {

	        0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0
	    },
	    {

	        0,0,0,0,0,0,0,0,1,0,3,0,1,0,0,0
	    },
	    {

	        0,0,0,0,0,0,0,1,0,3,0,1,0,0,0,0
	    },
	    {

	        0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1
	    },{

	        0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0
	    },{

	        0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0
	    },{

	        0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1
	    },{

	        0,0,0,0,0,0,0,0,0,0,0,1,0,0,1,0
	    }
	};

    for(i=9;i>=0;i--)
    {
        //first zero check condition
        if(which_material[i]==0)
            continue;
        // first condition
        if(which_material[i]==1 || which_material[i]==2)
        {
            dijkstra(G,16,current_node,4);
            current_node = 4;
            if(which_material[i]==1)
            {
               if(up==1)
               {
                    leftside();
                    left =1;
                    up= 0;
               }
               if(down == 1)
               {
                 rightside();
                 left =1;
                 down =0;

               }
                 pick();
                 house = (i/2) +1;
                 //placing condition
                 house_decision(G,house);
             }
             if(which_material[i]==2)
             {

               if(down==1)
               {
                    leftside();
                    right =1;
                    down= 0;
               }
               if(up == 1)
               {
                 rightside();
                 right =1;
                 up =0;

               }
                pick();
                house = (i/2) +1;
                 //placing condition
                house_decision(G,house);

             }
        }

        //second condition

        if(which_material[i]==3 || which_material[i]==4)
        {
            dijkstra(G,16,current_node,3);
            current_node = 3;

             if(which_material[i]==3)
             {
               if(up==1)
               {
                    leftside();
                    left =1;
                    up= 0;
               }
               if(down == 1)
               {
                 rightside();
                 left =1;
                 down =0;
               }
                 pick();
                 house = (i/2) +1;
                 //placing condition
                 house_decision(G,house);
             }
             if(which_material[i]==4)
             {

               if(down==1)
               {
                    leftside();
                    right =1;
                    down= 0;
               }
               if(up == 1)
               {
                 rightside();
                 right =1;
                 up =0;

               }
                pick();
                house = (i/2) +1;
                //placing condition
                house_decision(G,house);

              }
         }

    //third condition
        if(which_material[i]==5 || which_material[i]==6)
        {
            dijkstra(G,16,current_node,8);
            current_node = 8;
             if(which_material[i]==5)
             {
               if(up==1)
               {
                    leftside();
                    left =1;
                    up= 0;
               }
               if(down == 1)
               {
                 rightside();
                 left =1;
                 down =0;

               }
                 pick();
                 house = (i/2)+1;
                 //placing condition
                 house_decision(G,house);
             }
             if(which_material[i]==6)
             {
               if(down==1)
               {
                    leftside();
                    right =1;
                    down= 0;
               }
               if(up == 1)
               {
                 rightside();
                 right =1;
                 up =0;
               }
               pick();
               house = (i/2) +1;
               //placing condition
               house_decision(G,house);
            }
        }

        //fourth condition

        if(which_material[i]==7 || which_material[i]==8)
        {
            dijkstra(G,16,current_node,7);
            current_node = 7;
            if(which_material[i]==7)
            {
               if(up==1)
               {
                    leftside();
                    left =1;
                    up= 0;
               }
               if(down == 1)
               {
                 rightside();
                 left =1;
                 down =0;

               }
                 pick();
                 house =(i/2) +1;
                 //placing condition
                 house_decision(G,house);
             }
             if(which_material[i]==8)
             {

               if(down==1)
               {
                    leftside();
                    right =1;
                    down= 0;
               }
               if(up == 1)
               {
                 rightside();
                 right =1;
                 up =0;

               }
                pick();
                 house = (i/2) +1;
                 //placing condition
                 house_decision(G,house);
            }
         }
        //fifth condition
       if(which_material[i]==9 || which_material[i]==10)
          {
             dijkstra(G,16,current_node,12);
             current_node = 12;
             if(which_material[i]==9)
             {
               if(up==1)
               {
                    leftside();
                    left =1;
                    up= 0;
               }
               if(down == 1)
               {
                 rightside();
                 left =1;
                 down =0;
               }
                 pick();
                 house = (i/2) +1;
                 //placing condition
                 house_decision(G,house);
             }
               if(which_material[i]==10)
               {

               if(down==1)
               {
                    leftside();
                    right =1;
                    down= 0;
               }
               if(up == 1)
               {
                 rightside();
                 right =1;
                 up =0;

               }
                pick();
                house = (i/2) +1;
                //placing condition
                house_decision(G,house);
                }
        }

        //sixth condition
        if(which_material[i]==11 || which_material[i]==12)
        {
             dijkstra(G,16,current_node,11);
            current_node = 11;
             if(which_material[i]==11)
             {
               if(up==1)
               {
                    leftside();
                    left =1;
                    up= 0;
               }
               if(down == 1)
               {
                 rightside();
                 left =1;
                 down =0;

               }
                 pick();
                 house =(i/2)+1;
                 //placing condition
                 house_decision(G,house);
             }
             if(which_material[i]==12)
             {

               if(down==1)
               {
                    leftside();
                    right =1;
                    down= 0;
               }
               if(up == 1)
               {
                 rightside();
                 right =1;
                 up =0;

               }
                pick();
                house = (i/2) +1;
                //placing condition
                house_decision(G,house);

             }
        }
    }
}
//Main Function
int main(void)
{
	init_devices();
	
	lcd_set_4bit();
	lcd_init();
 	set_direction();//function to define predefined directions 
    eyantra();//function for defining whole condtion to traverse all the arena
    return 0;
}