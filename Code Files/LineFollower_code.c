#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h> //included to support power function
#include "lcd.h"
#define trsh 50
int err = 0;
unsigned char ADC_Conversion(unsigned char);
unsigned char LWS,RWS,MWS;
unsigned char sharp1,sharp2, distance, adc_reading;
unsigned int value1,value2,pickd;
volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle in degrees for turning

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


void forward (void) //both wheels forward
{
	motion_set(0x06);
}

void back (void) //both wheels backward
{
	motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
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
void forward_wls(unsigned int node)
{
	unsigned int count = 0;
	while(count<node)
	{
		readsensors();
		if(LWS<trsh && MWS>trsh && RWS<trsh)//forward
		{
			forward();
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
			forward();
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
			forward();
			velocity(90,90);
			_delay_ms(450);
			stop();
			_delay_ms(2000);
		}
		else if(LWS<trsh && MWS<trsh && RWS<trsh && err==3)//if another error occured without being reposition 
		{                                                  //in the right place then to change the movement of the bot 
			wall_data();                               //towards opposite direction to bring it back to stable position.
			forward();
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

void left_turn_wls(void)
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
			left();
			velocity(75,75);
		}
	}
}

void right_turn_wls(void)
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
			right();
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
	forward();
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
	left(); //Turn left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}



void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	right();  //Turn right
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
		forward();
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
		forward();
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
			forward();
			velocity(80,77);
			err = 0;
		}
		else if(LWS<trsh&&MWS>trsh&&RWS<trsh)
		{
			return;
		}
		else if(LWS>trsh && MWS>trsh && RWS<trsh)//right
		{
			forward();
			velocity(80,65);
			err = 1;
		}
		else if(LWS<trsh && MWS>trsh && RWS>trsh)//left
		{
			forward();
			velocity(65,80);
			err = 2;
		}
		else if(LWS>trsh && MWS>trsh && RWS>trsh && err == 0)
		{
			forward();
			velocity(80,77);
			err = 3;
		}
		else if(LWS>trsh && MWS>trsh && RWS>trsh && err == 1)
		{
			forward();
			velocity(65,80);
			err = 4;
		}
		else if(LWS>trsh && MWS>trsh && RWS>trsh && err == 2)
		{
			forward();
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
			forward();
			velocity(80,77);
		}
		else if(LWS>trsh && MWS>trsh && RWS>trsh && err==4)
		{
			forward();
			velocity(80,65);
		}else if(LWS>trsh && MWS>trsh && RWS>trsh && err==5)
		{
			forward();
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


//Main Function
int main(void)
{
	init_devices();
	
	lcd_set_4bit();
	lcd_init();
 	
}
