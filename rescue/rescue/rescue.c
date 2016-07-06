/*******************************************************************************
*Team ID:          eYRCPlus-SR#545
*
*Author List :     Bhavesh Jadav, Bhalchandra Naik
*
*Filename:         rescue.c
*
*Theme:            Search and Rescue - eYRC-Plus specific
*
*Functions:        void addSurvivorInfo(),
*				   void releaseOldSearchPath(),
*				   void updateNewSearchPath(),
*				   ISR(USART0_RX_vect),
*				   int isValidNode(int position),
*				   void turn_left(unsigned int deg),
*				   void turn_right(unsigned int deg),
*				   int isPlotValid(int plot),
*				   void init_cinfo_matrix(),
*				   void change_location(),
*				   void getAdjacentPlotPoints(int plot),
*				   int computeMinIndex(int d[],int v[]),
*				   struct pathStack findPath(int s, int d),
*				   struct pathStack selectMinPathtoDestinationPlot(),
*				   void follow(),
*				   void scanForBlackDebris(),
*				   void north(unsigned int current_orientation),
*				   void south(unsigned int current_orientation),
*				   void west(unsigned int current_orientation),
*				   void east(int current_orientation),
*				   void travel(int from, int to),
*				   int RescueIsAllowedtoGo(int node_pos),
*				   void goToNearestFreeNode(),
*				   void travelPath(struct pathStack paths),
*				   int nodeIsInSearchPath(int position),
*				   struct pathStack findPathToNearestFreeNode(),
*				   struct plot fetchSurvivorPlot(),
*				   void serveRed(int plot_pos),
*				   void serveGreen(int plot_pos),
*				   void performMSR(struct plot des),
*				   int main(void)
*		   
*Global Variables:   int point_north=0;
*					int point_east=1;
*					int point_south=2;
*					int point_west=3;
*					int current_pos=0;
*					volatile unsigned int blackDebrisCounter = 0;
*					int searchNodetobeExcluded = 0;
*					volatile unsigned int resetPath = 0;
*					volatile unsigned int searchComplete = 0;
*					int pltadjpoints[4] = {0};
*					volatile unsigned int vertexWithBlackDebris[40] = {0};
*					volatile unsigned int cinfo[100][5] = {0};
*					volatile int searchPathLength = 0;
*					volatile int searchPath[50] = {0};
*					volatile int survivorData[2] = {0};
*					volatile int searchCurrentPosition = 0;
*					int R = 8;
*					int G = 9;
*					struct pathStack
*					{
*						int path[100];
*						int TOP;	
*					};
*
*					struct survivorInfo
*					{
*						int plot[10];
*						int count;
*					};
*
*					struct plot
*					{
*						int pos;
*						int color;
*					};
*
*					struct survivorInfo redInfo;
*					struct survivorInfo greenInfo;
*
*					volatile unsigned long int ShaftCountLeft = 0;		
*					volatile unsigned long int ShaftCountRight = 0;		
*					volatile unsigned int Degrees;						
*					unsigned char ADC_Conversion(unsigned char);		
*					unsigned char ADC_Value;							
*					unsigned char Left_white_line = 0;					
*					unsigned char Center_white_line = 0;
*					unsigned char Right_white_line = 0;
*					int orientation=0;									
*																		 
*					volatile unsigned long int pulse = 0; 
*					volatile unsigned long int red;       
*					volatile unsigned long int green;     
*					int base_pos = 90;
*					int deposition_pos = 0;
*
*					int searchDataIndex = 0;
*					volatile int searchData[50] = {0};
*					volatile int survivorDataReceived = 0;
*					volatile int allowedtoService;
*
*********************************************************************************/
#define F_CPU 14745600

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

/********************************
   User Defined Header files 
*********************************/
#include "lcd.h"
#include "led.h"
#include "Servo.h"

void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();
struct pathStack findPathToNearestFreeNode();

int point_north=0;    //a constant variable that denotes the north direction                             
int point_east=1;     //a constant variable that denotes the east direction
int point_south=2;    //a constant variable that denotes the south direction
int point_west=3;     //a constant variable that denotes the west direction
int current_pos=0;    //current_pos: this variable holds the current position of the bot in the arena i.e. which node it has encountered most recently or upon which it stands. Range: [11,99]
volatile unsigned int blackDebrisCounter = 0;
int searchNodetobeExcluded = 0;
volatile unsigned int resetPath = 0;
volatile unsigned int searchComplete = 0;
int pltadjpoints[4] = {0};
volatile unsigned int vertexWithBlackDebris[40] = {0};
volatile unsigned int cinfo[100][5] = {0};
volatile int searchPathLength = 0;
volatile int searchPath[50] = {0};
volatile int survivorData[2] = {0};
volatile int searchCurrentPosition = 0;
int R = 8;
int G = 9;
volatile unsigned long int ShaftCountLeft = 0;		//to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0;		//to keep track of right position encoder
volatile unsigned int Degrees;						//to accept angle in degrees for turning
unsigned char ADC_Conversion(unsigned char);		//function declaration for analog to digital conversion
unsigned char ADC_Value;							//variable for storing ADC value
unsigned char Left_white_line = 0;					//variables for storing line sensors data
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
int orientation=0;									//variable to keep track of orientation of bot (1 = vertical , 0 = horizontal)
                                                    //bot will only scan plot on the left side of it when orientation is equal to 1
volatile unsigned long int pulse = 0;               //to keep the track of the number of pulses generated by the color sensor
volatile unsigned long int red;                     // variable to store the pulse count when read_red function is called
volatile unsigned long int green;                   // variable to store the pulse count when read_green function is called
int base_pos = 90;
int deposition_pos = 0;

int searchDataIndex = 0;
volatile int searchData[50] = {0};
volatile int survivorDataReceived = 0;
volatile int allowedtoService;

struct pathStack    // this data structure shall contain the path( an ordered sequence of co-ordinates) that the bot must follow.
{
	int path[100];  //the sequence of the co-ordinates is stored in this array
	int TOP;	    //this points to the topmost element in the array and is also the length of the path. This is any whole number between 1-100.
};

struct survivorInfo //This data structure contains the information of the survivors to be serviced 
{
	int plot[10];   //contains the co-ordinates of plots where the survivors are trapped. After the survivor is serviced the element is removed 
	int count;      //contains the number of survivors that are to be serviced. It is usually a whole number that varies between 0-16
};

struct plot         //This data structure contains the information of a particular survivor
{
	int pos;        //It contains the location of the bot in the arena 
	int color;      //It contains the color of the survivor
};

struct survivorInfo redInfo;     //contains the information of the red survivors waiting to be serviced 
struct survivorInfo greenInfo;   //contains the information of the green survivors waiting to be serviced



//Function to configure ports to enable robot's motion
void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
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

 //Interrupt 4 enable
void left_position_encoder_interrupt_init (void)
{
	cli();						//Clears the global interrupt
	EICRB = EICRB | 0x02;		// INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10;		// Enable Interrupt INT4 for left position encoder
	sei();						// Enables the global interrupt
}

 //Interrupt 5 enable
void right_position_encoder_interrupt_init (void)
{
	cli();						//Clears the global interrupt
	EICRB = EICRB | 0x08;		// INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20;		// Enable Interrupt INT5 for right position encoder
	sei();						// Enables the global interrupt
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

ISR(INT0_vect)
{
	pulse++; //increment on receiving pulse from the color sensor
}


/*
*function name: addSurvivorInfo
*
*Input:         NO INPUT
*
*Output:        adds the information of the survivor received from the search bot to the either of the structures redInfo or greenInfo
*
*Logic:         The information received from the search bot could either be that of green or red
*               depending on the data stored int the survivorData[] array which is written into when
*				xBee interrupt is called, the plot position of the survivor is stored in the respective
*				data structure. The data in the 0th position determines which color the survivor is of 
*				and the 1th position holds the co-ordinate of the survivor.
*				
*Example call:  addSurvivorInfo();
*
*/
void addSurvivorInfo()
{
	if(survivorData[0] == R)                          //survivorData[] array contains the color of survivor at 0th position. Here the color is red so data will be added in redInfo
	{                                                 
		for(int i = 0; i < 10; i++)                   //counter i will iterate from 0 to 10 which is maximum size of the array
		{
			if(redInfo.plot[i] == 0)                  //if an empty location within the array is found (i.e. holding 0)the plot co-ordinate is stored there
			{
				redInfo.plot[i] = survivorData[1];    //survivor[1] contains location of survivor
				redInfo.count++;                      // number of survivors waiting to be serviced is incremented
				break;
			}				
		}
	}
	else if(survivorData[0] == G)                     //survivorData[] array contains the color of survivor at 0th position. Here the color is green so data will be added in greenInfo
	{
		for(int i = 0; i < 10; i++)                   //counter i will iterate from 0 to 10 which is maximum size of the array
		{
			if(greenInfo.plot[i] == 0)                //if an empty location within the array is found the plot(i.e. holding 0) co-ordinate is stored there
			{
				greenInfo.plot[i] = survivorData[1];  //survivor[1] contains location of survivor
				greenInfo.count++;                    // number of survivors waiting to be serviced is incremented
				break;
			}
		}
	}
}


/*
*function name: releaseOldSearchPath
*
*Input:         NO INPUT
*
*Output:        old search path stored in the searchPath array is released
*
*Logic:         Since the Search has change its path the old path needs to be cleared. Here the loop 
                will iterate from 0 to maximum size of array which is 50
*Example call:  releaseOldSearchPath();
*
*/
void releaseOldSearchPath()
{
	for (int i = 0; i < 50; i++)
		searchPath[i] = 0; //search path is cleared
}

/*
*function name: updateNewSearchPath
*
*Input:         NO INPUT
*
*Output:        new path of the search bot which is stored in searchData array is copied into searchPath
*
*Logic:         Since the Search has change its path the new path needs to be stored, which has been written into searchData array. Here the loop 
*               will iterate from 0 to the size of the path, which is stored in searchPathLength
*Example call:  updateNewSearchPath();
*
*/
void updateNewSearchPath()
{
	for (int i = 0; i < searchPathLength; i++) //loop iterates from 0 to length of the search robot's Path
		searchPath[i] = searchData[i]; //each individual node in the path is copied
}

ISR(USART0_RX_vect) 		// ISR for receive complete interrupt
{
	char data = UDR0;
	_delay_ms(5);
	
	searchData[searchDataIndex] = data;
	
	if(searchData[searchDataIndex] == 7)
	{
		searchComplete = 1;
	    cinfo[searchCurrentPosition][4] = 1;
	}		
	else if(searchData[searchDataIndex] == 5)
	{
		allowedtoService = 1;
		searchDataIndex = -1;
	}
	
	else if(searchData[searchDataIndex] == 4)
	{
		searchDataIndex = -1;
		survivorData[0] = searchData[0];
		survivorData[1] = searchData[1];
		survivorDataReceived = 1;
	}
	
	else if(searchData[searchDataIndex] == 2)
	{
		for (int i = 0; i < searchPathLength; i++)
		{
			if(searchData[0] == searchPath[i])
			{
				searchPath[i] = 0;
				break;
			}
		}
		searchCurrentPosition = searchData[1];
		searchDataIndex = -1;
	}
	
	else if(searchData[searchDataIndex] == 3)
	{
		cinfo[searchData[0]][4] = 1;
		vertexWithBlackDebris[blackDebrisCounter] = searchData[0];
		blackDebrisCounter++;
		searchDataIndex = -1;
	}
	
	else if(searchData[searchDataIndex] == 1)
	{
		releaseOldSearchPath();
		searchPathLength = searchDataIndex;
		updateNewSearchPath();
		searchDataIndex = -1;
	}
	
	searchDataIndex++;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}

//both wheels forward
void forward (void) 
{
	motion_set(0x06);
}

//both wheels backward
void back (void) 
{
	motion_set(0x09);
}

//Left wheel backward, Right wheel forward
void left (void) 
{
	motion_set(0x05);
}

//Left wheel forward, Right wheel backward
void right (void) 
{
	motion_set(0x0A);
}

//stops the both wheels
void stop (void)
{
	motion_set(0x00);
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

//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;
	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
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

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	//stop(); //Stop robot
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
	angle_rotate(Degrees);
}

void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	right(); //Turn right
	angle_rotate(Degrees);
}

//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
	UCSR0B = 0x00; //disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	//UBRR0L = 0x47; //11059200 Hz
	UBRR0L = 0x5F; // 14745600 Hzset baud rate lo
	UBRR0H = 0x00; //set baud rate hi
	UCSR0B = 0x98;
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;			//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

//Function to Initialize PORTS
void port_init()
{
	motion_pin_config();			//robot motion pins config
	left_encoder_pin_config();		//left encoder pin config
	right_encoder_pin_config();	    //right encoder pin config
	buzzer_pin_config();			//buzzer pin config
	lcd_port_config();				//lcd pin config
	adc_pin_config();				//adc pin config
	motion_pin_config();			//motor driver pin config
	base_servo_pin_config();
	deposition_servo_pin_config();
	led_port_config();
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

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

//Function To Print Sensor Values At Desired Row And Coloumn Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{
	
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Initialize the devices
void init_devices (void)
{
 	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer5_init();
	timer1_init();
	uart0_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei();   //Enables the global interrupts
}


void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}


/*
*
* Function Name: isValidNode
* Input: 		 node_coordinate -> the coordinate that we have to check if it is in arena or not the plot coordinate
* Output: 		 returns 1 if the node_coordinate is inside the arena and not the plot coordinate or else returns 0
* Logic: 		 1) Extract x and y from coordinate that is passed as parameter
*				 2) If both x and y are even then it is plot coordinate then return 0
*				 3) If it is less than 11 and greater than 99 or it is multiple of 10 than it is out of the arena, return 0
*				 4) If both the above conditions are false then node_coordinate is valid and return 1
* Example Call:	 isValidNode(85);
*
*/
int isValidNode(int node_coordinate)
{
	int x, y;
	x = node_coordinate % 10; //extract x coordinate
	y = node_coordinate / 10; //extract y coordinate
	
	//If both are event means its a plot coordinate then return 0
	if(x % 2 == 0 && y % 2 == 0)
	return 0;
	
	//if it is less then 11 or greater then 99 or its a multiple of 10 then passed coordinate is not part of arena then return 0
	if((node_coordinate % 10 == 0) || (node_coordinate < 11 || node_coordinate > 99))
	return 0;
	
	//if above both the conditions are false then passed coordinate is valid so return 1
	return 1;
}

/*
*
* Function Name:  turn_left
* Input: 		  deg -> degree at which robot wants to rotate left
* Output: 		  void
* Logic: 		  1) The robot will turn left by the angle passed as the parameter
*				  2) After turning it will check if center white line censor is on black line or not
*				  3) If the center white line is not present on the black line then rotate robot left until center white line
*				     comes on the black line
* Example Call:	  turn_left();
*
*/
void turn_left(unsigned int deg)
{
	velocity(255, 255);
	left_degrees(deg);  //turn left by degree specified
	Center_white_line = ADC_Conversion(2);  //take readings of white line sensor
	if(Center_white_line < 70)  //check if center white line sensor is on black line or not
	{
		velocity(150,150);
		left();//if center white line is not on black line then turn left
		Center_white_line = ADC_Conversion(2);//take readings of white line sensor
		while (Center_white_line < 70)  //keep turning left while center white line does not come on black line
			Center_white_line = ADC_Conversion(2);//while turning keep checking the white line sensor
	}
	_delay_ms(60);//wait for some time
	stop();//stop after turning left
}

/*
*
* Function Name:  turn_right
* Input: 		  deg -> degree at which robot wants to rotate right
* Output: 		  void
* Logic: 		  1) The robot will turn right by the angle passed as the parameter
*				  2) After turning it will check if center white line censor is on black line or not
*				  3) If the center white line is not present on the black line then rotate robot right until center white line
*				     comes on the black line
* Example Call:	  turn_right();
*
*/
void turn_right(unsigned int deg)
{
	velocity(252, 255);
	right_degrees(deg);//turn right by degree specified
	Center_white_line = ADC_Conversion(2);//take readings of white line sensor
	if(Center_white_line < 70)//check if center white line sensor is on black line or not
	{
		velocity(150,150);
		right();//if center white line is not on black line then turn right
		Center_white_line = ADC_Conversion(2);//take readings of white line sensor
		while (Center_white_line < 70)//keep turning right while center white line does not come on black line
			Center_white_line = ADC_Conversion(2);//while turning keep checking the white line sensor
	}
	_delay_ms(60); //wait for some time
	stop(); //stop after turning right
}


/*
*
* Function Name: isPlotValid
* Input: 		 plot_coordinate -> the plot coordinate that we have to check if it is in arena or not
* Output: 		 returns 1 if the plot_coordinate is valid plot coordinate or else returns 0
* Logic: 		 1) Extract x and y from coordinate that is passed as parameter
*				 2) If it is less than 11 and greater than 99 or it is multiple of 10 than it is out of the arena, return 0
*				 3) If above condition are false then plot_coordinate is valid and return 1
* Example Call:	 isPlotValid(22);
*
*/
int isPlotValid(int plot_coordinate)
{
	//if it is less then 11 or greater then 99 or its a multiple of 10 then passed coordinate is not part of arena then return 0
	if(plot_coordinate % 10 == 0 || plot_coordinate < 11 || plot_coordinate > 99)
		return 0;
	else
		return 1;
}


/*
*
* Function Name:  init_cinfo_matrix
* Input: 		  void
* Output: 		  void
* Logic: 		  This function will use to initialize cinfo matrix. An entry to this table has been described below:
*                 an given below is an entry at 55th row for co-ordinate 55
*				  at j= 0  1  2  3  4
*				       |65|56|45|54|0|
*			      here 65 is the node north to 55(at index point_north = 0)
*				  here 56 is the node east to 55(at index point_east = 1)
*			      here 45 is the node south to 55(at index point_south = 2)
*			      here 54 is the node west to 55(at index point_north = 3)
*			      and 0  at the 4th position indicates that node is NOT blocked by a black debris. Had it been 1 then it would have been blocked.
*                 The nodes to the any 2-digit coordinate ' yx ' north, east, south or west are computed using the following equations:
*				  1) node to north of ' yx 'shall be  = (y+1)*10+x
*				  2) node to south of ' yx 'shall be  = (y-1)*10+x
*				  3) node to east of ' yx 'shall be  = y*10+(x+1)
*				  4) node to west of ' yx ' shall be = y*10+(x-1)
*				  5) the entry at the 4th position shall be 0 for all vertices i.e initially all co-ordinates shall be UNBLOCKED
*
* Example Call:	  init_cinfo_matrix();
*
*/
void init_cinfo_matrix()
{
	int i, j, temp;
	for(i = 11; i <= 99; i++)
	{
		if(isValidNode(i))
		{
			//for j value  0 = north, 1 = east, 2 = south, 3 = west, 4 = black debris present or not
			j = 0;
			temp = i + 10;   //temp shall hold the node to the north which shall be the N+10 
			if(isValidNode(temp))  //checks if the node to north is valid or not
				cinfo[i][j] = temp;  
			else
				cinfo[i][j] = 0; //if node is invalid then it shall be zero

			temp = i + 1;
			if(isValidNode(temp))  //temp shall hold the node to the east which shall be the N+1 
				cinfo[i][j+1] = temp; //checks if the node to east is valid or not
			else
				cinfo[i][j+1] = 0;  //if node is invalid then it shall be zero

			temp = i - 10;
			if(isValidNode(temp))  //temp shall hold the node to the south which shall be the N-10 
				cinfo[i][j+2] = temp;  //checks if the node to south is valid or not
			else
				cinfo[i][j+2] = 0;  //if node is invalid then it shall be zero

			temp = i - 1;
			if(isValidNode(temp))  //temp shall hold the node to the west which shall be the N-1 
				cinfo[i][j+3] = temp; //checks if the node to west is valid or not
			else
				cinfo[i][j+3] = 0;  //if node is invalid then it shall be zero
			
			cinfo[i][j+4] = 0; //initially all the coordinates are set as unblocked by BLACK DERBIS
		}
	}
}



/*
* Function Name: change_location
*
* Input: NONE
*
* Output: changes the current position of the bot
*
* Logic: When node is sensed by the white line sensors in the follow function, this functions is called 
*        to change the location depending on the orientation of the bot which stored in a global variable
*
*        if the current_pos of the bot can be written as a 2-digit integer 'yx' where y and x are non-zero single digit numbers then: 
*        1)if the orientation is north the the node shall be (y+1)*10+x
*		 2)if the orientation is south the the node shall be (y-1)*10+x
*		 3)if the orientation is east the the node shall be y*10+(x+1)
*		 4)if the orientation is west the the node shall be y*10+(x-1)
*
* Example Call: change_location();
*
*/
void change_location(){
	int x=current_pos%10;  //x bit is obtained 
	int y=current_pos/10;  //y-bit is obtained
	if (orientation==point_north)//if orientation is north 
	{
		y++; //co-ordinate changed in positive y direction
		current_pos=y*10+x;
	}
	else if (orientation==point_west)//if orientation is west
	{
		x--; //co-ordinate changed in negative x direction
		current_pos=y*10+x;
	}
	else if (orientation==point_east)//if orientation is east
	{
		x++; //co-ordinate changed in positive x direction
		current_pos=y*10+x;
	}
	else if(orientation==point_south){ //if orientation is south 
		y--;  //co-ordinate changed in negative y direction
		current_pos=y*10+x;
	}
}

/*
*
* Function Name:  getAdjacentPlotPoints
* Input: 		  plot_coordinate -> the plot of which all available adjacent midpoint is to be calculated
* Output: 		  void
* Logic: 		  1) do plus and minus 10 in case of north and south and do plus and minus 1 in case of east and west and store them in pltadjpoints array
*					 this will indicate all the available plot adjacent points
*				  2) if black debris is present on any of the adjacent point then it will set 0 on its position in array
* Example Call:	  getAdjacentPlotPoints(46);
*
*/
void getAdjacentPlotPoints(int plot)
{
	int i,j;
	pltadjpoints[0] = plot+10;  //calculate adjacent point present on north side of the plot
	pltadjpoints[1] = plot+1;   //calculate adjacent point present on east side of the plot
	pltadjpoints[2] = plot-10;  //calculate adjacent point present on south side of the plot
	pltadjpoints[3] = plot-1;   //calculate adjacent point present on west side of the plot
	//iterate over all calculated adjacent point to check if black debris is present on them or not
	for(i=0;i<4;i++)
	{
		j=0;
		//go through vertexWithBlackDebris array and compare them with the selected adjacent point
		while(vertexWithBlackDebris[j]!=0)
		{
			//if any of the black debris coordinate present in array is equal to adjacent point then it is blocked
			if(vertexWithBlackDebris[j] == pltadjpoints[i])
			pltadjpoints[i] = 0;//set 0 in place of blocked adjacent point
			j++;//increment j to goto next vertexWithBlackDebris position
		}
	}
}


/*
* Function Name: computeMinIndex
*
* Input: d[] -> integer array which stores the minimum distance of any ith node with whose co-ordinate is also i from the source node
*        v[] -> integer array which stores if any ith node is visited while the graph is being traversed. Holds 1 at 'i'th location if node 'i' is  visited.
*
*
* Output: returns the index(integer) of the node with least distance from the source and which is also NOT visited
*
* Logic:  the function iterates over the distance array d[] to find the minimum positive value which is the closest to 
*         the source and has not been visited(i.e the visited array shall hold 1 at the 'i'th position if node i
*         is visited). The algorithm is similar to the classic minimum value computing algorithm(where a variable 
*         'min' is set to infinity or a big value and is reset every time the array element being pointed to is 
*		  less than that), only with an additional condition and we are here returning the index and not the value. 
* 
* 
* Example Call: computeMinIndex(int d[],int v[]);
*
*/
int computeMinIndex(int d[],int v[]){
	int i;
	int min=999; // variable is set to a large value(infinity)
	int min_i=0; // it is the index of the element with least value
	for(i=0;i<100;i++)     
	{
		if(min>d[i]&&v[i]==0)   //if the min is greater that the distance array element being pointed to and is unvisited(i.e. v[i]==0) then...
		{
			min=d[i];           //min is assigned the above condition is true
			min_i=i;            // the index of that node which also happens to be it co-ordinate
		}
	}
	return min_i;               
}

/*
* Function Name: findPath
*
* Input: s -> the source from where the shortest path is to be found 
*        d -> the destination to be reached 
*
*
* Output: returns a structure pathStack which contains the path.
*
* Logic:  This algorithm is similar to the Dijkstra's shortest path computing algorithm between two nodes a and b.
*          It picks the unvisited vertex with the lowest-distance, calculates the distance through it to each unvisited 
*		  neighbor, and updates the neighbor's distance if smaller. The unvisited vertex with lowest-distance is picked
*		  using the computeMinIndex function and the distance the neighboring coordinates of that selected node is updated 
*		  using the equation ' distance[cinfo[min_index][j]]=distance[min_index]+1 ' if the distance[cinfo[min_index][j]]
*		  is less than distance[min_index]+1. here the min_index shall become the parent of the of its adjacent coordinates 
*		  being updated. This information shall be stored using ' parent[cinfo[min_index][j]]=min_index; '. While computing the 
*		  pathStack variable, the path shall be read in reverse manner from the destination until the source is reached.  
*
* Example Call: findPath(int s, int d);
*
*/

struct pathStack findPath(int s, int d)
{
	int visited[100] = {0};  //all elements of the visited array is set to 0 i.e all are set as unvisited
	int distance[100];  //the distance stores the distance of all the coordinates from the source
	int parent[100];  //this array stores the parent of each node to be that must be reached to reach with minimum path
	int temp;  
	struct pathStack paths;  //this structure shall contain requested path to be followed. It shall be returned later
	if(s == d)  //if the source and destination are the same then a path containing the source and destination with length equal to one should be sent
	{
		paths.TOP = 1;  //length of path shall be 1
		paths.path[0] = d; //the destination is at the end 
		paths.path[1] = s; //the source is at the beginning
		return paths;
	}
	int i,j,k,min_index,top_temp=-1;;
	for(i=0;i<100;i++)
	{
		parent[i] =- 1; //the parent of all nodes is set to -1
		distance[i] = 999; //the distance of the nodes from the source 's' set to a large value(infinity)
	}
	visited[s] = 1; //parent is set as visited
	distance[s] = 0; //distance of source vertex from itself shall be 0
	parent[s] = s; //parent of the source node shall be itself
	k = 1;
	for(j=0;j<4;j++) //the loop iterates 4 times since in our graph(i.e arena) any node can have maximum of 4 neighbors
	{
		if(cinfo[s][j ] != 0) //it is checked whether the node is a valid vertex or not
		{
			distance[cinfo[s][j]] = 1; //distance of all the coordinates adjacent to the source is et to 1
			parent[cinfo[s][j]] =s ;//the parent of these coordinates is set to the source itself
		}
	}
	
	while(visited[d] != 1) //the while loop shall iterate until the destination is marked visited
	{
		min_index=computeMinIndex(distance,visited); //the UNVISITED node with the least distance is computed 
		visited[min_index]=1; //the node computed in the previous node is marked as visited
		for(j=0;j<4;j++)
		{
			if(cinfo[min_index][j]!=0) //validity of the node adjacent to min_index is checked
			{
				if( distance[cinfo[min_index][j]]>distance[min_index]+1   //this condition checks whether the distance of the node adjacent to the min_index, from the source, is less greater than 1 edge more than that of the min_index 
				                                    &&  cinfo[min_index][4]==0 )  //checks if the node is having a black debris(checks if its blocked) or not. IF zero then its UNBLOCKED 
				{
					distance[cinfo[min_index][j]]=distance[min_index]+1; //the shorter distance is saved 
					parent[cinfo[min_index][j]]=min_index; //parent of this node is set to the min_index
				}
			}
		}
		k++;//counter incremented
		if (k > 500) //if the iterations exceeds 500 then no path can be found hence a deadlock has risen 
		{
			lcd_wr_command(0x01);
			lcd_string("deadlockinf");
			paths.TOP = 0;
			return paths;
			break;
		}
	}
	
	//In the following lines the path from the specified source (s) to the destination (d) shall be computed
	paths.TOP=top_temp;  //the top of the stack is set to -1
	paths.path[++paths.TOP]=d;//the destination shall be at the bottom of the stack and the source at the top
	temp=parent[d]; //parent of the destination node is stored in temp
	while(temp!=s) //until the source is reached the path stack will keep building up
	{
		paths.path[++paths.TOP]=temp; //the parent node(which must be arrived to to get shortest distance) is saved. This shall precede its intended destination
		temp=parent[temp]; //parent of the other parent is then copied in a temporary variable
	}	
	paths.path[++paths.TOP]=s; //since the loop terminated at the arrival of the source. The source is saved at the top of the stack
	return paths; //the path is returned
}

/*
*
* Function Name:  selectMinPathtoDestinationPlot
* Input: 		  void
* Output: 		  minpath -> it contains the minimum path to destination plot
* Logic: 		  1) go over all available adjacent plot points stored in pltadjpoints
*				  2) if adjacent point is not zero then calculate shortest path between current position and selected adjacent point
*				  3) compare path of all adjacent points and compare them on the basis of path length and number of turns required
*                    turns will be only compared when 2 paths have same length
*				  4) finally return path which has minimum length and minimum turns
* Example Call:	  selectMinPathtoDestinationPlot();
*
*/
struct pathStack selectMinPathtoDestinationPlot()
{
	int minlength = 999;  //initialize minlength to infinity
	int turn1 = 0; //initialize turns for the path that is calculated
	int turn2 = 0; //initialize turns for path that is stored in minpath
	struct pathStack path, minpath;//declare variables to store minpath and calculated path
	for(int i = 0; i < 4; i++)  //iterate over all the 4 plot adjacent point stored in pltadjpoints array
	{
		if(pltadjpoints[i] != 0)  //if the adjacent point is not 0
		{
			path = findPath(current_pos, pltadjpoints[i]); // calculate the path between current position and selected adjacent point   
			
			if(path.TOP < minlength) //compare path length with minlength
			{
				minlength = path.TOP; //if calculated path length is less then minlength then assign path length to minlength
				minpath = path;  //assign calculated path to minpath
			}
			else if(path.TOP == minlength)  //if calculated path length and minlength both are same
			{
				//then calculate the numbers of turns available in both the paths
				for(int i = 1; i < minlength; i++)
				{
					int a1 = path.path[i-1];
					int a2 = path.path[i];
					int a3 = path.path[i+1];
					if(abs(a1-a2) != abs(a2-a3))
						turn1++;
					
					a1 = minpath.path[i-1];
					a2 = minpath.path[i];
					a3 = minpath.path[i+1];
					if(abs(a1-a2) != abs(a2-a3))
						turn2++;
				}
				if(turn1 < turn2)  //if the calculated path turn in less then turn2
					minpath = path;  //then assign calculated path minpath 
			}
		}
		turn1 = turn2 = 0;   //reset both path turn1 and turn2 to 0
		minlength = minpath.TOP; //assign minpathlength to minlength variable
	}
	return minpath; //finally return calculated minpath. 
}

/*
*
* Function Name:  follow
* Input: 		  void
* Output: 		  void
* Logic: 		  this function is used to follow line. It takes the readings from white line sensor and check for the detection of nodes
*				  or else it will follow the line
* Example Call:	  follow();
*
*/
void follow()
{
	lcd_wr_command(0x01);
	lcd_string("follow");
	while(1)
	{
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		
		//sum value will be greater than 150 if bot is on the one of the node of midpoint
		int sum = Left_white_line + Center_white_line + Right_white_line;
		
		//if sum value is greater than 140 or left or right white senor is on node or midpoint increase the counter
		//first bot will encounter the node, counter value increase to 1. Then bot will encounter midpoint, counter value increase to 2.
		//So whenever bot encounters the node counter value will be ODD and whenever bot will encounter the midpoint counter value will be EVEN 
		if(sum > 130)  //if sum value is greater than 130 then bot is on the node or midpoint
		{
			//below condition will be used at the start of the run. At start the current_pos of rescue robot will be initialized to 0
			//when it encounters the node then it will be set to 59 according to our coordinate system
			if(current_pos == 0)
			{
				current_pos = 59;
				orientation = point_west;
				forward_mm(55); //set orientation to west because at the start of the run bot will be facing west
				return;
			}
			change_location(current_pos); //change the current position after encountering the node
			UDR0 = (char)current_pos;   //send rescue robot position after changing it
			_delay_ms(5);   //wait for data to be sent
			UDR0 = 2;//send terminal packet as 2 indicating previous data sent must be rescue robots current position
			_delay_ms(5);   //wait for data to be sent
			stop();
			buzzer_on();
		    _delay_ms(50);
			buzzer_off();
			//extract x and y from current position
			int x = current_pos % 10;
			int y = current_pos / 10;
			velocity(251,255);
			scanForBlackDebris();//scan for black debris in fron of bot
			if(x % 2 != 0 && y % 2 != 0)//if the bot is at a node which is at a corner of the survivor plot then go forward by 70mm
				forward_mm(70);
			else
				forward_mm(15);//if at a mid-point node then go forward by 15mm
			break;
		}
        //if no node is encounter then simply follow the line
		if(Center_white_line > 10)
		{
			velocity(255,255);
			forward();
		}

		else if(Left_white_line > 10)
		{
			velocity(150,255);
			forward();
			/*stop();
			velocity(150,150);
			left();*/
		}

		else if(Right_white_line > 10)
		{
			velocity(255,150);
			forward();
			/*stop();
			velocity(150,150);
			right();*/
		}
		
		else if(Center_white_line <= 10 && Left_white_line <= 10 && Right_white_line <= 10)
		{
			velocity(255,255);
			forward();
		}
	}
}

/*
*
* Function Name:  scanForBlackDebris
* Input: 		  void
* Output: 		  void
* Logic: 		  This function will scan the midpoints for presence of black debris. If black debris is present then
*				  1) it will update cinfo matrix
*				  2) store black debris coordinate in vertexWithBlackDebris array if black debris if found
*				  3) send information about this black debris to rescue robot
* Example Call:	  scanForBlackDebris();
*
*/
void scanForBlackDebris()
{
	int blackDebrisPos;  //variable to store calculated black debris position
	int x, y;
	x = current_pos % 10; //extract x from current position
	y = current_pos / 10; //extract y from current position
	
	//if both x and y are odd means robot is standing on node then scan for black debris in front of it
	if (x % 2 != 0 && y %2 != 0)
	{
		//take reading from sharp IR sensor
		int sharp = ADC_Conversion(11);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
		int value = Sharp_GP2D12_estimation(sharp);
		lcd_print(2,1,value,3);
		
		if(value < 230 && value > 0)  //if value is less then 230 that means black debris is present
		{
			resetPath = 1;  //set resetPath flag to 1 indicating path needs to recalculated excluding found black debris
			print_sensor(2,1,11);
			
			//calculate position of black debris based on orientation of the robot
			if(orientation == point_north)
			blackDebrisPos = current_pos + 10;
			else if(orientation == point_south)
			blackDebrisPos = current_pos - 10;
			else if(orientation == point_east)
			blackDebrisPos = current_pos + 1;
			else if(orientation == point_west)
			blackDebrisPos = current_pos - 1;
			
			//There is a possibility that search robot is standing on midpoint and rescue robot is standing on node facing search robot
			//rescue robot will scan that mid point for black debris and the calculated value will be less then 230. In this case we don't have to consider
			//that mid point as black debris so before updating cinfo matrix, vertexWithBlackDebris sending it's position to search robot
			//check if calculated black debris position is equal to search robot position or not. If it is equal then do not consider it as a black debris
			if(blackDebrisPos != searchCurrentPosition)
			{
				//if calculated black debris position is not equal to rescue robots current position then before doing anything check if it is already scanned or not
				for (int i = 0; vertexWithBlackDebris[i] != 0; i++)
				{
					//if that position is already scanned then it will be present in vertexWithBlackDebris
					if(vertexWithBlackDebris[i] == blackDebrisPos)
					return;  //if it is present then return before updating it
				}
				
				//if it is not present then update vertexWithBlackDebris with calculated position
				vertexWithBlackDebris[blackDebrisCounter] = blackDebrisPos;
				cinfo[blackDebrisPos][4] = 1;  //update cinfo matrix
				
				UDR0 = (char)blackDebrisPos;  //send calculated black debris coordinate to rescue robot
				_delay_ms(5);  //wait for data to be sent
				UDR0 = 3;  //send end packet as 3 indicating previous data sent must be information about black debris position
				_delay_ms(5);  //wait for data to be sent
				blackDebrisCounter++;  //increment blackDebrisCounter for the next time use
			}
		}
	}
}


/*
 *
 * Function Name: north
 * Input: current orientation of the robot
 * Output: void
 * Logic: 1) If the bot faces in north and has to move north then it must go forward
 *  	  2) If the bot faces in north and has to move east then it must go right
 *		  3) If the bot faces in north and has to move west then it must go left		
 * Example Call: north(pointing_east);
 *
 */
void north(unsigned int current_orientation) 
{
	if (current_orientation == point_east) 
	{
        velocity(255,255);
		turn_left(60);
    } 
	else if (current_orientation == point_west) 
	{ 
        velocity(255,255);
		turn_right(60);
    }
	else if (current_orientation == point_south)
	{
		velocity(255,255);
		turn_left(160);
	}
	orientation = point_north;
}

/*
 *
 * Function Name: south
 * Input: current orientation of the robot
 * Output: void
 * Logic: 1) If the bot faces in south and has to move south then it must go forward
 *        2) If the bot faces in south and has to move east then it must go left
 *        3) If the bot faces in south and has to move west then it must go right		
 * Example Call: south(pointing_east);
 *
 */
void south(unsigned int current_orientation) 
{
    if (current_orientation == point_east) 
	{
        velocity(255,255);
		turn_right(60);
    } 
	else if (current_orientation == point_west) 
	{
        velocity(255,255);
		turn_left(60);
    }
	else if (current_orientation == point_north)
	{
		velocity(255,255);
		turn_right(160);
	}
    orientation = point_south;
}

/*
 *
 * Function Name: west
 * Input: current orientation of the robot
 * Output: void
 * Logic: 1) If the bot faces in west and has to move west then it must go forward
 *		  2) If the bot faces in west and has to move north then it must go right
 *		  3) If the bot faces in west and has to move south then it must go left		
 * Example Call: north(pointing_east);
 *
 */
void west(unsigned int current_orientation) 
{ 
    if (current_orientation == point_north) 
	{ 
        velocity(255,255);
	    turn_left(60);
    } 
	else if (current_orientation == point_south) 
	{ 
		velocity(255,255);
        turn_right(60);      
    }
	else if (current_orientation == point_east)
	{ 
		velocity(255,255);
		turn_right(160);
	}
    orientation = point_west;
}

/*
 *
 * Function Name: east()
 * Input: current orientation of the robot
 * Output: void
 * Logic: 1) If the bot faces in east and has to move east then it must go forward
 *        2) If the bot faces in east and has to move north then it must go left
 *		  3) If the bot faces in east and has to move south then it must go right		
 * Example Call: south(pointing_east);
 *
 */
void east(int current_orientation) 
{
    if (current_orientation == point_north) 
	{
       velocity(255,255);
	   turn_right(60);
    } 
	else if (current_orientation == point_south) 
	{ 
        velocity(255,255);
        turn_left(60);
    }
	else if (current_orientation == point_west)
	{
		velocity(255,255);
		turn_right(160);
	}
    orientation = point_east;
}

/*
 *
 * Function Name: travel(from , to)
 * Input: (from_co-ordinate , to_co-ordinate)
 * Output: void
 * Logic: Split from and to co-ordinate value into x and y co-ordinate.
 *		  travel node to node until from(x co-ordinate) = to (x co-ordinate)
 *		  travel node to node until from(y co-ordinate) = to (y co-ordinate)	 	
 * Example Call: travel(12,13);
 *					OR
 *				  travel(current_position,next_position)
 */
void travel(int from, int to) 
{
    int x1 = from % 10;
    int y1 = from / 10;
    int x2 = to % 10;
    int y2 = to / 10;
	lcd_print(2,5,from,2);
	lcd_print(2,8,to,2);
    if (from != to) 
	{
            if (y1 != y2) 
			{
                if (y1 > y2) // e.g travel(21,11) in this case one has to travel south
                    while (y1 != y2) 
					{
                        south(orientation);
                        y1--;
                    } 
				else if (y2 > y1)// e.g travel(11,21) in this case one has to travel north
                    while (y2 != y1) 
					{
                        north(orientation);
                        y1++;
                    }
            }
            if (x1 != x2) 
			{
                if (x1 > x2)// e.g travel(12,11) in this case one has to travel west
                    while (x1 != x2) 
					{
                        west(orientation);
                        x1--;

                    } 
				else if (x2 > x1)// e.g travel(11,12) in this case one has to travel east
                    while (x1 != x2) 
					{
                        east(orientation);
                        x1++;
					}
            }
        }
		else
			return;
		scanForBlackDebris();
		if(resetPath != 1)
			follow();
}

/*
* Function Name: RescueIsAllowedtoGo
*
* Input: node -> a valid node from the block 
*
* Output: returns either 1 (i.e. true) or 0(i.e. false)
*
* Logic:  the specified node is searched for in the global variable searchPath using a loop, if found then the function shall return true i.e 0 and if not then false i.e. 1
*
* Example Call: RescueIsAllowedtoGo(52);
*
*/
int RescueIsAllowedtoGo(int node_pos)
{
	for(int i = 0; i < searchPathLength; i++) // length of the search robot's path is stored in the global variable searchPathLength. the loop shall iterate those many times 
		if(node_pos == searchPath[i]) //checks if the content of the array at 'i'th location matches the node_pos. If yes then it is in the search robots path 
				return 0; //the function returns false as its not allowed
	
	return 1; //function returns true as the robot is allowed
}



/*
* Function Name: goToNearestFreeNode
*
* Input: NONE
*
* Output: The functions makes the robot go to the nearest location(co-ordinate within the graph) 
*         that does not lie in the search paths way and is not blocked by a black debris.		  
*
* Logic:  1) the function findPathfindPathToNearestFreeNode() is called which shall find the nearest 
             location within the arena which
			 a)does not lie in the search robots path stored in  searchPath(an array), a global variable.
			 b)is Not blocked by black debris i.e. cinfo[NODE][4]==0 (here NODE is an integer)
		  
		  2) The search robot shall follow this path until the destination is reached. A situation may 
		     arise that the rescue bot encounter the search Robot. In that case, the rescue bot shall mark that 
			 node as blocked temporarily and then recalculate the path and go to the computed location.   
			 
		  3) The rescue shall keep trying to travel to the free node until its current position(current_pos) becomes 
		     equals to the free_node 
*
* Example Call: RescueIsAllowedtoGo(52);
*
*/

void goToNearestFreeNode()
{
	int freenode;  //the free node computed shall be stored here 
	int i = 0; //counter initialized
	do
	{
		struct pathStack paths = findPathToNearestFreeNode();  //path to reach the freenode is calculated here. The free_node is calculated by the same
		cinfo[searchNodetobeExcluded][4] = 0; //set previously blocked search node to UNBLOCKED
		if(i == 0)  //at the start of the procedure of going to free node wait for 5 seconds until the search bot reaches the next node
			_delay_ms(5000);
		i++;  //counter increment
		freenode = paths.path[0];  //in the path stack the destination at 0th position which is be our free node
		for (int i = paths.TOP; i > 0; i--)  //loop iterates along the path
		{
			if(paths.path[i - 1] == searchCurrentPosition)  //if while traveling the next node is the current position of the search bot then the rescue bot must stop traveling to avoid collision
			{
				searchNodetobeExcluded = searchCurrentPosition; // the current position of the search bot must now be blocked in the arena
				cinfo[searchNodetobeExcluded][4] = 1; 
				break; //terminate traveling and recalculate a new path
			}
			travel(paths.path[i], paths.path[i - 1]);  //travel from current to next position
			if(resetPath == 1)  //if reset path flag is set then path needs to be calculated
			{
				resetPath = 0; //reset flag
				break; //break the loop
			}
		}
		
		//now the following code is used to check if the current rescue position is in search path or not 
		int j = 2;
		for(int i = 0; i < 50; i++) //iterate over search path and compare them with rescue current position
		{
			if(current_pos == searchPath[i]) //if true then increment the j 
				j++;
		}
		if(j == 2)  //if the current position is in search path then the value of j will be 2 and it means that rescue has arrived at some free node which is not in search path  
			break;  //so break the do while loop
	}
	while(current_pos!=freenode);  //loop shall iterate until the current position is equal to the free node
	stop();
}

/*
* Function Name: travelPath
*
* Input: paths -> path which needs to travelled
*
* Output: The functions makes the robot follow the specified path.		  
*
* Logic:  1) This function will make the bot travel from node to node by calling travel function 
*         2) Before going to the next node rescue robot will check that the next node is in search path or not
*             a) If the next node is in search path then rescue robot will check that its current position is in search path or not 
*			     if the current position is in search path then tell search robot to stop while rescue robot will go to nearest free node
*			  b) Else just stop the robot until the next node is clear  
*         3) After reaching the next node check if the new survivor data is arrived or not , if it has been arrived then add them to respected  structures 
*         4) and also check the reset path flag if it is one then the path needs to be recalculated so break the current travel path loop
*
* Example Call: travelPath(paths);
*
*/
void travelPath(struct pathStack paths)
{
	lcd_wr_command(0x01);
	lcd_string("travelPath");
	for(int i = paths.TOP; i > 0; i--)  //iterate over the path that has been passed to this function
	{
		if(nodeIsInSearchPath(paths.path[i - 1]))  //check if next node is in search path or not 
		{
			stop();  //if the next node is in search path then stop the robot 
			if(nodeIsInSearchPath(current_pos))   //after stopping the robot check current position is in search path or not
			{
				UDR0 = 7;  //tells search robot to stop by sending data packet 7
				_delay_ms(5);  //wait for data to be sent
				goToNearestFreeNode();  //go to the nearest free node so that search can travel from its path
				UDR0 = 8;  //after reaching nearest free node tell search robot to move by sending data packet 8
				_delay_ms(5);//wait for data to be sent
			}
			break;  //if next node is search path then break this travel path loop
		}
		
		travel(paths.path[i], paths.path[i-1]);  //this function will make the bot travel to the next node in path 
		
		if(survivorDataReceived == 1)  //check if survivorDtaReceived flag is set to 1 
		{
			//if it is set to 1 then new survivor data is received
			survivorDataReceived = 0;  
			addSurvivorInfo();  //  add them in respective structure 
		}
		
		if(resetPath == 1)  //if this flag has been set then the bot  must choose an alternative path to reach its destination hence the loop shall be broken here 
		{
			stop();
			resetPath = 0;  // flag is reset 
			break;
		}			
	}
}


/*
* Function Name: nodeIsInSearchPath
*
* Input: position -> a valid node from the block
*
* Output: returns either 1 (i.e. true) or 0(i.e. false)
*
* Logic:  the specified node is searched for in the global variable searchPath using a loop, if found then the function shall return true i.e 1 and if not then false i.e. 0
*
* Example Call: nodeIsInSearchPath(72);
*
*/
int nodeIsInSearchPath(int position){
   
   for(int i = 0; i < 50; i++)//loop iterates to the maximum size of the searchPath array 
   {
	   if(searchPath[i] == position){ //the content at the 'i'th location of the array is compared to specified position
		   return 1;    //if position exists in search's path he true i.e. 1 is returned
	   }
   }
   return 0; //if the position is not found anywhere in the loop then false. i.e. is returned
}

/*
* Function Name: findPathToNearestFreeNode
*
* Input: NONE
*
* Output: The functions finds a path to a location within the graph that is 
*		  a)NOT in the search Robots path
*         b)is NOT blocked by a black debris
*
* Logic:  The working of this algorithm is similar to that of the logic of the findPath(int s, int d) algorithm, with 
          a primary difference that here the destination is not known, as the nearest free node is to be computed. 
*         This algorithm is similar to the Dijkstra's shortest path computing algorithm between two nodes a and b.
*         It picks the unvisited vertex with the lowest-distance, calculates the distance through it to each unvisited
*		  neighbor, and updates the neighbor's distance if smaller than what it is. The unvisited vertex with lowest-distance 
*         is picked using the computeMinIndex function and the distance the neighboring coordinates of that selected node is updated
*		  using the equation ' distance[cinfo[min_index][j]]=distance[min_index]+1 ' if the distance[cinfo[min_index][j]]
*		  is less than distance[min_index]+1. Here the min_index shall become the parent of the of its adjacent coordinates
*		  being updated. This information shall be stored using ' parent[cinfo[min_index][j]]=min_index; '. 
*          
*		  What is most important to note is that the min_index produced at any instance shall be our intended 
*         nearest free node if it fulfills the following condition:
*
*         1)The min_index does not lie in the search Robots path.
*
*		  The moment the above condition is fulfilled the min_index thus produced shall be saved.the loop is broken and
*         the graph is not traversed any further.
*         
*		  
*		  While computing the pathStack variable, the parent array shall be read in reverse manner from the destination until the source is reached.
*
*
* Example Call: findPathToNearestFreeNode()
*
*/
struct pathStack findPathToNearestFreeNode()
{
	int visited[100] = {0};  //all elements of the visited array is set to 0 i.e all are set as unvisited
	int distance[100];  //the distance stores the distance of all the coordinates from the source
	int parent[100];  //this array stores the parent of each node to be that must be reached to reach with minimum path
	int temp;
	int s = current_pos;//the source shall be the current position of the robot
	int freenode=0; //the free is initialized to 0
	struct pathStack paths;
	int i,j,k,min_index;
	for(i=0;i<100;i++)
	{
		parent[i] =- 1; //the parent of all nodes is set to -1
		distance[i] = 999; //the distance of the nodes from the source 's' set to a large value(infinity)
	}
	visited[s] = 1; //parent is set as visited
	distance[s] = 0; //distance of source vertex from itself shall be 0
	parent[s] = s; //parent of the source node shall be itself
	k = 1;	
	
	for(j=0;j<4;j++) //the loop iterates 4 times since in our graph(i.e arena) any node can have maximum of 4 neighbors
	{
		if(cinfo[s][j ] != 0) //it is checked whether the node is a valid vertex or not
		{
			distance[cinfo[s][j]] = 1; //distance of all the coordinates adjacent to the source is et to 1
			parent[cinfo[s][j]] =s ;//the parent of these coordinates is set to the source itself
		}
	}
	
	while(1)
	{
		min_index=computeMinIndex(distance,visited); //the UNVISITED node with the least distance is computed
		visited[min_index]=1; //the node computed in the previous node is marked as visited
		for(j=0;j<4;j++)
		{
			if(cinfo[min_index][j]!=0) //validity of the node adjacent to min_index is checked
			{
				if( distance[cinfo[min_index][j]]>distance[min_index]+1   //this condition checks whether the distance of the node adjacent to the min_index, from the source, is less greater than 1 edge more than that of the min_index
				                &&  cinfo[min_index][4]==0 )  //checks if the node is having a black debris(checks if its blocked) or not. IF zero then its UNBLOCKED
				{
					distance[cinfo[min_index][j]]=distance[min_index]+1; //the shorter distance is saved
					parent[cinfo[min_index][j]]=min_index; //parent of this node is set to the min_index
				}
			}
		}
		if (RescueIsAllowedtoGo(min_index) //checks if the computed min_index lies in the search Robots path or not
		                       && cinfo[min_index][4]==0) //checks if he node is unblocked or not
		{
			freenode = min_index; //the free node is saved and the loop is broken
			break;
		}
		k++; //counter incremented
		if (k > 500) //if the iterations exceeds 500 then no path can be found hence a deadlock has risen 
		{
			lcd_wr_command(0x01);
			lcd_string("deadlockinf");
			paths.TOP = 0;
			return paths;
			break;
		}
	}
	//In the following lines the path from the specified free source(s) to the destination (freenode) shall be computed
	paths.TOP = -1; //the top of the stack is set to -1
	paths.path[++paths.TOP] = freenode; //the destination shall be at the bottom of the stack and the source at the top
	temp = parent[freenode]; //parent of the destination node is stored in temp
	while(temp!=s) //until the source is reached the path stack will keep building up
	{
		paths.path[++paths.TOP]=temp; //the parent node(which must be arrived to to get shortest distance) is saved. This shall precede its intended destination
		temp=parent[temp]; //parent of the other parent is then copied in a temporary variable
	}
	paths.path[++paths.TOP]=s; //since the loop terminated at the arrival of the source. The source is saved at the top of the stack
	return paths; //the path is returned
}

/*
 *
 * Function Name: fetchSurvivorPlot
 * Input: NO INPUT
 * Output: a plot structure that contains the co-ordinate of the survivor plot to be serviced and its color (i.e either red or green)
 * Logic: 1) Since red survivors are of highest priority then the count of redInfo is checked. 
 *         If not zero then the plot structure for the closest red survivor is computed and returned.
 *		  
 *   	  2) The greenInfo.count is checked. If there are green survivors to be serviced then plot information of the closest one is returned
 *		  
 * 		  3) If the contents of both the structures redInfo and greenInfo are empty then rescue robot waits, until the searchDataRecieved flag is set and the loop is broken.
 *		     While the rescue Robot waits two of the following situation might arise :
 *	     	 a)The rescue bot is waiting at a co-ordinate that lies in the search robots path. If yes then the rescue bot shall go to the nearest 
 *  		   location that does not lie in the search bots path and is unblocked 
 *			 b)The searchComplete flag has been set, which means that the search bot has completed its routine. If yes then the loop will break which shall lead 
 *			   to the end of the rescue robot's task.
 * Example Call: fetchSurvivorPlot();
 *
 */
struct plot fetchSurvivorPlot() 
{
	lcd_wr_command(0x01);
	lcd_string("fetchSurvivorPlot");
	struct plot desPlot;   // this shall contain the information of the survivor plot to be serviced (i.e. color and co-ordinate)
	int i = 0, minlength = 999;  
	struct pathStack paths; //path for the rescue bot shall be stored here 
	if(redInfo.count != 0)  //since red survivors are of higher priority the red survivor info structure is checked for the same (0 indicates no red survivors)
	{
		while(i < 10)//counter shall iterate to max size of redInfo
		{
			if(redInfo.plot[i] != 0)  //validity of the content of 'i'th position in the redInfo are checked
			{  
				getAdjacentPlotPoints(redInfo.plot[i]); //The nodes adjacent to the 'i'th survivor plot are stored in global variable pltadjpoints[]
				paths = selectMinPathtoDestinationPlot(); //selects the shortest path to reach the 'i'th survivor
				int length = paths.TOP; //length of that path is obtained
				if(length < minlength) //in the following logic the nearest red survivor is found by finding path with minimum length
				{ 
					desPlot.pos = redInfo.plot[i]; //if less then the plot position is saved
					desPlot.color = R;  //color is saved i.e RED
					minlength = length; 
				}					
			}
			i++; //counter increment
		}
	}
	else if(greenInfo.count != 0) //the greenInfo structure is checked for any green survivors.(0 indicates no green survivors)
	{
		while(i < 10)//counter shall iterate to max size of greenInfo
		{
			if(greenInfo.plot[i] != 0)  //validity of the content of 'i'th position in the greenInfo are checked
			{
				getAdjacentPlotPoints(greenInfo.plot[i]);  //The nodes adjacent to the 'i'th survivor plot are stored in global variable pltadjpoints[]
				paths = selectMinPathtoDestinationPlot();  //selects the shortest path to reach the 'i'th survivor
				int length = paths.TOP; //length of that path is obtained
				if(length < minlength) //in the following logic the nearest green survivor is found by finding path with minimum length
				{
					desPlot.pos = greenInfo.plot[i];  //if less then the plot position is saved
					desPlot.color = G; //color is saved i.e RED
					minlength = length;
				}
			}
			i++;   //counter increment
		}
	}
	else //the execution arrives in this section if the rescue robot is currently having no information of survivors 
	{
		allowedtoService = 0;  //rescue is prohibited from servicing
		while(survivorDataReceived != 1)  //the rescue must be trapped in this loop indefinitely until any information is received from the search robot or the searchComplete flag is set
		{
			lcd_print(2,12,cinfo[16][4],1);
			if(searchComplete == 1) //since both structures (redInfo and greenInfo) are empty and the searchComplete flag is set, the the bot must finish its execution by first breaking from this loop 
				break;
			lcd_print(2,1,redInfo.count,2);
			lcd_print(2,4,greenInfo.count,2);
			if (nodeIsInSearchPath(current_pos))//as the rescue is stationery when its in this section of the code. It must keep checking if its lying in the search robot's path
			{
				UDR0 = 7; //if the rescue bot is in search bot's path, then it commands the search bot to halt by sending '7' to it and wait until the rescue reaches a place where search bot wont cross is path
				_delay_ms(5);
				lcd_wr_command(0x01);
				lcd_string("freenode");
				goToNearestFreeNode();//the rescue bot goes to a free location in the graph
				UDR0 = 8; //sending '8' to the search bot signals it to continue its routine as the rescue has moved to a location that does not lie in its path
				_delay_ms(5);
				_delay_ms(100);	
			}
		}
		survivorDataReceived = 0; //survivorDataReceived flag reset to 0
		addSurvivorInfo();  //the newly received information is added to the greenInfo or redInfo structures
		desPlot.pos = 0;  //the destination is set to 0
		desPlot.color = 0; //the color is set to zero just so the main logic may recall this function to get new information
	}
	return desPlot;
}

/*
 *
 * Function Name: serveRed
 * Input:  plot_pos -> the coordinates of the plot in the arena that has the survivor
 * Output: The rescue bot shall travel to the specified survivor plot location and deliver it back the medical camp
 * Logic: Here the function will make the rescue bot to perform the Routine for the red survivors which is as follows:
		  1)The Best node available amongst the 4 nodes adjacent to the plot shall be selected and the path for it shall be
		    computed using the function selectMinPathtoDestinationPlot().
		  2)The rescue bot shall keep computing new paths to reach the survivor and travel until the destination is reached.
		  3)Once the destination is reached the bot pretends to pick up the survivor and travels back to the medical camp which is 
		    to the east of coordinate 59
 * Example Call: serveRed(44);
 *
 */

void serveRed(int plot_pos)
{
	stop();
	lcd_wr_command(0x01);
	lcd_string("serveRed");
	getAdjacentPlotPoints(plot_pos); //the legitimate 4 available nodes from where the survivor can be serviced are stored 
	struct pathStack path = selectMinPathtoDestinationPlot(); //path to one of the 4 nodes(which shall give minimum turns and length)is calculated
	cinfo[searchNodetobeExcluded][4] = 0; //previously blocked node must be unblocked
	while(path.TOP == 0)  //If the path to the red survivor is blocked by search bot's path then, a path with top as 0 is returned
		path = selectMinPathtoDestinationPlot(); //the bot shall keep computing path to reach the survivor until a path with a non-zero top i.e. length is returned
	travelPath(path); //the bot follows the chosen path
	while(current_pos != path.path[0]) //the bot shall keep computing path and traveling until the destination is reached
	{
		getAdjacentPlotPoints(plot_pos); 
		path = selectMinPathtoDestinationPlot();
		cinfo[searchNodetobeExcluded][4] = 0;
		travelPath(path);
	}
	//the difference between a co-ordinate and the one to its north is 10 
	//the difference between a co-ordinate and the one to its south is -10
	//the difference between a co-ordinate and the one to its east is 1
	//the difference between a co-ordinate and the one to its west is -1
	int diff = current_pos-plot_pos;
	
	if (diff==1&&orientation==point_north||orientation==point_south&&diff==-1||diff==10&&orientation==point_west||orientation==point_east&&diff==-10)//The rescue robot must turn left either of the following is true
	{
		left_degrees(90);
		orientation = (orientation + 3) % 4; //orientation change equation for anti-clockwise rotation eg. if bot is turning left from west then, the new orientation shall be (3+3)%4 = 2 which is equal to point_south
		buzzer_on();
		_delay_ms(1000); //sound buzzer for 1 second to indicate the red survivor has been picked up
		buzzer_off();
	}
	if (diff==-1&&orientation==point_north||orientation==point_south&&diff==1||diff==-10&&orientation==point_west||orientation==point_east&&diff==10)//The rescue robot must turn left either of the following is true
	{
		right_degrees(90);
		orientation = (orientation + 1) % 4;  //orientation change equation for anti-clockwise rotation eg. if bot is turning right  from west then, the new orientation shall be (3+1)%4 = 0 which is equal to point_north
		buzzer_on();
		_delay_ms(1000); //sound buzzer for 1 second to indicate the red survivor has been picked up
		buzzer_off();
	}
	
	//The recue bot must now proceed to the mediccal camp which is east of node 59(see arena sample at the ttop of the code)
	while(current_pos != 59) //the bot stays in the loop until 59 bis reached
	{
		path = findPath(current_pos, 59); //path till the node 59 is found
		cinfo[searchNodetobeExcluded][4] = 0; //previously blocked node of the search robots's position in the arena must be unblocked 
		lcd_print(2,14,path.TOP,2);
		while(path.TOP == 0) //the bot shall stay in this loop until a valid path is returned which shall have a non-zero top i.e. non-zero length
		{
			stop();
			lcd_wr_command(0x01);
			lcd_string("deadlock");
			//_delay_ms(5000);
			path = findPath(current_pos, 59);  //path till the node 59 is found
			cinfo[searchNodetobeExcluded][4] = 0;  //previously blocked node of the search robots's position in the arena must be unblocked
		}
		travelPath(path); //The bot travels the path computed to reach the medical camp
	}
	stop();  //once the node 59 is reached the bot must stop momentarily
	if(orientation == point_north)  //if at the node 59 the rescue bot is facing north then it must turn right to face the medical camp
		turn_right(60);
	else if(orientation == point_south)  //if at the node 59 the rescue bot is facing south then it must turn left to face the medical camp
		turn_left(60);
	velocity(255,255);
	forward_mm(200);
	current_pos = 0; //current position of the search_bot shall be 0 which indicates it is at the medical camp 
	UDR0 = 0;
	_delay_ms(5);  
	UDR0 = 2;
	_delay_ms(5);
	stop();
	turn_off_led();
	buzzer_on();
	_delay_ms(1000); //sound buzzer for 1 second to indicate that the survivor has been delivered to medical camp 
	buzzer_off();
	turn_left(160); //the bot takes a U-turn to face the arena which is to its west
	orientation = point_west; //orientation after turning is set to west
}


/*
 *
 * Function Name: serveGreen
 * Input:  plot_pos -> the coordinates of the plot in the arena that has the green survivor
 * Output: The rescue bot shall travel to the specified survivor plot location and drop a first-aid kit
 * Logic: Here the function will make the rescue bot to perform the Routine for the green survivors which is as follows:
 *		  1)The Best node available amongst the 4 adjacent to the plot shall be selected and the path for it shall be
 *	        computed using the function selectMinPathtoDestinationPlot().
 *	      2)The rescue bot shall keep computing new paths to reach the survivor and travel until the destination is reached.
 *	      3)Once the destination is reached the bot drops a first-aid kit 
 * Example Call: serveGreen(44);
 *
 */
void serveGreen(int plot_pos)
{
	stop();
	lcd_wr_command(0x01);
	lcd_string("serveGreen");
	getAdjacentPlotPoints(plot_pos);  //the legitimate 4 available nodes from where the survivor can be serviced are stored 
	struct pathStack path = selectMinPathtoDestinationPlot();  //path to one of the 4 nodes(which shall give minimum turns and length)is calculated
	cinfo[searchNodetobeExcluded][4] = 0;  //previously blocked node must be unblocked
	while(path.TOP == 0)  //If the path to the red survivor is blocked by search bot's path then, a path with top as 0 is returned
		path = selectMinPathtoDestinationPlot();  //the bot shall keep computing path to reach the survivor until a path with a non-zero top i.e. length is returned
	travelPath(path);  //the bot follows the chosen path
	while(current_pos != path.path[0])//the bot shall keep computing path and traveling until the destination is reached
	{
		getAdjacentPlotPoints(plot_pos);
	    path = selectMinPathtoDestinationPlot();
		cinfo[searchNodetobeExcluded][4] = 0;
		travelPath(path);
	}
	
	stop();
	//the difference between a co-ordinate and the one to its north is 10
	//the difference between a co-ordinate and the one to its south is -10
	//the difference between a co-ordinate and the one to its east is 1
	//the difference between a co-ordinate and the one to its west is -1
	int diff = current_pos-plot_pos;
	
	//Please note that the default orientation of kit-dropping mechanism is the the same as that of the bot which is towards the front. Hence the base servo shall by default be at 90 degrees
	if (diff==1&&orientation==point_north||orientation==point_south&&diff==-1||diff==10&&orientation==point_west||orientation==point_east&&diff==-10) //The rescue robot's kit-dropping mechanism must turn left if either of the conditions  is true
	{
		base_rotate(0);
	}
	if (diff==-1&&orientation==point_north||orientation==point_south&&diff==1||diff==-10&&orientation==point_west||orientation==point_east&&diff==10) //The rescue robot's kit-dropping mechanism must turn right if either of the conditions  is true
	{
		base_rotate(180);
	}
	
	deposition_rotate(180);
	_delay_ms(100);
	deposition_rotate(0);
	base_rotate(90); //after the first-aid kit is dropped the base servo must turn to 90 degrees again  
	turn_off_led();
	buzzer_on();
	_delay_ms(1000); //sound buzzer for 1 second
	buzzer_off();
	servo_base_free();
	servo_deposition_free();
}


/*
 *
 * Function Name: performMSR
 * Input:  struct plot des ->the structure containing the color of the survivor and its location
 * Output: VOID
 * Logic: The rescue bot shall wait until the allowedtoService flag is set to true
          Depending on the contents of the argument passed the following is done:
          1)if the color of the survivor is red i.e. des.color == R then serveRed is called 
		  2)if the color of the survivor is green i.e. des.color == G then serveGreen is called 
 * Example Call: serveGreen(44);
 *
 */
void performMSR(struct plot des)
{
	lcd_wr_command(0x01);
	lcd_string("performMSR");
	while(allowedtoService != 1);  // function waits until allowedtoService flag is set
	if(current_pos == 0) //if the rescue bot is at the medical camp then it must move forward until it senses a node i.e 59
		follow();
	
	if(des.color == R) //if survivor is red then serveRed
		serveRed(des.pos); //the coordinate of the location of the red survivor is passed 
	else if(des.color == G) //if survivor is red then serveRed
		serveGreen(des.pos); //the coordinate of the location of the green survivor is passed 
}


/*
 *
 * Function Name: main
 * Input:  void
 * Output: The rescue performs its intended task i.e. rescue all survivors and finish its task 
 * Logic: 1) The redInfo and greenInfo structure are initialized to be empty, as no-information has been received
          2) The Rescue bot shall call the fetchSurvivorPlot() function to return a survivor plot structure.
		     This survivor info shall be stored in the struct plot destination structure
		  3) If the plot returned is valid i.e destination.pos != 0 then the following is done:
		     a)If color of the survivor is red then glow the LED as red
			 b)If color of the survivor is green then glow the LED as green
			 c)the performMSR function is called to carry out the rescuie routine
			 d)after the MSR is complete the information of the survivor is removed from its corresponding structure
		  4) If the plot info received is invalid i.e destination.pos == 0 then the execution will continue. 
		     The return of an invalid plot indicates that the rescue bot does not have the information of any 
			 survivors and is waiting for information from the search bot.
			 The Rescue shall check if the searchComplete flag has been set. If yes then this would logically mean 
			 that the all survivors have been found and the contents of the survivor structure are empty, hence the 
			 rescue bot has completed its task.
 * Example Call: serveGreen(44);
 *
 */
int main(void)
{
	for(int i = 0; i < 10; i++)  //survivor structures are initialized to empty
		redInfo.plot[i] = greenInfo.plot[i] = 0;
	redInfo.count = greenInfo.count = 0;
	
	/*allowedtoService = 1;
	redInfo.plot[0] = 46;
	redInfo.plot[1] = 44;
	redInfo.plot[2] = 82;
	redInfo.plot[3] = 88;
	redInfo.count = 4;*/
	
	init_devices();
	lcd_set_4bit();
	lcd_init();
	init_cinfo_matrix();
	base_servo(90);
	_delay_ms(500);
	deposition_servo(0);
	_delay_ms(500);
	while(1)
	{
		lcd_wr_command(0x01);
		int tempPosition = current_pos;  
		if(tempPosition == 0)  //if the position of the rescue bot is 0 i.e at medical camp then it's reset to 59 since the distance to plot must be computed from the 59
			current_pos = 59;
		struct plot destination = fetchSurvivorPlot(); //fetch the information of the survivor to be rescued which is closest and fulfills priority criteria(i.e that of red and green survivors)
		current_pos = tempPosition; //restore the current position to what it was
		if(destination.pos != 0) //if the survivor info is Valid i.e plot location is equal to some valid plot co-ordinate eg.22,24,64 etc
		{
			if (destination.color == R)//glow red LED if survivor is red
				red_led();
			else if(destination.color == G)//glow green LEd if survivor is green 
				green_led();
			performMSR(destination); //perform the appropriate MSR
			
			//after the routine is complete then remove the survivor info from its corresponding structure
			if(destination.color == R) 
			{
				for (int i = 0; i < 10; i++) //loop iterates from 0 to max size of the survivors structure i.e 10
				{
					if(redInfo.plot[i] == destination.pos) //if the destination plot coordinate is found then it is removed
					{
						redInfo.plot[i] = 0;
						redInfo.count--; //count of redInfo is decremented as one red survivor is rescued
						break;
					}
				}
			}
			else if(destination.color == G)
			{
				for (int i = 0; i < 10; i++)//loop iterates from 0 to max size of the survivors structure i.e 10
				{
					if(greenInfo.plot[i] == destination.pos) //if the destination plot coordinate is found then it is removed
					{
						greenInfo.plot[i] = 0;
						greenInfo.count--; //count of greenInfo is decremented as one green survivor is rescued
						break;
					}
				}
			}
		}			
		else
		{
			if(searchComplete == 1) //If the searchComplete flag has been set and if the survivor structures are empty(the execution is in this 
			{                       //section only if the survivor structures are empty), Then there are no more survivors left to be rescued
				stop();
				buzzer_on();
				_delay_ms(5000); //sound a buzzer for 5 seconds indicating end of routine and the task
				buzzer_off();
				break;     //Break the loop. The rescue bot can only be free from this loop when contents of its survivor structure are 
				           //empty and the search has completed its task by setting RESCUE's searchComplete flag, meaning that no more 
						   //survivors to be rescued are left
			}
		}
		
		/*red_led();
		_delay_ms(100);
		green_led();
		_delay_ms(100);
		blue_led();
		_delay_ms(100);*/
	}
	
	while(1);
}