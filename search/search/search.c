/*******************************************************************************
*Team ID:          eYRCPlus-SR#545
*
*Author List :     Bhavesh Jadav, Bhalchandra Naik
*
*Filename:         search.c
*
*Theme:            Search and Rescue - eYRC-Plus specific
*
*Functions:        ISR(USART0_RX_vect),
*				   int isValidNode(int position),
*				   void turn_left(unsigned int deg),
*				   void turn_right(unsigned int deg),
*				   int isPlotValid(int plot),
*				   void init_cinfo_matrix(),
*				   void change_location(),
*                  int findUnscannedPlot()
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
*				   void travelPath(struct pathStack paths),				   
*                   void scanPlots(int plot1, int plot2)
*				   void scanForWhiteDebris()
*				   void check_for_survivor()
*				   void performIDR(int scan_pos)
*				   void performRescueOperation(char dir)
*				   void sendPathtoRescueBot(struct pathStack path)
*				   void gotoNearestCornerAndStop()
*				   int main(void)
*
*Global Variables:  int point_north = 0; 
					int point_east = 1;
					int point_south = 2; 
					int point_west = 3;	
					int current_pos = 0; 
					int not_scanned_plot = 0;	
					int plotScanCounter = 0;
					int orientation=0; 
					int survivor;
					int base_pos = 90;
					int elbow_pos = 0; 
					int gripper_pos = 20; 
					int pltadjpoints[4] = {0}; 
					int scannedPlot[4][4] = {0}; 
					const int R = 8;  
					const int G = 9; 
					unsigned char ADC_Value; 
					unsigned char Left_white_line = 0; 
					unsigned char Center_white_line = 0; 
					unsigned char Right_white_line = 0; 
					volatile unsigned int rescueData[2] = {0}; 
					volatile unsigned int rescueDataIndex = 0; 
					volatile unsigned int rescueCurrentPosition = 0; 
					volatile unsigned int stopSearchRobot = 0; 
					volatile unsigned int blackDebrisCounter = 0; 
					volatile unsigned int resetPath = 0; 
					volatile unsigned long int pulse = 0; 
					volatile unsigned long int red; 
					volatile unsigned long int green; 
					volatile unsigned long int ShaftCountLeft = 0;	
					volatile unsigned long int ShaftCountRight = 0;	
					volatile unsigned int Degrees; 
					volatile unsigned int coordinatesWithBlackDebris[40] = {0}; 
					volatile unsigned int cinfo[100][5] = {0};
					struct pathStack
					{
						int path[100];
						int length;
					};
*
*********************************************************************************/

#define F_CPU 14745600
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include "lcd.h"
#include "led.h"
#include "Servo.h"
#include "color_sensor.h"

int point_north = 0; //variable check if robot is pointing in north direction
int point_east = 1;	//variable check if robot is pointing in east direction
int point_south = 2; //variable check if robot is pointing in south direction
int point_west = 3;	//variable check if robot is pointing in west direction
int current_pos = 0; //variable to keep track of search robot current position in arena
int not_scanned_plot = 0;	//variable to hold coordinate of the plot that search is targeting to scan
int plotScanCounter = 0; //variable to keep track of scanned plot. It will increment after plot is scanned
int orientation=0; //variable to keep track of orientation of the robot in arena
int survivor; //variable to store the type of survivor after scanning the survivor
int base_pos = 90; //variable to keep track of base servo position. At start it will be 90
int elbow_pos = 0; //variable to keep track of elbow servo position. At start it will be 0
int gripper_pos = 20; //variable to keep track of gripper servo position. At start it will be 20
int pltadjpoints[4] = {0}; //This array will store all the available adjacent mid points of the plot. It will store zero if adjacent point is blocked
int scannedPlot[4][4] = {0}; //This matrix represents the 16 plots in arena. It will store 1 if the particular plot is scanned or else it will be 0
const int R = 8; //Constant which defines the type of packet that will be send in case of red survivor 
const int G = 9; //Constant which defines the type of packet that will be send in case of green survivor
unsigned char ADC_Value; //variable for storing ADC value
unsigned char Left_white_line = 0; //variables to keep track of left white line sensor
unsigned char Center_white_line = 0; //variables to keep track of center white line sensor
unsigned char Right_white_line = 0; //variables to keep track of right white line sensor
volatile unsigned int rescueData[2] = {0}; //This array will store the data received from the rescue robot
volatile unsigned int rescueDataIndex = 0; //This variable will keep track of index of array mentioned above
volatile unsigned int rescueCurrentPosition = 0; //variable to store the current position of rescue robot
volatile unsigned int stopSearchRobot = 0; //flag variable indicating search robot to stop or not. 1 = stop, 0 = do not stop
volatile unsigned int blackDebrisCounter = 0; //variable to keep track of count of black debris
volatile unsigned int resetPath = 0; //flag variable indicating whether to reset path to destination of not. 1 = reset path, 0 = do no reset
volatile unsigned long int pulse = 0; //to keep the track of the number of pulses generated by the color sensor
volatile unsigned long int red; // variable to store the pulse count when read_red function is called
volatile unsigned long int green; //variable to store the pulse count when read_green function is called
volatile unsigned long int ShaftCountLeft = 0;	//to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0;	//to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle in degrees for turning
volatile unsigned int coordinatesWithBlackDebris[40] = {0}; //This array will store all encountered black debris coordinates present in arena
volatile unsigned int cinfo[100][5] = {0}; //This data structure will store the information about nodes that are adjacent to any valid co-ordinate in the arena
	                                       //a sample entry to this table(matrix) has been described below:
										   //given below is an entry at 55th row for co-ordinate 55
										   //at j= 0  1  2  3  4
										   //     |65|56|45|54|0|
										   //here 65 is the node north to 55(at index point_north = 0) 
										   //here 56 is the node east to 55(at index point_east = 1)
										   //here 45 is the node south to 55(at index point_south = 2)
										   //here 54 is the node west to 55(at index point_north = 3)
										   //and 0  at the 4th position indicates that node is NOT blocked by a black debris. Had it been 1 then it would have been blocked. 
										   
//below structure will use store calculated path to destination
struct pathStack
{
	int path[100];
	int length;
};

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

ISR(USART0_RX_vect) // ISR for receive complete interrupt for USART0
{
	//How to identify the coordinate which is sent by rescue robot is whether its current position or is it coordinate of black debris
	//because its just a 2 digit no. that rescue robot is going to send in both cases? We can distinguish this kind of data by sending  
	//different end packets for both kind of data. For example in case of information about rescue robots current position the end packet
	//will be 2, so when search robot receives data packet 2 the previous data sent must be rescue robots current position
	
	char data = UDR0;  //store the incoming data in 8 bit variable
	rescueData[rescueDataIndex] = data;  //store the received data in int array
	
	if(rescueData[rescueDataIndex] == 3)  //if end packet received is 3 then the previous data sent must be information about black debris
	{
		cinfo[rescueData[0]][4] = 1;  //update cinfo data structure. on place of black debris coordinate put 1 in 4th column
		coordinatesWithBlackDebris[blackDebrisCounter] = rescueData[0];  //update the coordinatesWithBlackDebris array
		blackDebrisCounter++; //increment blackDebrisCounter after updating coordinatesWithBlackDebris array
		rescueDataIndex = -1; //reset index of rescue data array to -1 so next when it receives data it will store from 0th position
							  //-1 because at the end of the ISR rescueDataIndex is getting incremented so it will become 0 automatically
	}
	
	else if(rescueData[rescueDataIndex] == 2)  //if end packet received is 2 then previous data sent must be the information about rescue current position
	{
		rescueCurrentPosition = rescueData[0];  //store the rescue current position in rescueCurrentPosition variable
		rescueDataIndex = -1;  //reset index of rescue data array to -1
	}
	else if(rescueData[rescueDataIndex] == 7) //if the data received is 7 then set flag variable stopSearchRobot to 1
	{
		stopSearchRobot = 1;
		rescueDataIndex = -1;
	}
	else if(rescueData[rescueDataIndex] == 8) //if the data received is 8 then set flag variable stopSearchRobot to 0
	{
		stopSearchRobot = 0;
		rescueDataIndex = -1;
	}
	rescueDataIndex++; //increment the rescueDataIndex after receiving packet
}

//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}

//ISR for color sensor output
ISR(INT0_vect)
{
	pulse++; //increment on receiving pulse from the color sensor
}


//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibble for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibble to 0
	PortARestore |= Direction; // adding lower nibble for forward command and restoring the PORTA status
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
	//stop(); //Stop robot
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
	elbow_servo_pin_config();
	gripper_servo_pin_config();
	color_sensor_pin_config();
	led_port_config();
}

// Timer 5 initialized in PWM mode for velocity control
// Pre-scale:256
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
	UBRR0L = 0x5F; // 14745600 Hz set baud rate lo
	UBRR0H = 0x00; //set baud rate hi
	UCSR0B = 0x98;
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

//Function To Print Sensor Values At Desired Row And Column Location on LCD
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
	color_sensor_pin_interrupt_init();
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
	left_degrees(deg); //turn left by degree specified
	Center_white_line = ADC_Conversion(2); //take readings of white line sensor
	if(Center_white_line < 70) //check if center white line sensor is on black line or not
	{
		left(); //if center white line is not on black line then turn left
		Center_white_line = ADC_Conversion(2); //take readings of white line sensor
		while (Center_white_line < 70) //keep turning left while center white line does not come on black line
			Center_white_line = ADC_Conversion(2); //while turning keep checking the white line sensor
	}
	_delay_ms(60); //wait for some time
	stop(); //stop after turning left
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
	velocity(255, 255);
	right_degrees(deg); //turn right by degree specified
	Center_white_line = ADC_Conversion(2); //take readings of white line sensor
	if(Center_white_line < 70) //check if center white line sensor is on black line or not
	{
		right(); //if center white line is not on black line then turn right
		Center_white_line = ADC_Conversion(2); //take readings of white line sensor
		while (Center_white_line < 70) //keep turning right while center white line does not come on black line
			Center_white_line = ADC_Conversion(2); //while turning keep checking the white line sensor
	}
	_delay_ms(60); //wait for some time
	stop(); //stop after turning right
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
* Function Name:  findUnscannedPlot
* Input: 		  void
* Output: 		  returns int i.e. plot coordinate that is not scanned
* Logic: 		  it will check scannedPlot matrix in particular order as soon as it finds the 0 in matrix means plot is not scanned
*   			  it will break the loop, calculate the plot coordinate with indices of the the matrix and then return the plot coordinate
* Example Call:	  findUnscannedPlot();
*
*/

int findUnscannedPlot()
{
	int flag = 0, i, j, x, y, plot;
	for(i = 0; i < 4; i++)  //loop shall iterate from row 0 to 4 
	{
		if(i < 2) //if the row index is less than 2 (i.e for lower 2 rows of survivor plots) then the arena arena shall be scanned from right to left
		{
			for(j = 3; j >= 0; j--)//as scan is from right to left column shall change as j=3 to j=0
			{
				if(scannedPlot[i][j] == 0)
				{
					flag = 1;//if flag is set break loop as plot to be scanned is found
					break;
				}
			}
		}			
		else //if the row index is greater than 1 (i.e for upper 2 rows of survivor plots) then the arena shall be scanned from right to left
		{
			for(j = 0; j < 4; j++)//as scan is from left to right column shall change as j=0 to j=3
			{
				if(scannedPlot[i][j] == 0)
				{
					flag = 1;//if flag is set break loop as plot to be scanned is found
					break;
				}
			}
		}
		if(flag == 1)//if flag is 1 break outer loop too for same reason
			break;
	}
	
	//in the following code plot coordinate is computed from the values of i and j
	//eg. if i=0 and j=3 then survivor plot coordinate shall be x=0*2+2=2 and y=3*2+2=8 so that yx is  8*10+2=82
	y = i * 2 + 2;
	x = j * 2 + 2;
	plot = y * 10 + x;
	return plot;
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
void getAdjacentPlotPoints(int plot_coordinate)
{
	int i,j;
	pltadjpoints[0] = plot_coordinate+10;  //calculate adjacent point present on north side of the plot
	pltadjpoints[1] = plot_coordinate+1;   //calculate adjacent point present on east side of the plot
	pltadjpoints[2] = plot_coordinate-10;  //calculate adjacent point present on south side of the plot
	pltadjpoints[3] = plot_coordinate-1;   //calculate adjacent point present on west side of the plot
	
	//iterate over all calculated adjacent point to check if black debris is present on them or not
	for(i = 0; i < 4; i++)
	{
		j = 0;
		//go through coordinatesWithBlackDebris array and compare them with the selected adjacent point
		while(coordinatesWithBlackDebris[j] != 0)
		{
			//if any of the black debris coordinate present in array is equal to adjacent point then it is blocked
			if(coordinatesWithBlackDebris[j] == pltadjpoints[i])
			{
				pltadjpoints[i] = 0;  //set 0 in place of blocked adjacent point
				break;
			}				
			j++; //increment j to goto next coordinatesWithBlackDebris position
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
	while(k <= 65) //loop shall iterate to a maximum of 65
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
		k++; //counter incremented
	}
	//In the following lines the path from the specified source (s) to the destination (d) shall be computed
	paths.length=-1; //the length of the path is set to -1
	paths.path[++paths.length]=d;//the destination shall be at the bottom of the stack and the source at the top
	temp=parent[d];//parent of the destination node is stored in temp
	while(temp!=s)//until the source is reached the path stack will keep building up
	{
		paths.path[++paths.length]=temp;//the parent node(which must be arrived to to get shortest distance) is saved. This shall precede its intended destination
		temp=parent[temp];//parent of the other parent is then copied in a temporary variable
	}
	paths.path[++paths.length]=s;//since the loop terminated at the arrival of the source. The source is saved at the top of the path
	return paths;  //the path is returned
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
	int path_turn = 0; //initialize turns for the path that is calculated
	int minpath_turn = 0;  //initialize turns for path that is stored in minpath
	struct pathStack path, minpath;  //declare variables to store minpath and calculated path
	minpath.length = 0;  //initialize minpath length to 0. In case of any path not found the length of minpath returned will be 0
	for(int i = 0; i < 4; i++)  //iterate over all the 4 plot adjacent point stored in pltadjpoints array
	{
		if(pltadjpoints[i] != 0) //if the adjacent point is not 0
		{
			path = findPath(current_pos, pltadjpoints[i]); // calculate the path between current position and selected adjacent point   
			if(path.length != 0)  //if length of calculated path is not 0
			{
				if(path.length < minlength)  //compare path length with minlength
				{
					minlength = path.length;  //if calculated path length is less then minlength then assign path length to minlength
					minpath = path;  //assign calculated path to minpath
				}
				else if(path.length == minlength) //if calculated path length and minlength both are same
				{
					//then calculate the numbers of turns available in both the paths
					for(int i = 1; i < minlength; i++)
					{
						int a1 = path.path[i-1];
						int a2 = path.path[i];
						int a3 = path.path[i+1];
						if(abs(a1-a2) != abs(a2-a3))
							path_turn++;
					
						a1 = minpath.path[i-1];
						a2 = minpath.path[i];
						a3 = minpath.path[i+1];
						if(abs(a1-a2) != abs(a2-a3))
							minpath_turn++;
					}
					if(path_turn < minpath_turn)  //if the calculated path turn in less then minpath_turn
						minpath = path;  //then assign calculated path minpath 
				}
			}
			path_turn = minpath_turn = 0; //reset both path turn and minpath turn to 0
			minlength = minpath.length;	 //assign minpathlength to minlength variable
		}
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
	while(1)
	{
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line 
		
		//sum value will be greater than 120 if bot is on the one of the node or midpoint
		int sum = Left_white_line + Center_white_line + Right_white_line;
		
		if(sum > 120)  //if sum value is greater than 120 then bot is on the node or midpoint
		{
			//below condition will be used at the start of the run. At start the current_pos of search robot will be initialized to 0
            //when it encounters the node then it will be set to 15 according to our coordinate system
			if(current_pos == 0)
			{
				current_pos = 15;
				orientation = point_north; //set orientation to north because at the start of the run bot will be facing north
				forward_mm(60);
				return;
			}
			//whenever the node is encountered the change_location function will change current position of the robot
			//so before changing the location send previous node position where search came from. This information will used by
			//rescue robot to release node from search robot path
			UDR0 = (char)current_pos;  //send search robot position before changing it
			_delay_ms(5);  //wait for data to be sent

			change_location(current_pos);  //change the current position after encountering the node

			//and also send the new current position. It will be used by rescue robot to exclude search robot detection as a black debris
			UDR0 = (char)current_pos; //send search robot position after changing it
			_delay_ms(5);  //wait for data to be sent
			UDR0 = 2;  //send terminal packet as 2 indicating previous data sent must be search robots current position
			_delay_ms(5);  //wait for data to be sent
			
			velocity(255,255);
			
			//extract x and y from current position
			int x = current_pos % 10;  
			int y = current_pos / 10;
			
			//both search and rescue robot will scan mid point for black debris for total of 2 times. One here in follow function before going forward mm
			//and second in travel function after going forward mm. This is to ensure that black debris detection happens properly
			scanForBlackDebris();
			
			//forward mm is done to ensure proper turn on nodes and avoid double detection of nodes and mid points
			if(x % 2 != 0 && y % 2 != 0) //if its a node for which both x and y coordinate will be odd go 45 mm forward 
				forward_mm(45);
			else                       //else go 20 mm forward
				forward_mm(20);
			
			//check stopSearchRobot flag if it is 1 then stop the robot. This is used in collision avoidance. Whenever rescue robot is standing still on 
			//search robots path rescue robot will send packet and stopSearchRobot will be set 1 to indicating search robot to stop. after telling search 
			//robot to stop rescue robot will go to the nearest free node and after reaching nearest free node, rescue robot will tell search robot move
			//by sending another packet where stopSearchRobot will be set to 0
			if(stopSearchRobot == 1)  //stopSearchRobot is one then stop the robot
				stop();
			while(stopSearchRobot != 0);  //wait while stopSearchRobot becomes 0
			forward();  //after stopSearchRobot becomes 0 search robot can move now
			
			break;  //break the line following loop after encountering node to perform further actions
		}		
		else  //if no node is encounter then simply follow the line
		{
			if(Center_white_line>15)
			{
				forward();
				velocity(255,255);
			}

			else if(Left_white_line>15)
			{
				stop();
				velocity(140,140);
				left();
			}

			else if(Right_white_line>15)
			{
				stop();
				velocity(140,140);
				right();
			}
			
			else if(Center_white_line<=15 && Left_white_line<=15 && Right_white_line<=15)
			{
				forward();
				velocity(255,255);
			}
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
*				  2) store black debris coordinate in coordinatesWithBlackDebris array if black debris if found
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
			
			//There is a possibility that rescue robot is standing on midpoint and search robot is standing on node facing rescue robot
			//search robot will scan that mid point for black debris and the calculated value will be less then 230. In this case we don't have to consider
			//that mid point as black debris so before updating cinfo matrix, coordinatesWithBlackDebris sending it's position to rescue robot
			//check if calculated black debris position is equal to rescue robot position or not. If it is equal then do not consider it as a black debris
			if(blackDebrisPos != rescueCurrentPosition)
			{
				//if calculated black debris position is not equal to rescue robots current position then before doing anything check if it is already scanned or not
				for (int i = 0; coordinatesWithBlackDebris[i] != 0; i++)
				{
					//if that position is already scanned then it will be present in coordinatesWithBlackDebris
					if(coordinatesWithBlackDebris[i] == blackDebrisPos)
						return;  //if it is present then return before updating it
				}
				
				//if it is not present then update coordinatesWithBlackDebris with calculated position
				coordinatesWithBlackDebris[blackDebrisCounter] = blackDebrisPos;
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
* Function Name:  scanPlots
* Input: 		  int plot1, int plot2 -> each mid point has 2 adjacent plot so plot1 and plot2 are the two possible adjacent plot from the midpoint
* Output: 		  void
* Logic: 		  This function will perform IDR based on the passed plot coordinate. It will perform following steps for both the passed plot coordinate
*				  1) If passed plot coordinate is not 0 then perform IDR on that plot
*				  2) After performing IDR update scannedPlot matrix that the plot has been scanned
* Example Call:	  scanPlots(22, 44);
*
*/
void scanPlots(int plot1, int plot2)
{
	int i, j;  //variable to store calculated row and column to update scannedPlot matrix 
	stop();  //stop the robot before scanning the plot
	
	//If both the passed plot coordinates are 0 than both plots are scanned then return from the function
	if(plot1 == 0 && plot2 == 0)
		return;
		
	if(plot1 != 0)  //if coordinate of plot1 is not 0 means it is a valid coordinate
	{
		performIDR(plot1);  //perform IDR on that plot
		
		i = plot1 / 20 - 1;  //calculate row of matrix which needs to updated
		j = (plot1 % 10)/2 - 1;  //calculate column of the matrix which needs to updated
		
		scannedPlot[i][j] = 1;  //set in place of calculated row and column indicating that plot has been scanned
	} 
	if(plot2 != 0)  //if coordinate of plot2 is not 0 means it is a valid coordinate
	{
		performIDR(plot2);  //perform IDR on that plot
		
		i = plot2/20 - 1;  //calculate row of matrix which needs to updated
		j = (plot2%10)/2 - 1;  //calculate column of the matrix which needs to updated
		
		scannedPlot[i][j] = 1;  //set in place of calculated row and column indicating that plot has been scanned
	}
}

/*
*
* Function Name:  scanForWhiteDebris
* Input: 		  void
* Output: 		  void
* Logic: 		  this function will be used to scan the plot for the presence of white debris. It will perform following task
*				  1) calculate the possible two plots adjacent to current position based on orientation of the robot
*				  2) check if the calculated plots are inside the arena and are valid
*				  3) call scanPlots function by passing two calculated plot coordinate
* Example Call:	  scanForWhiteDebris();
*
*/
void scanForWhiteDebris()
{
	int x = current_pos % 10;  //extract x from current position
	int y = current_pos / 10;  //extract y from current position
	int plot1, plot2;  //variable to store plot positions
	int i, j;  //variable to store calculated row and column to update scannedPlot matrix
	int NS = 10, EW = 1;  //constants to add in current position based on orientation
	
	//if either x or y is even then robot is standing on mid point then only do the finding plot operation
	if(x % 2 == 0 || y % 2 == 0)
	{
		//if orientation is either north or south then add EW constant to current position to get two adjacent plot
		if(orientation == point_north || orientation == point_south)
		{
			plot1 = current_pos - EW;  //do minus EW to get left side of plot
			plot2 = current_pos + EW;  //do plus EW to get right side of plot
		}
		//else orientation is either east or west then add NS constant to current position to get two adjacent plot
		else
		{
			plot1 = current_pos - NS;  //do minus NS to get left side of plot
			plot2 = current_pos + NS;  //do plus NS to get right side of plot
		}						
		
		//check if plot is valid
		if(isPlotValid(plot1))
		{
			x = plot1 % 10;
			y = plot1 / 10;
			
			i = y / 2 - 1;
			j = x / 2 - 1;
			
			//check if plot is already scanned or not by checking in scannedPlot matrix
			if(scannedPlot[i][j] == 1)
				plot1 = 0;  //if it is already scanned then set it 0
		}
		else  //if plot is not valid means it is outside the arena then set it 0
			plot1=0;
		
		//check if plot is valid
		if(isPlotValid(plot2))
		{
			x = plot2 % 10;
			y = plot2 / 10;
			
			i = y/2 - 1;
			j = x/2 - 1;
			
			//check if plot is already scanned or not by checking in scannedPlot matrix
			if(scannedPlot[i][j] == 1)
				plot2 = 0;
		}
		else  //if plot is not valid means it is outside the arena then set it 0
			plot2=0;
		
		//call scanPlots function by passing two calculated plot to perform further operation 
		scanPlots(plot1, plot2);
		
		//If search robot has to scan two plots form its mid point and both of them has survivor then rescue robot will perform MSR
		//after rescue robot has done scanning both the plots for survivor type. This done to prevent collision. This done by sending a packet to
		//rescue robot which indicates that search robot has scanned both adjacent plot and rescue robot is allowed to service
		UDR0 = 5;
		_delay_ms(5);
	}
}

void red_read(void) // function to select red filter and display the count generated by the sensor on LCD. The count will be more if the color is red. The count will be very less if its blue or green.
{
	//Red
	filter_red(); //select red filter
	pulse=0; //reset the count to 0
	_delay_ms(100); //capture the pulses for 100 ms or 0.1 second
	red = pulse;  //store the count in variable called red
}

void green_read(void) // function to select green filter and display the count generated by the sensor on LCD. The count will be more if the color is green. The count will be very less if its blue or red.
{
	//Green
	filter_green(); //select green filter
	pulse=0; //reset the count to 0
	_delay_ms(130); //capture the pulses for 100 ms or 0.1 second
	green = pulse;  //store the count in variable called green
}

/*
 *
 * Function Name: check_for_survivor
 * Input: void 
 * Output: void
 * Logic:  1) the color sensor is turned on and the color is sensed 
 *         2) while the color is being sensed the bot shall move a little to get a proper reading
 *	 	
 * Example Call: performIDR(44);
 *					
 */
void check_for_survivor()
{
	velocity(150,150);
	int greenCounter = 0;//green counter is set to 0
	turn_on_color_sensor();//color sensor turned on 
	_delay_ms(100);
	
	red_read();
	green_read();
	if(green > red)//if green read is greater than red in first swipe then green counter is incremented 
		greenCounter++;
	
	left_degrees(7); //move slightly to left
	
	red_read();
	green_read();
	if(green > red) //if green read is greater than red in first swipe then green counter is incremented 
		greenCounter++;
	
	right_degrees(10); //move slightly to right
	
	red_read();
	green_read();
	if(green > red) //if green read is greater than red in first swipe then green counter is incremented 
		greenCounter++;
		
	left_degrees(15);// move back to original orientation
		
	if(greenCounter > 0) //if the greenCounter is greater than zero then the green read must have been greater than red read at some point while the scan
		survivor = G; //color is set to green
	else
		survivor = R; //color is set to red
	
	turn_off_color_sensor();

	stop();
}


/*
 *
 * Function Name: performRescueOperation
 * Input: char dir -> a char to denot which direction the servo mechnism must be rotated in 
 * Output: void
 * Logic:  1)depending the dir specified the base servo shall turn left or right 
 *         2)the arm moves down holds the survivor and moves it to the side in the clearing zone
 *          3)the arm then moves over the survivor to sense the color 
 *		   4)the LED is glowed to the color that is sensed by color sensor and rearrange to its default orientation 
 *	 	
 * Example Call: performIDR(44);
 *					
 */
void performRescueOperation(char dir)
{
	gripper_rotate(20);//open gripper servo
	_delay_ms(200);
	elbow_rotate(120); //move down elbow servo
	_delay_ms(200);
	gripper_rotate(85); //close gripper servo
	_delay_ms(200);
	elbow_rotate(100); //left elbow servo slightly
	_delay_ms(200);
	if(dir == 'L')//if dir specified is Left the the base servo moves toward 20 degrees or else 160 degrees
		base_rotate(20);
	else
		base_rotate(160);
	_delay_ms(200);
	elbow_rotate(115);//the elbow servo moves slightly below
	_delay_ms(200);
	gripper_rotate(20);//the white debris is dropped
	_delay_ms(200);
	elbow_rotate(80);//lift elbow servo 
	_delay_ms(200);
	if(dir == 'L')//if dir specified was left then move base servo to 0 or else 180 so as to bring color sensor over survivor
		base_rotate(0);
	else
		base_rotate(180);
	_delay_ms(200);
	gripper_rotate(90); //close gripper servo
	_delay_ms(200);
	elbow_rotate(134); //bring down the elbow servo
	_delay_ms(200);
	
	check_for_survivor(); //sense the color of the survivor
	
	if(survivor == R) //glow red LED if red survivor
		red_led();
	else if(survivor == G) //glow green LED if green survivor
		green_led();
	_delay_ms(2000);
	turn_off_led();
	
	elbow_rotate(0);//lift elbow servo
	_delay_ms(200);
	gripper_rotate(20);//open gripper servo
	_delay_ms(200);
}

/*
 *
 * Function Name: performIDR
 * Input: int scan_pos -> the co-ordinate of the survivor plot to be scanned 
 * Output: void
 * Logic:  1) This function will make the bot scan the specified survivor plot
 *         2) The bot shall compare its own orientation to with difference between scan position and current position to turn its servo mechanism accordingly
 *         3) The search bot shall then move the white debris away from the survivor and then scan its color
 *		   4) if no white debris is sensed then glow blue LED and ring the buzzer
 *		   5) the servo mechanism shall be reset its default orientation which is 90 degrees 
 *	 	
 * Example Call: performIDR(44);
 *					
 */
void performIDR(int scan_pos)
{	
	char dir;
	
	//the difference between a co-ordinate and the one to its north is 10
	//the difference between a co-ordinate and the one to its south is -10
	//the difference between a co-ordinate and the one to its east is 1
	//the difference between a co-ordinate and the one to its west is -1
	int diff=current_pos-scan_pos;
	
	//Please note that the default orientation of kit-dropping mechanism is the the same as that of the bot which is towards the front. Hence the base servo shall by default be at 90 degrees
	if (diff==1&&orientation==point_north||orientation==point_south&&diff==-1||diff==10&&orientation==point_west||orientation==point_east&&diff==-10)//The search robot's servo mechanism must turn left if either of the conditions  is true
	{
		base_rotate(0);
		dir = 'L'; // dir of the servo mechanism is set as left
	}
	if (diff==-1&&orientation==point_north||orientation==point_south&&diff==1||diff==-10&&orientation==point_west||orientation==point_east&&diff==10)//The rescue robot's servo mechanism must turn right if either of the conditions  is true
	{
		base_rotate(180);
		dir = 'R';  // dir of the servo mechanism is set as right
	}
	
	_delay_ms(200);
	
	int sharp = ADC_Conversion(11);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
	int value = Sharp_GP2D12_estimation(sharp);
	lcd_print(2,1,value,3);
	if(value < 200 && value > 20) //checks if white debris is present or not
	{
		performRescueOperation(dir);  //function makes the search robot to move the white debris and then scan the color of the survivor
		UDR0 = (char)survivor; //send survivor color to the rescue bot
		_delay_ms(5);
		UDR0 = (char)scan_pos;//send survivor co-ordinate to rescue bot
		_delay_ms(5);
		UDR0 = 4;
		_delay_ms(5);
	}
	else
	{
		//if no white debris is found then ring the buzzer for 2 seconds and glow blue LED 
		int sharp = ADC_Conversion(11);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
		int value = Sharp_GP2D12_estimation(sharp);
		lcd_print(2,1,value,3);
		buzzer_on();
		blue_led();
		_delay_ms(2000);
		buzzer_off();
		turn_off_led();
	}
	base_rotate(90);  //reset the servo mechanism to its default orientation
	stop();
	plotScanCounter++;//increment the number of plots scanned i.e plotScanCounter
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
		turn_left(70);
    } 
	else if (current_orientation == point_west) 
	{ 
        velocity(255,255);
		turn_right(70);
    }
	else if (current_orientation == point_south)
	{
		velocity(255,255);
		turn_right(160);
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
		turn_right(70);
    } 
	else if (current_orientation == point_west) 
	{
        velocity(255,255);
		turn_left(70);
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
	    turn_left(70);
    } 
	else if (current_orientation == point_south) 
	{ 
		velocity(255,255);
        turn_right(70);      
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
	   turn_right(70);
    } 
	else if (current_orientation == point_south) 
	{ 
        velocity(255,255);
        turn_left(70);
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
		
		scanForBlackDebris();
		scanForWhiteDebris();
		if(resetPath != 1)
			follow();
}

/*
 *
 * Function Name: travelPath
 * Input: paths
 * Output: void
 * Logic:  1) This function will make the bot travel from node to node by calling travel function
 *         2) and also check the reset path flag if it is one then the path needs to be recalculated so break the current travel path loop
 *	 	
 * Example Call: travelPath(paths);
 *					
 */
void travelPath(struct pathStack paths)
{
	for(int i = paths.length; i > 0; i--) //the loop shall iterate from the length of the structure to 1 
	{
		travel(paths.path[i], paths.path[i-1]); //travel function is called node by node to travel from one node to other adjacent node 
		
		if(resetPath == 1)//if the path has been reset then stop following and break
		{
			stop();
			return;
		}
	}
	//After reaching the destination plot scan for the white debris
	scanForWhiteDebris();
}

/*
 * Function Name: sendPathtoRescueBot
 * Input: struct pathStack path -> the path to be followed to reach intended destination
 * Output: void
 * Logic:  this function shall make the search bot send its path to the rescue bot so it may avoid traveling by this path 
 * Example Call: sendPathtoRescueBot(path);			
 */
void sendPathtoRescueBot(struct pathStack path)
{
	for (int i = path.length; i >= 0; i--)//loop shall iterate from the length of the path to 0 and send the path node by node
	{
		UDR0 = (char)path.path[i];
		_delay_ms(5);
	}
	UDR0 = 1; //to indicate end of path sending routine to the rescue
	_delay_ms(50);
}

/*
 * Function Name: gotoNearestCornerAndStop
 * Input: void
 * Output: void
 * Logic:  this function shall make the search bot go to the nearest corner in the arena after it has scanned all plots so that rescue may travel freely in the arena
 * Example Call: gotoNearestCornerAndStop();			
 */
void gotoNearestCornerAndStop()
{
	int cornersCoordinates[4] = {11, 19, 91, 99}; //these are the co-ordinates of the corner nodes
	int selectedCorner = 0;
	//loop shall end when the search bot arrives at some corner co-ordinate 
	do 
	{
		selectedCorner = 0;
		int mindiff = 999; //parameter for closest node computation is set to a large value
		for(int i = 0; i < 4; i++) //loop iterates through all 4 corner vertex
		{
			int diff = abs(current_pos - cornersCoordinates[i]); //the node which gives the least difference shall be our chosen node
			if(diff < mindiff)
			selectedCorner = cornersCoordinates[i];//selectedCorner is changed if a different corner gives a lesser difference i.e closer 
		}
		struct pathStack paths = findPath(current_pos, selectedCorner); //path is computed for the intended corner
		travelPath(paths);
	}
	while(current_pos != selectedCorner);		
	
}

/*
 * Function Name: main
 * Input: void
 * Output: void
 * Logic:  1) the servo mechanism is set to its default orientation
           2) Repeat the following steps until all plots are scanned i.e plotScanCounter ==16
		      a)check reset path flag if set to 1 then reset to 0
			  b)check plotScanCounter?=16 if yes then send packet to rescue to set its searchComplete flag and then go and stand at the nearest corner node. If not then continue
			  c)fetch the next unscanned plot
			  d)get the locations from where the search may scan the survivor i.e the mid-point nodes which are adjacent to the survivor plot
			  e)obtain the shortest path to reach any one of those nodes 
			  f)follow that path
		   3)stop
			  
 * Example Call: gotoNearestCornerAndStop();			
 */
int main(void)
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	color_sensor_scaling();
	init_cinfo_matrix();//initialize the table that holds co-ordinate information
	
	//set the servo mechanism to its default orientation
	base_servo(90);
	_delay_ms(300);
	gripper_servo(20);
	_delay_ms(300);
	elbow_servo(0);
	_delay_ms(300);
	servo_base_free();
	servo_elbow_free();
	servo_gripper_free();
	follow();//follow line 
	scanForBlackDebris();//scan for black debris at the front of start 
	while(1)
	{
		stop();//stop
		if(resetPath==1)//if path has been reset then change the flag to 0
		    resetPath = 0;
			
		if(plotScanCounter == 16)//if all plots scanned then send packet to rescue to set its searchComplete flag and then go and stand at the nearest corner node
		{
			stop();
			UDR0 = 7;//send terminal packet to the rescue to set its searchComplete flag to 1
			_delay_ms(5);
			gotoNearestCornerAndStop();//go and stand at the nearest corner node
			break;
		}
		not_scanned_plot = findUnscannedPlot();//fetch the next unscanned plot
		getAdjacentPlotPoints(not_scanned_plot);//get the locations from where the search may scan the survivor i.e the mid-point nodes which are adjacent to the survivor plot
		struct pathStack path = selectMinPathtoDestinationPlot();
		sendPathtoRescueBot(path);//obtain the shortest path to reach any one of those nodes 
		travelPath(path);//follow that path
		int sharp = ADC_Conversion(11);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
		int value = Sharp_GP2D12_estimation(sharp);
		lcd_print(2,1,value,3);
	}
	stop();
}