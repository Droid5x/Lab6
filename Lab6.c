/* Aaron Clippinger, Lance Gerday, Bradley Jewett, Mark Blanco
   Section 2 Side A Seat 5
   October 27, 2014
   Lab 6
*/


#include <stdio.h>
#include <stdlib.h>
#include <c8051_SDCC.h>
#include <i2c.h>

#define MAX_RANGE 100
#define R_ADDR 0xE0
#define C_ADDR 0xC0
#define fan_PW_NEUT 2750 // Neutral PW value
#define fan_PW_MIN 2000 // Minimum left PW value
#define fan_PW_MAX 3500 // Maximum right PW value


//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void Port_Init(void);
void PCA_Init(void);
unsigned int Read_Compass();
void Steering(unsigned int current_heading);
void Interrupt_Init(void);
void SMB_Init(void);
void ADC_Init(void); // Initialize A to D Conversion
unsigned char read_AD_input(void);
unsigned char read_ranger(void);
void Check_Menu(void);
void Load_Menu(void);
void Angle_Adjust(void);
void Fan_Update(void);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
unsigned char interrupts; // Counter for refreshing ranger and compass values
unsigned int c = 0; // Counter for printing data at regular intervals
unsigned char take_heading = 0; // Boolean flag to read the compass
unsigned char getRange = 1; // Boolean flag to tell if safe to read ranger

long int fan_C_PW = fan_PW_NEUT; // Start PW at neutral
long int fan_L_PW = fan_PW_NEUT; // Start PW at neutral
long int fan_R_PW = fan_PW_NEUT; // Start PW at neutral
unsigned int angle_PW = 2500; // Motor Pulsewidth to control motor speed

__xdata unsigned int desired_heading = 900; // Set initial heading to 90 degrees

signed int prev_error = 0; // Previous compass error
signed int error; // Global variable for compass error
unsigned int compass_val; // Current heading
unsigned int range_val = 0; // Range value in cm
float voltage; // Global voltage variable for checking battery voltage
unsigned char Data[2]; // Array for sending and receiving from ranger
unsigned int heading_adj; // Range-based heading adjustment

__xdata float derivative_gain = 45; // Derivative gain
__xdata float proportional_gain = 2; // Compass gain setting

__sbit __at 0xB6 SS_range; // Assign P3.6 to SS (Slide Switch)
__sbit __at 0xB7 SS_steer; // Slide switch input pin at P3.7

//-----------------------------------------------------------------------------
// Main Function
//-----------------------------------------------------------------------------

void main(void) {
    // initialize board
    Sys_Init();
    putchar(' '); //the quotes in this line may not format correctly
    Port_Init();
    PCA_Init();
    SMB_Init();
    ADC_Init();
    Interrupt_Init();
    printf("Starting\n\r");

    //print beginning message
    printf("Embedded Control Blimp\r\n");
    printf("desired heading\t actual heading\t ranger reading\t heading adjustment\t rudder pw\t voltage\n\r");
    Angle_Adjust();
    Load_Menu();
    //Main Functionality
	// Start a new ping
    Data[0] = 0x51; // write 0x51 to reg 0 of the ranger:
    // write one byte of data to reg 0 at R_ADDR
    i2c_write_data(R_ADDR, 0, Data, 1);
    while (1) {
	    if (SS_steer) { // If the slide switch is active, set PW to center
            fan_L_PW = fan_PW_NEUT;
            fan_R_PW = fan_PW_NEUT;
            fan_C_PW = fan_PW_NEUT;
            Fan_Update();
        } else if (take_heading) { // Otherwise take a new heading
            compass_val = Read_Compass();
            Steering(compass_val); // Change PW based on current heading
            Fan_Update();
        }
        if (getRange) {
            getRange = 0; // Reset 80 ms flag
            range_val = read_ranger(); // Read the distance from the ranger
            // Floor the value of the ranger within the specified limits:
            if (range_val > MAX_RANGE) range_val = MAX_RANGE;
            if (!SS_range){
                heading_adj = (int)(36 * (MAX_RANGE/2 - range_val));
            } else {
                //heading_adj = 0;
            }
            // Start a new ping
            Data[0] = 0x51; // write 0x51 to reg 0 of the ranger:
            // write one byte of data to reg 0 at R_ADDR
            i2c_write_data(R_ADDR, 0, Data, 1);
        }
		// Hold the motor in neutral if the slide switch is active
        if (c >= 25) {
            //Print Serial Output for data collection
            printf("%d\t %d\t %d\t %d\t %d\t "
                    , desired_heading, compass_val, range_val, heading_adj, fan_C_PW);
            printf_fast_f("%f\n\r", voltage);

            // Print the battery voltage (from AD conversion);
            voltage = read_AD_input();
            voltage *= 0.04;
            printf_fast_f("Battery voltage is: %.2f\n\r", voltage);
            Load_Menu();
            c = 0;
        }
        while (SS_range && SS_steer) {
            fan_R_PW = fan_PW_NEUT;
            fan_L_PW = fan_PW_NEUT;
            fan_C_PW = fan_PW_NEUT;
            Fan_Update();
            Check_Menu();
            c = 0;
            while (c < 5) {
            }
            c = 0;
        }
    }
}


//-----------------------------------------------------------------------------
// Helper Functions
//-----------------------------------------------------------------------------
//

unsigned int Read_Compass() {
    unsigned char c_Data[2]; // c_Data array to store heading data
    unsigned int heading; // Variable to store heading data
    // Read data from compass registers, store it in c_Data buffer
    i2c_read_data(C_ADDR, 2, c_Data, 2);
    //Take high and low c_data bytes, convert to int
    heading = (((unsigned int) c_Data[0] << 8) | c_Data[1]);
    take_heading = 0;
    return heading; // Return C_Data heading between 0 and 3599 
}

void Check_Menu() {
    signed char menu_input = read_keypad(); //Determine pressed button on keypad
    unsigned int keypad_input;

    if ((menu_input - '0') == 1) { //If proportional gain is selected
        printf("Please enter a 5 digit gain constant (of the form : xx.xxx) \n\r");
                lcd_clear();
                lcd_print("Enter a 5 digit gain\nconstant (xx.xxx)");
        while (read_keypad() != -1);
                keypad_input = kpd_input(1);
                proportional_gain = keypad_input * 0.001;
                printf_fast_f("New proportional gain is %f\n\r", proportional_gain);
                Load_Menu();
        } else if ((menu_input - '0') == 2) { //If derivative gain is selected
        printf("Please enter a 5 digit gain constant (of the form : xx.xxx) \n\r");
                lcd_clear();
                lcd_print("Enter a 5 digit gain\nconstant (xx.xxx)");
        while (read_keypad() != -1);
                keypad_input = kpd_input(1);
                derivative_gain = keypad_input * 0.001;
                printf_fast_f("New derivative gain is %f\n\r", derivative_gain);
                Load_Menu();
        } else if ((menu_input - '0') == 3) { //If desired heading is selected
        printf("Please choose an option: \n\r");
                //Print menu on terminal output
                printf("1: 0 degrees\n\r2: 90 degrees\n\r3: 180 degrees\n\r4 : 270 degrees\n\r5 : Enter a value\n\r");
                lcd_clear();
                //Print menu on lcd
                lcd_print("\n1.0 deg   2.90 deg\n3.180 deg 4.270 deg\n5.Enter a value");
        while (read_keypad() != -1);
        menu_input = read_keypad();
        while (menu_input == -1) menu_input = read_keypad();
        if ((menu_input - '0') == 1) { //For 0 degrees
            desired_heading = 0;
			while(read_keypad() != -1);
        } else if ((menu_input - '0') == 2) { //For 90 degrees
            desired_heading = 900;
			while(read_keypad() != -1);
        } else if ((menu_input - '0') == 3) { //For 180 degrees
            desired_heading = 1800;
			while(read_keypad() != -1);
        } else if ((menu_input - '0') == 4) { //For 270 degrees
            desired_heading = 2700;
			while(read_keypad() != -1);
        } else if ((menu_input - '0') == 5) { //For enter own value
            printf("Please enter a 5 digit compass heading (of the form : 0xxxx) \n\r");
                    lcd_clear();
                    lcd_print("\nEnter a 5 digit\nheading (0xxxx)\n\r");

            while (read_keypad() == -1);
            keypad_input = kpd_input(1);
            desired_heading = keypad_input % 3600;
        }
        printf("New desired heading is %d\n\r", desired_heading);
                Load_Menu();
    }
}

void Load_Menu(void) {
	printf("in the loading func\n\r");
    //unsigned int PW_Percent;
    lcd_clear();
    lcd_print("1. Prop Gain\n");
    lcd_print("2. Derivative Gain\n");
    lcd_print("3. Desired Heading\n");
	PW_Percent = (abs(fan_C_PW - fan_PW_NEUT)*200.0) / ((fan_PW_MAX - fan_PW_MIN));
    lcd_print("R:%3dH:%4dS:%2dB:%3d\n", range_val, compass_val, PW_Percent, (int) voltage*10);
}



//-----------------------------------------------------------------------------
// read_ranger
//-----------------------------------------------------------------------------
//
// Read the value of the ultrasonic ranger
// 

unsigned char read_ranger(void) {
    unsigned int range = 0;
    unsigned char slave_reg = 2; //start at register 2
    unsigned char num_bytes = 2; //read 2 bytes
	// read two bytes, starting at reg 2
    i2c_read_data(R_ADDR, slave_reg, Data, num_bytes);
	range = (((unsigned int) Data[0] << 8) | Data[1]);
    return range;
}


//-----------------------------------------------------------------------------
// Port_Init
//-----------------------------------------------------------------------------
//
// Set up ports for input and output
//

void Port_Init() {

    XBR0 = 0x25; // configure crossbar with UART, SPI, SMBus, and CEX channels
				 // set output pins P0.4, P0.5, P0.6, P0.7 for push-pull
				 // mode (CEX 0, 1, 2, 3)
	P0MDOUT |= 0xF0;

    // Port 1 ADC
    P1MDIN &= ~0x20; //set P1.5 to Analog input
    P1MDOUT &= ~0x20; //set P1.5 to open drain mode
    P1 |= 0x20; //set P1.5 to high impedance

    P3MDOUT &= ~0xC0; // Set P3.6 and 3.7 to inputs
    P3 |= 0xC0; //set P3.6 and 3.7 to high impedance

}

//-----------------------------------------------------------------------------
// ADC_Init
//-----------------------------------------------------------------------------
//
// initilize analog to digitial conversion
//

void ADC_Init(void) {

    REF0CN = 0x03; // Use internal reference voltage (2.4V)
    ADC1CN = 0x80; // Enable A/D conversion
    ADC1CF &= 0xFC; // Reset last two bits to 0
    ADC1CF |= 0x01; // Gain set to 1.0
}

//-----------------------------------------------------------------------------
// read_AD_input
//-----------------------------------------------------------------------------
//
// read analog input
//

unsigned char read_AD_input(void) {
    AMX1SL = 5; // Set pin 5 as the analog input
    ADC1CN &= ~0x20; // Clear 'conversion complete' flag
    ADC1CN |= 0x10; // Initiate A/D conversion
    while ((ADC1CN & 0x20) == 0x00); // Wait for conversion to complete
    return ADC1; // Return digital conversion value
}

//-----------------------------------------------------------------------------
// PCA_Init
//-----------------------------------------------------------------------------
//
// Set up Programmable Counter Array
//

void PCA_Init(void) {
    PCA0MD = 0x81; // enable CF interrupt, use SYSCLK/12
    PCA0CN = 0x40; // enable PCA0 counter
    // select 16bit PWM, enable positive edge capture, 
    // enable pulse width modulation(ranger)
    PCA0CPM0 = 0xC2;
    PCA0CPM1 = 0xC2;
    PCA0CPM2 = 0xC2;
    PCA0CPM3 = 0xC2;
}

//-----------------------------------------------------------------------------
// Interrupt_Init
//-----------------------------------------------------------------------------
//
// Set up interrupts
//

void Interrupt_Init(void) {
	EIE1 |= 0x08; //Enable PCA0 Interrupt (bit 3) 
	EA = 1; //Enable global interrupts

}

//-----------------------------------------------------------------------------
// SMB_Init
//-----------------------------------------------------------------------------
//
// Configure Magnetic Compass Interface
//

void SMB_Init(void) {
	SMB0CR = 0x93; //Configure SCL frequency to 100kHz
	ENSMB = 1; // Enable SMBus
}



//-----------------------------------------------------------------------------
// PCA_ISR
//-----------------------------------------------------------------------------
//
// Interrupt Service Routine for Programmable Counter Array Overflow Interrupt
//

void PCA_ISR(void) __interrupt 9 {
    if (CF) { // If an interrupt has occured
        interrupts++;
        c++; // Counter for initial wait to initialize motor
    	if (interrupts % 2 == 0) {
        	take_heading = 1; // It is appropriate to take a reading
    	}
        if (interrupts >= 4) {
			getRange = 1; // 80ms flag
            interrupts = 0; // Reset counter
        }
        CF = 0; // Clear Interrupt Flag
        PCA0 = 28672; // Jump timer ahead for given period
    }
    PCA0CN &= 0xC0; // Handle other PCA interrupts
}

//-----------------------------------------------------------------------------
// Steering
//-----------------------------------------------------------------------------
//

void Steering(unsigned int current_heading) {
    error = (desired_heading + heading_adj)%3600 - current_heading; // Calculate signed error
    if (error > 1800) { // If the error is greater than 1800
    	error = 3600 % error; // or less than -1800, then the 
        error *= -1; // conjugate angle needs to be generated
    } else if (error < -1800) { // with opposite sign from the original
        error = 3600 % abs(error); // error
    }
    
    // Update PW based on error and distance to obstacle
    fan_C_PW = 0.5*((long)proportional_gain * (long)error +( long)derivative_gain*((long)error-(long)prev_error)) + (long)fan_PW_NEUT;
    fan_L_PW = 0.5*((long)proportional_gain * (long)error +( long)derivative_gain*((long)error-(long)prev_error)) + (long)fan_PW_NEUT;
    fan_R_PW = ((long)proportional_gain * (long)error +( long)derivative_gain*((long)error-(long)prev_error))*-1 + (long)fan_PW_NEUT;
    
    if (fan_C_PW > fan_PW_MAX) { // Check if pulsewidth maximum exceeded
        fan_C_PW = fan_PW_MAX; // Set PW to a maximum value
    } else if (fan_C_PW < fan_PW_MIN) { // Check if less than pulsewidth min
        fan_C_PW = fan_PW_MIN; // Set fan_PW to a minimum value
    }


	if (fan_L_PW > fan_PW_MAX) { // Check if pulsewidth maximum exceeded
		fan_L_PW = fan_PW_MAX; // Set PW to a maximum value
	} else if (fan_L_PW < fan_PW_MIN) { // Check if less than pulsewidth min
		fan_L_PW = fan_PW_MIN; // Set PW to a maximum value
	}


	if (fan_R_PW > fan_PW_MAX) { // Check if pulsewidth maximum exceeded
		fan_R_PW = fan_PW_MAX; // Set PW to a maximum value
	} else if (fan_R_PW < fan_PW_MIN) { // Check if less than pulsewidth min
		fan_R_PW = fan_PW_MIN; // Set PW to a maximum value
	}
    prev_error = error;
}

void Angle_Adjust(void){
    while(1){
        char input;
        PCA0CP1 = 0xFFFF - angle_PW;
        printf("PW: %u \n\r", angle_PW);
        //wait for a key to be pressed
        input = getchar();
        if(input == 'd')  // single character input to increase the pulsewidth
        {
            angle_PW += 10;
        }
        else if(input == 'a')  // single character input to decrease the pulsewidth
        {
            angle_PW -= 10;       // Subtract 10 from PW
        }
        else if (input == 's')  // single character input to select value as final
        {
            printf("Value selected!\n\r");
            return;
        }
    }
}

void Fan_Update(void){
    PCA0CP0 = 0xFFFF - fan_C_PW;
    PCA0CP2 = 0xFFFF - fan_L_PW;
    PCA0CP3 = 0xFFFF - fan_R_PW;
}