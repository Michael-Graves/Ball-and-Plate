#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "button.h"
#include "touch.h"
#include "servo.h"
#include "com.h"

// Timer related defines, Change these to modify update times
// All variables in milliseconds (ms)
#define TOUCH_UPDATE_RATE 10

#define UART_UPDATE_DELAY 100
#define UART_UPDATE_RATE 100

#define PID_UPDATE_DELAY 240
#define PID_UPDATE_RATE 40

#define MOTOR_UPDATE_DELAY 280
#define MOTOR_UPDATE_RATE 40


// Maximum PID ErrorSum before Ki saturates (Keep this low as Windup is not handled)
#define PID_ERROR_SUM_RANGE 100000

// Number of motor samples to be averaged (Will slow down the response of the system but reduces
#define MOTOR_SAMPLES 1

// Touch screen center positions
#define CENTER_X 2150
#define CENTER_Y 2150

// Servo center positions and range of movements (in 10th of a degrees, 900 = 90 deg)
#define SERVO_X_ZERO 880
#define SERVO_Y_ZERO 880

#define SERVO_X_RANGE 300
#define SERVO_Y_RANGE 350

// This depends on the python script used to generate the lookup table
#define CIRCLE_SIZE 360
#define CIRCLE_DEFAULT_RATE 5

// Function Definitions
void Setup(void);
void SysTick_Init(unsigned long);
void SysTick_Handler(void);
void OnButtonPushed(_Bool btn1, _Bool btn2);
_Bool UpdateBallPosition(void);
void UpdatePIDController(void);
void UpdateMotor(void);

// Volatile Definitions
volatile uint32_t currentXDegrees = SERVO_X_ZERO;
volatile uint32_t currentYDegrees = SERVO_Y_ZERO;
volatile unsigned long currentTime = 0;
volatile _Bool needTouchUpdate = true;
volatile _Bool needUARTUpdate = false;
volatile _Bool needPIDUpdate = false;
volatile _Bool needMotorUpdate = false;
volatile _Bool needCircleUpdate = false;


// PID Controller Variables
uint32_t SetPosition_X = CENTER_X;
uint32_t SetPosition_Y = CENTER_Y;

int32_t ErrorXLast = 0;
int32_t ErrorXSum = 0;
int32_t ErrorXDif = 0;

int32_t ErrorYLast = 0;
int32_t ErrorYSum = 0;
int32_t ErrorYDif = 0;

// PID K Constants. (Good values: 120, 25, 240)
const int32_t Px = 120;
const int32_t Ix = 5;
const int32_t Dx = 240;

const int32_t Py = 120;
const int32_t Iy = 5;
const int32_t Dy = 220;

// Lookup tables generated with Python script (CircleGenerator.py)
uint16_t CircleUpdateRate = CIRCLE_DEFAULT_RATE;
uint16_t CirclePosition_Index = 0;
const int16_t CirclePosition_X[CIRCLE_SIZE] = {250,250,250,250,249,249,249,248,248,247,246,245,245,244,243,241,240,239,238,236,235,233,232,230,228,227,225,223,221,219,217,214,212,210,207,205,202,200,197,194,192,189,186,183,180,177,174,170,167,164,161,157,154,150,147,143,140,136,132,129,125,121,117,113,110,106,102,98,94,90,86,81,77,73,69,65,60,56,52,48,43,39,35,30,26,22,17,13,9,4,0,-4,-9,-13,-17,-22,-26,-30,-35,-39,-43,-48,-52,-56,-60,-65,-69,-73,-77,-81,-86,-90,-94,-98,-102,-106,-110,-113,-117,-121,-125,-129,-132,-136,-140,-143,-147,-150,-154,-157,-161,-164,-167,-170,-174,-177,-180,-183,-186,-189,-192,-194,-197,-200,-202,-205,-207,-210,-212,-214,-217,-219,-221,-223,-225,-227,-228,-230,-232,-233,-235,-236,-238,-239,-240,-241,-243,-244,-245,-245,-246,-247,-248,-248,-249,-249,-249,-250,-250,-250,-250,-250,-250,-250,-249,-249,-249,-248,-248,-247,-246,-245,-245,-244,-243,-241,-240,-239,-238,-236,-235,-233,-232,-230,-228,-227,-225,-223,-221,-219,-217,-214,-212,-210,-207,-205,-202,-200,-197,-194,-192,-189,-186,-183,-180,-177,-174,-170,-167,-164,-161,-157,-154,-150,-147,-143,-140,-136,-132,-129,-125,-121,-117,-113,-110,-106,-102,-98,-94,-90,-86,-81,-77,-73,-69,-65,-60,-56,-52,-48,-43,-39,-35,-30,-26,-22,-17,-13,-9,-4,0,4,9,13,17,22,26,30,35,39,43,48,52,56,60,65,69,73,77,81,86,90,94,98,102,106,110,113,117,121,125,129,132,136,140,143,147,150,154,157,161,164,167,170,174,177,180,183,186,189,192,194,197,200,202,205,207,210,212,214,217,219,221,223,225,227,228,230,232,233,235,236,238,239,240,241,243,244,245,245,246,247,248,248,249,249,249,250,250,250};
const int16_t CirclePosition_Y[CIRCLE_SIZE] = {0,4,9,13,17,22,26,30,35,39,43,48,52,56,60,65,69,73,77,81,86,90,94,98,102,106,110,113,117,121,125,129,132,136,140,143,147,150,154,157,161,164,167,170,174,177,180,183,186,189,192,194,197,200,202,205,207,210,212,214,217,219,221,223,225,227,228,230,232,233,235,236,238,239,240,241,243,244,245,245,246,247,248,248,249,249,249,250,250,250,250,250,250,250,249,249,249,248,248,247,246,245,245,244,243,241,240,239,238,236,235,233,232,230,228,227,225,223,221,219,217,214,212,210,207,205,202,200,197,194,192,189,186,183,180,177,174,170,167,164,161,157,154,150,147,143,140,136,132,129,125,121,117,113,110,106,102,98,94,90,86,81,77,73,69,65,60,56,52,48,43,39,35,30,26,22,17,13,9,4,0,-4,-9,-13,-17,-22,-26,-30,-35,-39,-43,-48,-52,-56,-60,-65,-69,-73,-77,-81,-86,-90,-94,-98,-102,-106,-110,-113,-117,-121,-125,-129,-132,-136,-140,-143,-147,-150,-154,-157,-161,-164,-167,-170,-174,-177,-180,-183,-186,-189,-192,-194,-197,-200,-202,-205,-207,-210,-212,-214,-217,-219,-221,-223,-225,-227,-228,-230,-232,-233,-235,-236,-238,-239,-240,-241,-243,-244,-245,-245,-246,-247,-248,-248,-249,-249,-249,-250,-250,-250,-250,-250,-250,-250,-249,-249,-249,-248,-248,-247,-246,-245,-245,-244,-243,-241,-240,-239,-238,-236,-235,-233,-232,-230,-228,-227,-225,-223,-221,-219,-217,-214,-212,-210,-207,-205,-202,-200,-197,-194,-192,-189,-186,-183,-180,-177,-174,-170,-167,-164,-161,-157,-154,-150,-147,-143,-140,-136,-132,-129,-125,-121,-117,-113,-110,-106,-102,-98,-94,-90,-86,-81,-77,-73,-69,-65,-60,-56,-52,-48,-43,-39,-35,-30,-26,-22,-17,-13,-9,-4};
;

// Variables for averaging motor set points
uint8_t averageIndex = 0;
uint32_t degreeAverageX[MOTOR_SAMPLES];
uint32_t degreeAverageY[MOTOR_SAMPLES];

// Variables to hold the current ball position
_Bool touchPresent = false;
uint32_t x = 0;
uint32_t y = 0;

uint8_t mode = 0;

void Setup(void) {
    // Setting Clock to 80MHz
      SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    //mInitialization of system components..
    SysTick_Init(80000);
    Button_Init(OnButtonPushed); // on_button_pushed() is provided as the callback function

    COM_Init();

    //Send 'START\r\n' over uart
    UARTCharSend('S'); UARTCharSend('T'); UARTCharSend('A'); UARTCharSend('R'); UARTCharSend('T'); UARTCharSend('\r'); UARTCharSend('\n');

    //Initialize Average Motor Arrays
    uint8_t i;
    for(i = 0; i < MOTOR_SAMPLES; i++) {
        degreeAverageX[i] = SERVO_X_ZERO;
        degreeAverageY[i] = SERVO_Y_ZERO;
    }
    Servo_Init(SERVO_Y_ZERO, SERVO_X_ZERO);

    Touch_Init();
}

int main(void) {
  Setup();

  // Main update loop
  while(1) {
      // Waits for SysTick Timer logic to set the "need____Update" variables for true

      if(needTouchUpdate) {
          //UpdateBallPosition returns true only if the read was successful, keep trying until then
          if(UpdateBallPosition()) {
              // If Read was successful, clear the update flag and set <touchPresent> to true
              needTouchUpdate = false;
              touchPresent = true;
              LEDWrite(RED);
          } else {
              touchPresent = false;
              LEDWrite(OFF);
          }
      }

      // Only called when the current mode is following a circle
      if(needCircleUpdate) {
          needCircleUpdate = false;
          SetPosition_X = CENTER_X + CirclePosition_X[CirclePosition_Index];
          SetPosition_Y = CENTER_Y + CirclePosition_Y[CirclePosition_Index];
      }

      // Update PID controller
      if(needPIDUpdate && touchPresent) {
          needPIDUpdate = false;
          UpdatePIDController();
      }

      // Update Motor with the current PID values
      if(needMotorUpdate && touchPresent) {
          needMotorUpdate = false;
          UpdateMotor();
      }

      // Send the current ball position over UART to a connected Computer
      if(needUARTUpdate && touchPresent) {
          needUARTUpdate = false;
          UARTIntSend(x);
          UARTCharSend(',');
          UARTIntSend(y);
          UARTCharSend('\r');
          UARTCharSend('\n');
      }
  }
}

/* Function called when a button is pushed */
void OnButtonPushed(_Bool btn1, _Bool btn2) {
    // Change mode/state variable depending on the button that was pushed
    if(btn1) {
        mode = (mode + 1)%6;
    } else if(btn2) {
        if(mode > 0) {
            mode --;
        } else {
            mode = 5;
        }
    }

    // Handle new mode
    switch(mode) {
    default:
        needCircleUpdate = false;
        SetPosition_X = CENTER_X;
        SetPosition_Y = CENTER_Y;
        break;
    case(1):
        needCircleUpdate = false;
        SetPosition_X = CENTER_X + 600;
        SetPosition_Y = CENTER_Y;
        break;
    case(2):
        needCircleUpdate = false;
        SetPosition_X = CENTER_X - 600;
        SetPosition_Y = CENTER_Y;
        break;
    case(3):
        CircleUpdateRate = CIRCLE_DEFAULT_RATE;
    case(4):
        CircleUpdateRate = CIRCLE_DEFAULT_RATE;
    }
}

_Bool UpdateBallPosition() {
    if(!Touch_Present()) return false;
    x = Touch_Read_X();
    if(!Touch_Present()) return false;
    y = Touch_Read_Y();
    return true;
}

int32_t Limit(int32_t value, int32_t min, int32_t max) {
    if(value > max) return max;
    if(value < min) return min;
    return value;
}

int32_t Abs(int32_t value) {
    if(value < 0) return -value;
    return value;
}

void UpdatePIDController(void) {
    int32_t ErrorX = SetPosition_X - (int32_t)(x); //Range of -4096 to 4096
    int32_t ErrorY = SetPosition_Y - (int32_t)(y); //Range of -4096 to 4096

	// This should have code for resetting/handling wind-up
    ErrorXSum = Limit(ErrorXSum + ErrorX, -PID_ERROR_SUM_RANGE, PID_ERROR_SUM_RANGE); //*dt
    ErrorYSum = Limit(ErrorYSum + ErrorY, -PID_ERROR_SUM_RANGE, PID_ERROR_SUM_RANGE); //*dt

    ErrorXDif = Limit(ErrorX - ErrorXLast, -500, 500); //dt
    ErrorYDif = Limit(ErrorY - ErrorYLast, -500, 500); //dt

    // Calculate PID Control
    int32_t xVal = ((Px * ErrorX) + (Ix * ErrorXSum)/5 + (Dx * ErrorXDif) * 5)/100;
    int32_t yVal = ((Py * ErrorY) + (Iy * ErrorYSum)/5 + (Dy * ErrorYDif) * 5)/100;

    averageIndex = (averageIndex + 1) % MOTOR_SAMPLES;

    currentXDegrees = SERVO_X_ZERO + Limit(xVal / 10, -SERVO_X_RANGE, SERVO_X_RANGE);
    degreeAverageX[averageIndex] = currentXDegrees;

    currentYDegrees = SERVO_Y_ZERO + Limit(yVal / 10, -SERVO_Y_RANGE, SERVO_Y_RANGE);
    degreeAverageY[averageIndex] = currentYDegrees;

    ErrorXLast = ErrorX;
    ErrorYLast = ErrorY;
}

void UpdateMotor(void) {
    uint32_t sumX = 0;
    uint32_t sumY = 0;
    uint8_t i;
    for(i = 0; i < MOTOR_SAMPLES; i++) {
        sumX += degreeAverageX[i];
        sumY += degreeAverageY[i];
    }

    Servo_Set_Degrees(SERVO_1, sumY / MOTOR_SAMPLES);
    Servo_Set_Degrees(SERVO_2, sumX / MOTOR_SAMPLES);
}

void SysTick_Init(unsigned long period) {
    //Disable interrupts and Systick while setting up
    IntMasterDisable();
    SysTickDisable();

    SysTickPeriodSet(period - 1);

    //Force the counter to reload by writing any value to this register
    NVIC_ST_CURRENT_R = 0;

    //Set the priority of the SysTick Interrupt to 2 (value is in the top 3 bits)
    NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|(2 << 29); // priority 2

    //Set SysTick clock source to main core clock.
    NVIC_ST_CTRL_R = NVIC_ST_CTRL_CLK_SRC;

    // Register the interrupt handler
    SysTickIntRegister(SysTick_Handler);

    //Enable Systick and resume interrupts
    SysTickEnable();
    IntMasterEnable();
}

/* Interrupt service routine for SysTick Interrupt
 *  Executes every 1 ms
 *  Checks if each system needs an update
 *  based on their _UPDATE_RATE define
*/
void SysTick_Handler(void){
    currentTime++;

    if((currentTime % TOUCH_UPDATE_RATE) == 0) {
        needTouchUpdate = true;
    }

    if((currentTime > PID_UPDATE_DELAY) && ((currentTime % PID_UPDATE_RATE) == 0)) {
        needPIDUpdate = true;
    }

    if((currentTime > UART_UPDATE_DELAY) && ((currentTime % UART_UPDATE_RATE) == 0)) {
        needUARTUpdate = true;
    }

    if((currentTime > MOTOR_UPDATE_DELAY) && ((currentTime % MOTOR_UPDATE_RATE) == 0)) {
        needMotorUpdate = true;
    }

    // If the mode is a circle update mode, increment the index
    if((mode == 3) && (currentTime % CircleUpdateRate) == 0) {
		// Increment Up
        CirclePosition_Index = (CirclePosition_Index + 1) % CIRCLE_SIZE;
        needCircleUpdate = true;
    } else if((mode == 4) && (currentTime % CircleUpdateRate) == 0) {
		// Increment Down
        if(CirclePosition_Index > 0) {
            CirclePosition_Index = (CirclePosition_Index - 1);
        } else {
            CirclePosition_Index = CIRCLE_SIZE - 1;
        }
        needCircleUpdate = true;
    }
}

