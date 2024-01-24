#include "PinChangeInterrupt.h" 
#include <TimerOne.h>


#define Hall_A 8 //15;  // Pin do odczytu Halla A
#define Hall_B 9 //14;  // Pin do odczytu Halla B
#define Hall_C 10//16;  // Pin do odczytu Halla C

#define H_A 6//9; // Pin do sygnału mos Hi gałęzi A PWM
#define L_A 7//8; // Pin do sygnału mos Lo gałęzi A
#define H_B 5//5; // Pin do sygnału mos Hi gałęzi B PWM
#define L_B 4//4; // Pin do sygnału mos Lo gałęzi B
#define H_C 3//3; // Pin do sygnału mos Hi gałęzi C PWM
#define L_C 2//2; // Pin do sygnału mos Lo gałęzi C

#define POT_pin A0


byte hall_states_temp [6] = 
{
  5,1,3,2,6,4
};
// {//A  B  C
//   {1, 0, 1}, //5
//   {0, 0, 1}, //1
//   {0, 1, 1}, //3
//   {0, 1, 0}, //2
//   {1, 1, 0}, //6
//   {1, 0, 0}  //4
// };

// bool motor_states [6][6] = 
// {//2  1  8  4  2  1
//   {0, 1, 1, 0, 0, 0}, //5 0x18
//   {0, 1, 0, 0, 1, 0}, //1 0x12
//   {0, 0, 0, 1, 1, 0}, //3 0x06
//   {1, 0, 0, 1, 0, 0}, //2 0x24
//   {1, 0, 0, 0, 0, 1}, //6 0x21
//   {0, 0, 1, 0, 0, 1}  //4 0x09
// // 1L 1H 2L 2H 3L 3H
// // 7  6  5  4  3  2
// // 8  7  6  5  4  3
// };

byte hall_state = 0b000;
int pwm_pin, POT_Value;
unsigned long startTime = 0;
const int defaultFrequency = 10;


void HALL_A_ISR(void){
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(Hall_A));
  if(trigger == FALLING)
    hall_state |= 0b100;
  if(trigger == RISING)
    hall_state &= ~0b100;
}
void HALL_B_ISR(void){
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(Hall_B));
  if(trigger == FALLING)
    hall_state |= 0b010;
  if(trigger == RISING)
    hall_state &= ~0b010;
}
void HALL_C_ISR(void){
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(Hall_C));
  if(trigger == FALLING)
    hall_state |= 0b001;
  if(trigger == RISING)
    hall_state &= ~0b001;
}

void bldc_move(int hall_state){

  switch(hall_state)
  {
    case 1:
      PORTD = 0b01001000;
      pwm_pin = H_A;
      break;
    case 2:
      PORTD = 0b10010000;
      pwm_pin = H_B;
      break;
    case 3:
      PORTD = 0b00011000;
      pwm_pin = H_B;
      break;
    case 4:
      PORTD = 0b00100100;
      pwm_pin = H_C;
      break;
    case 5:
      PORTD = 0b01100000;
      pwm_pin = H_A;
      break;
    case 6:
      PORTD = 0b10000100;
      pwm_pin = H_C;
      break;
    default:
      PORTD = 0b00000000;
      pwm_pin = 0;
      break;
  }
}


void setup() {

  noInterrupts();

  Serial.begin(9600);
  DDRD |= 0xFC;
  pinMode(Hall_A, INPUT_PULLUP);
  pinMode(Hall_B, INPUT_PULLUP);
  pinMode(Hall_C, INPUT_PULLUP);

  attachPCINT(digitalPinToPCINT(Hall_A), HALL_A_ISR, CHANGE);
  attachPCINT(digitalPinToPCINT(Hall_B), HALL_B_ISR, CHANGE);
  attachPCINT(digitalPinToPCINT(Hall_C), HALL_C_ISR, CHANGE);
  interrupts();

  Timer1.initialize(1000000 / defaultFrequency);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(3, OUTPUT);

  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

}

void loop() {
  // hall_state += 1;




  hall_state = hall_state % 7;

  bldc_move(hall_state);
  Serial.println(hall_state);

  //POT_Value = analogRead(POT_pin); 
  Timer1.pwm(6, 100);
  Timer1.pwm(6, 100);
  Timer1.pwm(6, 100);
  Timer1.pwm(9, 100);
  Timer1.pwm(10, 100);
  Timer1.pwm(11, 100);



  //3129 us
  //3236 us

  //if(motor_speed < 250){
  //  while(TCNT2 < motor_speed);
    // digitalWrite(pwm_pin, LOW);
    // delay(5);
//   //}
//  // if(motor_speed > 0){
//   //  while(TCNT2 >= motor_speed);
    // digitalWrite(pwm_pin, HIGH);
    // delay(5);
  //}
}

//https://howtomechatronics.com/tutorials/arduino/arduino-brushless-motor-control-tutorial-esc-bldc/
