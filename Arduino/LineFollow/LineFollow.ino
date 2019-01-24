/*
Component     Port     Arduino Pin
              1REV      7
              1EN       24
              1FWD      6
LCHB-100      2REV      3
              2EN       25
              2FWD      2
              
              TX        18
HC-05         RX        19

              Trigger   23
HC-SR04       Echo      22

Yellow LED    -         13 

ROS COMMAND : rosrun rosserial_python serial_node.py /dev/ttyACM0
*/
//First Motor
#define REV1 7
#define EN1 24
#define FWD1 6

// Second Motor
#define REV2 3
#define EN2 25
#define FWD2 2

// Onboard LED
#define LED 13

// SPEED Control
#define BASE_RATE 127

// Ultrasonic sensor SR-04
#define trigPin 23 
#define echoPin 22   
#define THRESHOLD 5 // Threshold Distance for stopping(cm)

// TIMER 1
#define NUMBER_TICKS 4000 // Used for counting 200ms with 50uS Timer Interrupt

// TIMER 2
#define NUMBER_TICKS_2 200 // Used for counting 200ms with 1ms Timer Interrupt

#include <ros.h>
#include <geometry_msgs/Twist.h>

//#include <String.h>

volatile long echo_duration = 0;                     
volatile int trigger_time_counter = 0;
volatile int trigger_time_counter_2 = 0;
float Multiplier = 1; // Set by SR-04 (Distance)

// For Bluetooth
class NewHardware : public ArduinoHardware {
public: NewHardware():ArduinoHardware(&Serial1, 57600)
{};
}; 

ros::NodeHandle_<NewHardware> nh;

//ros::NodeHandle nh;
// Motor Control
void moveForward(int speedForward)
{
    analogWrite(FWD1,speedForward);
    analogWrite(FWD2,speedForward);
    digitalWrite(EN1,HIGH);
    digitalWrite(EN2,HIGH);
}
void moveBackward(int speedBackward)
{
    analogWrite(REV1,speedBackward);
    analogWrite(REV2,speedBackward);
    digitalWrite(EN1,HIGH);
    digitalWrite(EN2,HIGH);
}
void brake()
{
    digitalWrite(REV1,LOW);
    digitalWrite(FWD1,LOW);
    digitalWrite(EN1,LOW);
    digitalWrite(REV2,LOW);
    digitalWrite(FWD2,LOW);
    digitalWrite(EN2,LOW);  
}
void moveRight(int turnspeed)
{
    //digitalWrite(REV2,HIGH);
//    analogWrite(FEV1,(turnspeed*));
    analogWrite(REV1,turnspeed);
    digitalWrite(EN1,HIGH);    
}
void moveLeft(int turnspeed)
{
    //digitalWrite(REV2,HIGH);
    analogWrite(REV2,turnspeed);
    digitalWrite(EN2,HIGH);    
}

// Callback to process Geometry/Twist message
void velocityCallback(const geometry_msgs::Twist& msg)
{
  TIMSK2 |= (0 << OCIE2A); // disable timer compare interrupt NOT SLEEPING  
  //cli(); // stop interrupts 
  //nh.loginfo("Inside callback");
  
  if (msg.linear.x > 0) // Positive velocity
  {
      moveForward(BASE_RATE*Multiplier);
  }
  if (msg.linear.x < 0) // Negative Velocity
  {
      moveBackward(BASE_RATE*Multiplier);
  }
  if (msg.angular.z < 0) // Turn Right
  {
      moveRight(BASE_RATE*msg.angular.z*-1);
  }
  if (msg.angular.z > 0) // Turn Left
  {
      moveLeft(BASE_RATE*msg.angular.z);
  }
  
  //sei();
  trigger_time_counter_2 = 200;  // Reload counter value [200 mS]
  TIMSK2 |= (1 << OCIE2A); // enable timer compare interrupt for SLEEP  
  
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &velocityCallback);

void setup()
{ 
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(REV1, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(FWD1, OUTPUT);
  pinMode(REV2, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(FWD2, OUTPUT);
  
  brake(); // STOP all motors

  pinMode(trigPin, OUTPUT);                           
  pinMode(echoPin, INPUT);

  nh.initNode();
  nh.subscribe(sub);

  interrupts();
   
  // TIMER 1 interrupt every 50 uS
  cli(); // stop interrupts
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0
  OCR1A = 799; // 50 microsecond cycle -----------> compare match register = [ 16,000,000Hz/ (prescaler * desired interrupt frequency) ] - 1
  TCCR1B |= (1 << WGM12);// turn on CTC mode
  TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10); // Set CS12, CS11 and CS10 bits for 1 prescaler   
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt   

  // TIMER 2 interrupt every 1 mS
  TCCR2A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT2  = 0; // initialize counter value to 0
  OCR2A = 255; //  1 millisecond cycle -----------> compare match register = [ 16,000,000Hz/ (prescaler * desired interrupt frequency) ] - 1
  TCCR2A |= (1 << WGM21);// turn on CTC mode
  TCCR2B |= (1 << CS22) | (0 << CS21) | (0 << CS20); // Set CS12, CS11 and CS10 bits for 64 prescaler   
  TIMSK2 |= (1 << OCIE2A); // enable timer compare interrupt   
  sei(); // allow interrupts
}

ISR(TIMER1_COMPA_vect)
{
  static volatile int state = 0; // Default state
  //char result[8]; 
  float distance_cm = 0;
  if (!(--trigger_time_counter))   // counter to count to 200mS and send pulse [check distance every 200 ms]
  {                                          
      trigger_time_counter = NUMBER_TICKS;  // Reload counter value [50uS * 4000 = 200 mS]
      state = 1; // go to state 1 and send Pulse NOW
  }    
  switch(state)
  {
      case 0:  // Default state -> do nothing
              break;
      case 1: 
             digitalWrite(trigPin, HIGH); // Pulse start 
             state = 2;                   // and go to state 2 
             break;                       //in next interrupt
      case 2:
             digitalWrite(trigPin, LOW); // Pulse stop
             echo_duration = pulseIn(echoPin, HIGH);
             distance_cm = (echo_duration/2) / 29.1;
             //dtostrf(distance_cm, 6, 2, result);
             //nh.loginfo(result);  
            if (distance_cm < THRESHOLD) 
            {
              Multiplier = 0;
              brake();
            }
            else
            {
              Multiplier = distance_cm/10;
            }
            state = 0;
            break;
  }
}
// ISR FOR SLEEP TIMER
ISR(TIMER2_COMPA_vect)
{
  if (trigger_time_counter_2 == 0)   // counter to count to 200mS and send pulse [check distance every 200 ms]
  {                                          
      trigger_time_counter_2 = 200;  // Reload counter value [1ms * 200 = 200 mS]
      brake();
  }    
  --trigger_time_counter_2; 
}

void loop()
{    
  nh.spinOnce();
  delay(1);
}
