#define echo_int 0 // Interrupt id for echo pulse
#define NUMBER_TICKS 40000 // Used for counting 200ms with 50uS Timer Interrupt
#define trigPin 13 
#define echoPin 12   

volatile long echo_start = 0;                         
volatile long echo_end = 0;                          
volatile long echo_duration = 0;                     
volatile int trigger_time_counter = 0;

void setup(void) {
   pinMode(13,OUTPUT);
   digitalWrite(13,LOW);
   Serial.begin(57600);
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
   sei(); // allow interrupts

   // External Interrupt for Hardware Pin Echo
   attachInterrupt(echo_int, echo_interrupt, CHANGE);  // interrupt ID = 0
}

ISR(TIMER1_COMPA_vect)
{
    trigger_pulse(); // check if time to send trigger pulse
}

void trigger_pulse()
{
      static volatile int state = 0; // Default state
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
           state = 0;                  // go to state 0 in next interrupt
           break;
     }
}

// Note: this routine does not handle the case where the timer counter overflows which will result in the occassional error.

void echo_interrupt()
{
  if (digitalRead(echoPin) == HIGH)
  {
      echo_end = 0;                                 
      echo_start = micros();                        
  }      
  else
  { 
      echo_end = micros();                          
      echo_duration = echo_end - echo_start;        
  }
}

void loop(void)
{
    //cli();  // One way to disable the timer, and all interrupts

    //TCCR1B &= ~(1<< CS12);  // turn off the clock altogether
    //TCCR1B &= ~(1<< CS11);
    //TCCR1B &= ~(1<< CS10);

    //TIMSK1 &= ~(1 << OCIE1A); // turn off the timer interrupt
}
