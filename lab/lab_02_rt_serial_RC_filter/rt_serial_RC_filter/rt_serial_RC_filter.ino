#define PWM_pin 9
#define analog_read_pin A0
#define squarewave_pin 11
byte inByte;

int nISR;
int nIn;
int ISR_Happened;
int mypause = 1;
int ISRstate = 0;
int vIn,vOut;
bool send_ser;

unsigned long t0;
unsigned long t;
unsigned long t1;
unsigned long cur_t, prev_t;
float t_ms;

int reassemblebytes(unsigned char msb, unsigned char lsb){
  int output;
  output = (int)(msb << 8);
  output += lsb;
  return output;
}


unsigned char getsecondbyte(int input){
    unsigned char output;
    output = (unsigned char)(input >> 8);
    return output;
}


int readtwobytes(void){
    unsigned char msb, lsb;
    int output;
    int iter = 0;
    while (Serial.available() <2){
      iter++;
      if (iter > 1e5){
	break;
      }
    }
    msb = Serial.read();
    lsb = Serial.read();
    output = reassemblebytes(msb, lsb);
    return output;
}

void SendTwoByteInt(int intin){
    unsigned char lsb, msb;
    lsb = (unsigned char)intin;
    msb = getsecondbyte(intin);
    Serial.write(msb);
    Serial.write(lsb);
}



void setup(){
   Serial.begin(115200);
   Serial.println("RT Serial for RC filter v.1.0.0");
   pinMode(PWM_pin, OUTPUT);
   pinMode(squarewave_pin, OUTPUT);
   pinMode(analog_read_pin, INPUT);


   //=======================================================
   // set up the Timer2 interrupt
   //=======================================================
   cli();// disable global interrupts temporarily
   TCCR2A = 0;// set entire TCCR3A register to 0
   TCCR2B = 0;// same for TCCR3B
   //TCNT3  = 0;//initialize counter value to 0
   // set compare match register for 8khz increments
   OCR2A = 154;// = (16*10^6) / (8000*8) - 1 (must be <255)
   // turn on CTC mode
   //TCCR3A |= (1 << WGM31);
   //TCCR3B = (TCCR3B & 0b11111000) | 0x07;// set prescaler to 1034
   // taken from here: https://playground.arduino.cc/Main/TimerPWMCheatsheet
   //        scroll down to "Pins 11 and 3: controlled by timer 3"
   //TCCR3B = (TCCR3B & 0b11111000) | 0x06;

   TCCR2B |= (1 << WGM12);

   // Set CS10 and CS12 bits for 1024 prescaler:
   TCCR2B |= (1 << CS10);
   TCCR2B |= (1 << CS12);

   TIMSK2 |= (1 << OCIE2A);
   sei();// re-enable global interrupts
   //=======================================================
}


void loop(){
  cur_t = micros();
  t = cur_t-t0;
  if (t < 0){
    t += 65536;
  }
  t_ms = t/1000;
  
  unsigned char lsb, msb;
  if (ISR_Happened == 1){
    ISR_Happened = 0;
    if (Serial.available() > 0) {
      inByte = Serial.read();
      if (inByte == 1){
        //main control case
	nIn = readtwobytes();
        vIn = readtwobytes();
	if (nIn == 0){
	  t0 = micros();
	}
      }
      else if (inByte == 2){
        //start new test
        send_ser = true;
        nISR = -1;
	vIn = 0;
	vOut = 0;
        delay(5);
        SendTwoByteInt(2);// make python wait until it recieves this
      }
      else if (inByte == 3){
	// end test
        send_ser = false;
        vIn = 0;
        delay(5);
        SendTwoByteInt(3);// make python wait until it recieves this
      }
    }
    if (send_ser){
        SendTwoByteInt(nISR);
	SendTwoByteInt(t);
        SendTwoByteInt(vIn);
	SendTwoByteInt(vOut);
        Serial.write(10);//newline
      }
  }
}


ISR(TIMER2_COMPA_vect)
{
  ISR_Happened = 1;
  nISR++;
  // - send pwm out analogWrite
  analogWrite(PWM_pin,vIn);
  // - read vOut using analogRead
  vOut = analogRead(analog_read_pin);  
  if (ISRstate == 1){
    ISRstate = 0;
    digitalWrite(squarewave_pin, LOW);
  }
  else{
    ISRstate = 1;
    digitalWrite(squarewave_pin, HIGH);
  }
}
