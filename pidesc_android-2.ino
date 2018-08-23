#include <PID_v1.h>
//#include <Servo.h>
//Servo X;
//Define Variables we'll be connecting to
double Setpoint, Input, lastinput, Output, acctime = 0, Kp = 9.5,  Ki = 3.6, Kd = 1, Error = 0;
double LogOutput;
boolean acc = true, next = true, data = false;
int outputPin = 9;
int z = 0;       // added
volatile byte rotation; // variale for interrupt fun
//int rotation;
double  dtime, mtime, mintime ;
double v, prpm, A, rpm;
unsigned long timetaken, pevtime, stepLength, timer, steadytime = 0;
int hall = 3;
int led = 13, Count = 0, enb = 10, b = 0;
double NewSampleTime;
int received[30]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // 11 - edit - to receive 11 speed stages : Speed, Acceleration time, Steady time
//int received[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, acctime, LogOutput, DIRECT);
//PID SetSampleTime(&NewSampleTime);

unsigned long serialTime ; // this will help us know when to talk with processing

void setup()
{
  //initialize the serial link with processing
  // pinMode(led, OUTPUT);
  //digitalWrite(hall, HIGH);
  pinMode(outputPin, OUTPUT);
  //  for (int i=0; i<200; i+=1)
  //  {
  //
  //    digitalWrite(outputPin, HIGH);
  //    delayMicroseconds(2000);
  //    digitalWrite(outputPin, LOW);
  //    delay(20-(2000/1000));
  //  }
  //  for (int i=0; i<500; i+=1)   // setting up the esc
  //  {
  //    digitalWrite(outputPin, HIGH);
  //    delayMicroseconds(1000);
  //    digitalWrite(outputPin, LOW);
  //    delay(20-(1000/1000));
  //  }
  Serial.begin(9600);
  //delay(500); // delay() is OK in setup as it only happens once
  while(!Serial);
  //Serial.println("ready Pi");
  Serial.flush();

  pinMode(enb, OUTPUT);
  //digitalWrite(enb, HIGH);   // edit
  pinMode(hall, INPUT);
  //digitalWrite(outputPin, LOW);
  cli();//disable interrupts
  sei();//enable int
  rotation = 0;
  attachInterrupt(1, magnet_detect, RISING);//second pin of arduino used as interrupt

  timetaken = 0;
  rpm = 0; prpm = 0; dtime = 0;
  pevtime = 0;
  //setPwmFrequency(3,8);
  //initialize the variables we're linked to
  Input = 0;
  Output = 0;
  stepLength = 0;
  Setpoint = 0;
  //Error = Setpoint;

  //myPID.Compute();
  myPID.SetMode(AUTOMATIC);
  //X.attach(outputPin);
  //  X.write(100);
  //  delay(1000);
  digitalWrite(enb, LOW);    // added

}

void loop()
{
  if (micros() - dtime > 1000000)
  {
    Input = 0;//to drop down to zero when braked.
  }

SerialReceive1();
  //serialTime += 500;
  //}
  //else {
  
  if (data) {
    //digitalWrite(enb, LOW);    // added - to make disable maxon controller after the cycle completes

    SerialSend1();
  }
 
myPID.Compute();
  

  
  //Output = map (Output , 200.0, 2000.0, 180.0,1578.0);
  //Output = Setpoint;
  Output = constrain(Output, 1000.0, 7000.0); //edit - Output = constrain(Output,1000.0,7000.0);
  // if(Output==900)
  // {
  //  digitalWrite(enb, LOW);
  // }
  if (Setpoint == 0) {
    Output = 1000;
    digitalWrite(enb, LOW);    // added
  }
  else
  {
    digitalWrite(enb, HIGH);    // added
  }
  digitalWrite(outputPin, HIGH);
  delayMicroseconds(Output);
  digitalWrite(outputPin, LOW);
  delayMicroseconds(10000 - Output);


  //send-receive with processing if it's time
  //if (millis() > serialTime){
  //if (!data)  // changed 23-05-2017
    
  //if (millis() > serialTime)
  //{
  //Serial.println(Input);
}

//void myDelay(unsigned long p) {
//  if (p > 16380) {
//    delay (p / 1000);
//  } else {
//    delayMicroseconds(p);
//  }
//}

void magnet_detect()//Called whenever a magnet is detected
{
  rotation++;
  double tempTime;

  if (rotation >= 7)
  {
    timetaken = micros() - pevtime; //time in millisec
    tempTime = (1000000.0 / double(timetaken));
    Input = tempTime * 60.0 * float(rotation) / 7.0;
    //delayMicroseconds(100);
    pevtime = micros();
    dtime = micros();
    rotation = 0;
    //delayMicroseconds(10);
  }
}


/********************************************
   Serial Communication functions / helpers
 ********************************************/

        // float array



// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats

//  the bytes coming from the arduino follow the following
//  format:
//  0: 0=Manual, 1=Auto, else = ? error ?
//  1: 0=Direct, 1=Reverse, else = ? error ?
//  2-5: float setpoint
//  6-9: float input
//  10-13: float output
//  14-17: float P_Param
//  18-21: float I_Param
//  22-245: float D_Param

/*union {                // This Data structure lets
  byte asBytes[4];    // us take the byte array
   long int asInt[0];    // sent from processing and
  }                      // easily convert it to a
  fooa;                   // float array
*/


void SerialReceive1() {

  /* while (Serial.available() && index < 8){
    fooa.asBytes[index] = Serial.read();
    index++;
    }
    if (index == 4){
     Setpoint =   ((fooa.asInt[0])/100);
     acctime =  (fooa.asInt[0])%100;*/
  //while(Serial.available()){

  //String serial = Serial.readString();
  //Serial.println(serial.toInt());
  //Serial.println(Setpoint);
  //Serial.println(acctime);
  //Serial.println((fooa.asInt[0]));
  if  (Serial.available()>0) {
    b = 0;
    next = false;

    digitalWrite(enb, LOW);
    //digitalWrite(enb, HIGH);
    for (int i = 0; i <= 30; i++) { // edit - to receive 11 speed stages : Speed, Acceleration time, Steady time
      String serial = Serial.readStringUntil(',');
      received[i] = serial.toInt();
    }
    Setpoint = double(received[0]);
    acctime = double(received[1]);

    steadytime = (received[2])* 1000;
    data = true ;
     double p , i , d ;
  myPID.SetTunings(p, i, d, acctime , LogOutput);


  Serial.println(b);
  //Serial.println((int)myPID.Getacctime());

  //Serial.flush();
  }
  //while (Serial.available()){
  //int serial = Serial.read();
  //long int serial = Serial.parseInt();
  // Setpoint  = serial/100;
  //acctime= serial%100;
  //Serial.println(serial);
  //Setpoint  = 2500;
  //acctime = 10;
  //}
 

}
void SerialSend1() {
  double p , i , d ;


  /* if (myPID.Getsteady())
    timer = micros();

    if ((micros()-timer) > (steadytime*1000000)){
    next = true;
    //b++;
    //b=1;
    }
    //     if(b==3) {
    //      Setpoint = 0;
    //      acctime = 0;
    //     }*/
  if (myPID.Getsteady() ) {
    
    timer = millis();
    Serial.println(timer);
    //acc = true;
 /*   for (z = 0; z < steadytime * 108; z++) { //  steady loop


      if (micros() - dtime > 1000000)
        Input = 0;//to drop down to zero when braked.


     
      myPID.Compute();
      //Output = map (Output , 200.0, 2000.0, 180.0,1578.0);
      //Output = Setpoint;
      Output = constrain(Output, 1000.0, 7000.0); // edit from : Output = constrain(Output,1000,7000.0);
      digitalWrite(enb, HIGH);  // commented

      if (Setpoint == 0){
        Output = 1000;
        digitalWrite(enb, LOW);
        }
        else
        {
        digitalWrite(enb, HIGH);
        }                                   // edited
      digitalWrite(outputPin, HIGH);
      delayMicroseconds(Output);
      digitalWrite(outputPin, LOW);
      delayMicroseconds(10000 - Output);

      //digitalWrite(enb, HIGH);


    } */
    //next = true;

  }
  if ((millis()-timer) > (steadytime)){
    Serial.println(b);
    next = true;
  }
  
  if (next && b < 9) {
    b++;
    Setpoint = double(received[3 * b]);
    acctime = double(received[(3 * b) + 1]);
 myPID.SetTunings(p, i, d, acctime , LogOutput);
    steadytime = (received[(3 * b) + 2]) * 1000;
    next = false;
  }


 
  //digitalWrite(enb, HIGH);
  if (b == 9) { // edit               // edited
    data = false;
    Serial.println(b);
    //digitalWrite(enb, LOW);
  }

}

