#include <PID_v1.h>
//#include <Servo.h>
//Servo X;
//Define Variables we'll be connecting to
double Setpoint, Input,lastinput, Output,acctime = 0, Kp = 0.3,  Ki = 0.99, Kd = 0.95,Error = 0;
double LogOutput;
int outputPin = 9;
volatile byte rotation; // variale for interrupt fun
//int rotation;
double  dtime,mtime,mintime ;
double v, prpm, A,rpm;
unsigned long timetaken, pevtime, stepLength;
int hall = 2;
int led = 13, Count = 0,enb = 10;
double NewSampleTime;
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
//   for (int i=0; i<200; i+=1) 
//  {
//
//    digitalWrite(outputPin, HIGH);
//    delayMicroseconds(2000); 
//    digitalWrite(outputPin, LOW);
//    delay(20-(2000/1000));
//
//  }
//   for (int i=0; i<500; i+=1)   // setting up the esc
//  {
//
//    digitalWrite(outputPin, HIGH);
//    delayMicroseconds(1000); 
//    digitalWrite(outputPin, LOW);
//    delay(20-(1000/1000));
//
//  }
  Serial.begin(115200);
  Serial.flush();
 
  pinMode(enb, OUTPUT);
  digitalWrite(enb, HIGH);
  pinMode(hall, INPUT);
   //digitalWrite(outputPin, LOW);
   cli();//disable interrupts
   sei();//enable int
   rotation = 0;
  attachInterrupt(0, magnet_detect, RISING);//second pin of arduino used as interrupt
  
 timetaken = 0;
  rpm = 0; prpm = 0;dtime = 0;
  pevtime = 0;
  //setPwmFrequency(3,8);
  //initialize the variables we're linked to
  Input = 0;
  Output = 0;
  stepLength = 0;
  Setpoint = 0;
  Error = Setpoint;
  
//myPID.Compute();
  myPID.SetMode(AUTOMATIC);
   //X.attach(outputPin);
//  X.write(100);
//  delay(1000);
}

void loop()
{

 
  
 if (micros() - dtime > 1000000)
  Input = 0;//to drop down to zero when braked.
 
 
Error = myPID.GetKp()*(Setpoint - Input);
  
  
 //Input = rpm;

// if (Input > inmax) Input = inmax;
//   if (Input < inmin) Input = inmin;
  
  
  
 
  //Output = constrain(Output,180,1360);
  //Output = 550;
  //if (Setpoint == 0)Output = 180;
  //X.write(Output);
 //Serial.println(Output);
digitalWrite(enb, HIGH);  
//if (((Setpoint - Input) > 10) || ((Input - Setpoint) > 10))
 myPID.Compute();
 //Output = map (Output , 200, 2000, 180,1578);
 Output = constrain(Output,1000,7000);
 digitalWrite(enb, HIGH);
 
if (Setpoint == 0)Output = 1000;
 digitalWrite(outputPin, HIGH);
 delayMicroseconds(Output); 
 digitalWrite(outputPin, LOW);
 delayMicroseconds(10000-Output);


  //send-receive with processing if it's time
  if (millis() > serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime += 500;
  }
//lastinput = Input-50;
digitalWrite(enb, HIGH);
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
    tempTime = (1000000.0/double(timetaken));   
    Input = tempTime*60.0*float(rotation)/7.0;
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


union {                // This Data structure lets
  byte asBytes[24];    // us take the byte array
  float asFloat[6];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array



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
void SerialReceive()
{

  // read the bytes sent from Processing
  int index = 0;
  byte Auto_Man = -1;
  byte Direct_Reverse = -1;
  while (Serial.available() && index < 26)
  {
    if (index == 0) Auto_Man = Serial.read();
    else if (index == 1) Direct_Reverse = Serial.read();
    else foo.asBytes[index - 2] = Serial.read();
    index++;
  }

  // if the information we got was in the correct format,
  // read it into the system
  if (index == 26  && (Auto_Man == 0 || Auto_Man == 1) && (Direct_Reverse == 0 || Direct_Reverse == 1))
  {
    Setpoint = double(foo.asFloat[0]);
    //Input=double(foo.asFloat[1]);       // * the user has the ability to send the
    //   value of "Input"  in most cases (as
    //   in this one) this is not needed.
    //if (acctime == 0)                    // * only change the output if we are in
    //{ //   manual mode.  otherwise we'll get an
      acctime = double(foo.asFloat[2]);    //   output blip, then the controller will
      
   // }
    double p, i, d;                       // * read in and set the controller tunings
   /* if(Setpoint >= 0 && Setpoint <=2500){
      p = 1.5;
      i = 0.8;
      d = 0.01;
    }
    else if (Setpoint > 2500 && Setpoint <= 6000){
      p = 8.3;
      i = 2.4;
      d = 0.005;
    }
    else if (Setpoint > 6000){
      p = 9.9;
      i = 3.5;
      d = 0.15;
    }*/
    /*p = 0.25;
      i = 2.6;
      d = 0;*/
    p = double(foo.asFloat[3]);           //
    i = double(foo.asFloat[4]);           //
    d = double(foo.asFloat[5]);          //
    myPID.SetTunings(p, i, d, acctime , LogOutput);            //
     //Output = map (Setpoint,0,12000,180,1360);

    if (Auto_Man == 0) myPID.SetMode(MANUAL); // * set the controller mode
    else myPID.SetMode(AUTOMATIC);             //

    if (Direct_Reverse == 0) myPID.SetControllerDirection(DIRECT); // * set the controller Direction
    else myPID.SetControllerDirection(REVERSE);          //
  }
  Serial.flush();                         // * clear any random data from the serial buffer
}

// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void SerialSend()
{
  Serial.print("PID "); //O
  Serial.print(Setpoint); //1
  Serial.print(" ");
  Serial.print(Input);//2
  Serial.print(" ");
  Serial.print(myPID.Getacctime());//3
  Serial.print(" ");
  Serial.print(myPID.GetKp(),5);//4
  Serial.print(" ");
  Serial.print(myPID.GetKi(),5);//5 >> ki = Ki * sampletime 
  Serial.print(" ");
  Serial.print(myPID.GetKd(),5);//6 >> kd = Kd / sampletime
  Serial.print(" ");
  if (myPID.GetMode() == AUTOMATIC) Serial.print("Automatic"); //7
  else Serial.print("Manual");
  Serial.print(" ");
  if (myPID.GetDirection() == DIRECT) Serial.print("Direct"); //8
  else Serial.print("Reverse");
 //Serial.print("\n");
  Serial.print(" ");
  Serial.print(Output);//9 >> Mapped output
  Serial.print(" ");
  Serial.print(Error,5);//10 >> Kp * (setpoint - feedback (input))
  Serial.print(" ");
  Serial.print(myPID.GetLogOutput());//11 >> PID output >> before mapping
  Serial.print(" ");
  Serial.print(myPID.GetITerm());//12 >> ITERM - sum of error
  Serial.print(" ");
  Serial.println(myPID.GetdInput(),5); // 13 >> DInput
  
  
  
   
}

