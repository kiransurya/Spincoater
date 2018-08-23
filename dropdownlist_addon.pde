import java.nio.ByteBuffer;
import processing.serial.*;
import controlP5.*;
import java.util.Timer;
import java.util.TimerTask;


/***********************************************
 * User spcification section
 **********************************************/
int windowWidth = 1024;      // set the size of the 
int windowHeight = 760;     // form

float InScaleMin = 0;       // set the Y-Axis Min
float InScaleMax = 16000;    // and Max for both
float OutScaleMin = 0;      // the top and 
float OutScaleMax = 90;    // bottom trends


int windowSpan = 50000;    // number of mS into the past you want to display
int refreshRate = 100;      // how often you want the graph to be reDrawn;

//float displayFactor = 1; //display Time as Milliseconds
float displayFactor = 1000; //display Time as Seconds
//float displayFactor = 600000; //display Time as Minutes

String outputFileName = ""; // if you'd like to output data to 
// a file, specify the path here

/***********************************************
 * end user spec
 **********************************************/

int nextRefresh;
int arrayLength = windowSpan / refreshRate+1;
int[] InputData = new int[arrayLength];     //we might not need them this big, but
int[] SetpointData = new int[arrayLength];  // this is worst case
int[] OutputData = new int[arrayLength];


float inputTop = 25;
float inputHeight = (windowHeight-150)*2/3;
float outputTop = inputHeight+50;
float outputHeight = (windowHeight-150)*1/6;

float ioLeft = 135, ioWidth = windowWidth-ioLeft;
float ioRight = ioLeft+ioWidth;
float pointWidth= (ioWidth)/float(arrayLength-1);

int vertCount = 10;

int nPoints = 0;

float Input, Setpoint, Output, feedoutput, feederror;
String LogDisplay1,LogDisplay2,LogDisplay3,LogDisplay4, LogDisplay5;

boolean madeContact =false;
boolean justSent = true;
boolean pause = false ;
DropdownList ports;              //Define the variable ports as a Dropdownlist.
//Serial port;                     //Define the variable port as a Serial object.
int Ss;                          //The dropdown list will return a float value, which we will connvert into an int. we will use this int for that).
String[] comList ;               //A string to hold the ports in.
boolean serialSet;               //A value to test if we have setup the Serial port.
boolean Comselected = false;     //A value to test if you have chosen a port in the list.
Serial myPort;

ControlP5 controlP5;

controlP5.Button AMButton, DRButton;
controlP5.Textlabel AMLabel, AMCurrent, InLabel, 
OutLabel, SPLabel, PLabel, 
ILabel, DLabel,DRLabel, DRCurrent;
controlP5.Textfield SPField, InField, OutField, 
PField, IField, DField;

PrintWriter output;
PFont AxisFont, TitleFont; 

//ADDED BY MARUTHA>>

controlP5.Textfield MyField1;
controlP5.Textfield MyField2;
controlP5.Textfield MyField3;
controlP5.Textfield MyField4;



int gCountStep=0;
int count=0;

controlP5.Textfield SPFieldArray[] = new controlP5.Textfield[10];
controlP5.Textfield ATFieldArray[] = new controlP5.Textfield[10];
controlP5.Textfield STFieldArray[] = new controlP5.Textfield[10];

controlP5.Textarea MyTextarea;


controlP5.Button ContinousStartButton;
controlP5.Button PauseButton;
controlP5.Button PlayButton;

//<<ADDED BY MARUTHA

void setup()
{
  frameRate(30);
  //size(windowWidth , windowHeight);
  fullScreen();

  //println(Serial.list());                                           // * Initialize Serial
  //myPort = new Serial(this, Serial.list()[0], 9600);                //   Communication with
  //myPort.bufferUntil(10);   //   the Arduino
  

  controlP5 = new ControlP5(this);                                  // * Initialize the various
  
  SPField= controlP5.addTextfield("Setpoint",10,100,60,20);         //   Buttons, Labels, and
  InField = controlP5.addTextfield("Input",10,150,60,20);           //   Text Fields we'll be
  OutField = controlP5.addTextfield("Acctime",10,200,60,20);         //   using
  PField = controlP5.addTextfield("Kp (Proportional)",10,275,60,20);          //
  IField = controlP5.addTextfield("Ki (Integral)",10,325,60,20);          //
  DField = controlP5.addTextfield("Kd (Derivative)",10,375,60,20);          //
  AMButton = controlP5.addButton("Toggle_AM",0.0,10,50,60,20);      //
  AMLabel = controlP5.addTextlabel("AM","Manual",12,72);            //
  AMCurrent = controlP5.addTextlabel("AMCurrent","Manual",80,65);   //
  controlP5.addButton("Send_To_Arduino",0.0,10,475,120,20);         //
  SPLabel=controlP5.addTextlabel("SP","3",80,103);                  //
  InLabel=controlP5.addTextlabel("In","1",80,153);                  //
  OutLabel=controlP5.addTextlabel("Out","2",80,203);                //
  PLabel=controlP5.addTextlabel("P","4",80,278);                    //
  ILabel=controlP5.addTextlabel("I","5",80,328);                    //
  DLabel=controlP5.addTextlabel("D","6",80,378);                    //
  DRButton = controlP5.addButton("Toggle_DR",0.0,10,425,60,20);      //
  DRLabel = controlP5.addTextlabel("DR","Direct",12,447);            //
  DRCurrent = controlP5.addTextlabel("DRCurrent","Direct",80,440);   //
 PauseButton = controlP5.addButton("Pause",0.0,1000,720,50,20);
 PlayButton = controlP5.addButton("Play",0.0,900,720,50,20);
  
//ADDED BY MARUTHA>>
  MyField1 = controlP5.addTextfield("Main",300,600,400,20);
  MyField2 = controlP5.addTextfield("AccTime",10,660,100,20);
  MyField3 = controlP5.addTextfield("SteadyTime",10,700,100,20);

/* TEXT AREA*/

  MyTextarea = new controlP5.Textarea(controlP5,"Log");
  //MyTextarea.setSize(300,700);
  //MyTextarea.setPosition(400,800);
  
  MyTextarea =  controlP5.addTextarea("Log","",150,600,900,100);
  MyTextarea.setColorBackground(0);
  MyTextarea.setScrollActive(1);
  MyTextarea.showScrollbar();
  
    for(int i=0;i<=9;i++){
    SPFieldArray[i] = controlP5.addTextfield("Setpoint"+i,1100,100+(i*50),60,20);         //   Set Point For 1st loop 
    ATFieldArray[i] = controlP5.addTextfield("Acctime"+i,1200,100+(i*50),60,20);           //   Acceleration Time For 1st loop
    STFieldArray[i] = controlP5.addTextfield("Steadytime"+i,1300,100+(i*50),60,20);         //   Steady Time For 1st loop
    SPFieldArray[i].setValue(""+0);
    ATFieldArray[i].setValue(""+0);
    STFieldArray[i].setValue(""+0);
  }
  
  ContinousStartButton = controlP5.addButton("Start_Continous",0.0,1100,650,80,20); 
  
  ports = controlP5.addDropdownList("list-1",1075,25,100,84);
  customize(ports); 
  
//<<ADDED BY MARUTHA
  
  AxisFont = loadFont("axis.vlw");
  TitleFont = loadFont("Titles.vlw");
  //customize(ports);
 
  nextRefresh=millis();
  if (outputFileName!="") output = createWriter(outputFileName);
}



//here we setup the dropdown list.
void customize(DropdownList ddl) {
  ddl.setBackgroundColor(color(200));
  
  ddl.setItemHeight(20);
  ddl.setBarHeight(15);
  ddl.getCaptionLabel().set("Select COM port");
  ddl.getCaptionLabel().getStyle().marginTop = 3;
  ddl.getCaptionLabel().getStyle().marginLeft = 3;
  ddl.getValueLabel().getStyle().marginTop = 3;
  comList =  myPort.list();
  String comlist = join(comList, ",");
  String COMlist = comList[0];
  int size2 = COMlist.length();
  
  int size1 = comlist.length() / size2;
 
  for(int i=0; i< size1; i++)
  {
    
    ddl.addItem(comList[i],i);
  }
  println("com port :" + size1);
  ddl.setColorBackground(color(60));
  ddl.setColorActive(color(255,128));
}




void draw()
{
  background(#9A9EC4);
  while(Comselected == true && serialSet == false)
  {
    
    startSerial(comList);
 }
  drawGraph();
  drawButtonArea();
  
  //ADDED BY MARUTHA>>
  count++;
  //<<ADDED BY MARUTHA
}
void controlEvent(ControlEvent theEvent) {
  if (theEvent.isGroup()) 
  {
    float S = theEvent.getGroup().getValue();
    println("event from group : "+theEvent.getGroup().getValue()+" from "+theEvent.getGroup());

    Ss = int(S);
    Comselected = true;
    //println(Ss);
  }
  else if (theEvent.isController()) {
    float S = theEvent.getController().getValue();
    println("event from controller : "+theEvent.getController().getValue()+" from "+theEvent.getController());
    Ss = int(S);
    Comselected = true;
  }
}

void startSerial(String[] theport)
{
  
  myPort = new Serial(this, theport[Ss], 9600);
  myPort.bufferUntil(10);
  serialSet = true;
  //println("my port :");
}

//ADDED BY MARUTHA>>

void Start_Continous(){
  
  gCountStep = 0;
  Run_Continous();
  
}

void Run_Continous(){
  
  
  //CODE TO SEND TO ARDINO>>
  
  
    float[] toSend = new float[6];
    
    if(gCountStep < 10){
      toSend[0] = float(SPFieldArray[gCountStep].getText());
      toSend[1] = float(InField.getText());
      toSend[2] = float(ATFieldArray[gCountStep].getText());
      toSend[3] = float(PField.getText());
      toSend[4] = float(IField.getText());
      toSend[5] = float(DField.getText());
      Byte a = (AMLabel.getValueLabel().getText()=="Manual")?(byte)0:(byte)1;
      Byte d = (DRLabel.getValueLabel().getText()=="Direct")?(byte)0:(byte)1;
      myPort.write(a);
      myPort.write(d);
      myPort.write(floatArrayToByteArray(toSend));
      justSent=true;
    }
  
  //<<CODE TO SEND TO ARDINO
  
  MyField1.setValue("Step "+gCountStep+" Count:"+count+" RPM:"+toSend[0]+" AT:"+toSend[2]);
  
  if(gCountStep < 10){
    int accTime=int(ATFieldArray[gCountStep].getText()); 
    
    AccTimerTask tasknew = new AccTimerTask();
    tasknew.setStep(gCountStep);
    Timer timer = new Timer();
    timer.schedule(tasknew,accTime*1000);
    
  } else gCountStep= 0;
  
}

class AccTimerTask extends TimerTask{
  
  int Step;
  
  void setStep(int vStep){
     Step = vStep; 
  }
  
  void run(){
   
   MyField2.setValue("Acc "+int(millis()/1000));
   int steadyTime=int(STFieldArray[gCountStep].getText()); 
   
   SteadyTimerTask tasknew = new SteadyTimerTask();
   tasknew.setStep(Step);
   Timer sTimer = new Timer();
      
   sTimer.schedule(tasknew,steadyTime*1000);
   
  }
  
}

class SteadyTimerTask extends TimerTask{
  
  int Step;
  
  void setStep(int vStep){
     Step = vStep; 
  }
  
  void run(){
    MyField3.setValue("Steady "+int(millis()/1000));
    gCountStep++;
    Run_Continous(); 
  }  
}

//<<ADDED BY MARUTHA

void drawGraph()
{
  //draw Base, gridlines
  stroke(#D1CF9A);
  fill(#BADBCB);
  rect(ioLeft, inputTop,ioWidth-1 , inputHeight);
  rect(ioLeft, outputTop, ioWidth-1, outputHeight);
  stroke(#212720);

  //Section Titles
  textFont(TitleFont);
  fill(255);
  text("PID Input / Setpoint",(int)ioLeft+10,(int)inputTop-5);
  text("PID Output",(int)ioLeft+10,(int)outputTop-5);


  //GridLines and Titles
  textFont(AxisFont);
  
  //horizontal grid lines
  int interval = (int)inputHeight/5;
  for(int i=0;i<6;i++)
  {
    if(i>0&&i<5) line(ioLeft+1,inputTop+i*interval,ioRight-2,inputTop+i*interval);
    text(str((InScaleMax-InScaleMin)/5*(float)(5-i)+InScaleMin),ioRight+5,inputTop+i*interval+4);

  }
  interval = (int)outputHeight/5;
  for(int i=0;i<6;i++)
  {
    if(i>0&&i<5) line(ioLeft+1,outputTop+i*interval,ioRight-2,outputTop+i*interval);
    text(str((OutScaleMax-OutScaleMin)/5*(float)(5-i)+OutScaleMin),ioRight+5,outputTop+i*interval+4);
  }


  //vertical grid lines and TimeStamps
  int elapsedTime = millis();
  interval = (int)ioWidth/vertCount;
  //println(ioWidth);
  //println(interval);  
  int shift = elapsedTime*(int)ioWidth / windowSpan;
  shift %=interval;

  int iTimeInterval = windowSpan/vertCount;
  float firstDisplay = (float)(iTimeInterval*(elapsedTime/iTimeInterval))/displayFactor;
 // float firstDisplay = (float)(elapsedTime)/displayFactor;
  float timeInterval = (float)(iTimeInterval)/displayFactor;
  for(int i=0;i<vertCount;i++)
  {
    int x = (int)ioRight-shift-2-i*interval;

    line(x,inputTop+1,x,inputTop+inputHeight-1);
    line(x,outputTop+1,x,outputTop+outputHeight-1);    

    float t = firstDisplay-(float)i*timeInterval;
    //float t = firstDisplay-(float)i*timeInterval;
    if(t>=0)  text(str(t),x,outputTop+outputHeight+10);
  }


  // add the latest data to the data Arrays.  the values need
  // to be massaged to get them to graph correctly.  they 
  // need to be scaled to fit where they're going, and 
  // because 0, 0 is the top left, we need to flip the values.
  // this is easier than having the user stand on their head
  // to read the graph.
  if(millis() > nextRefresh && madeContact)
  {
    nextRefresh += refreshRate;

    for(int i=nPoints-1;i>0;i--)
    {
      InputData[i]=InputData[i-1];
      SetpointData[i]=SetpointData[i-1];
      OutputData[i]=OutputData[i-1];
    }
    if (nPoints < arrayLength) nPoints++;

    InputData[0] = int(inputHeight)-int(inputHeight*(Input-InScaleMin)/(InScaleMax-InScaleMin));
    SetpointData[0] =int( inputHeight)-int(inputHeight*(Setpoint-InScaleMin)/(InScaleMax-InScaleMin));
    OutputData[0] = int(outputHeight)-int(outputHeight*(Output-OutScaleMin)/(OutScaleMax-OutScaleMin));
  }
  //draw lines for the input, setpoint, and output
  strokeWeight(2);
  for(int i=0; i<nPoints-2; i++)
  {
    int X1 = int(ioRight-2-float(i)*pointWidth);
    int X2 = int(ioRight-2-float(i+1)*pointWidth);
    boolean y1Above, y1Below, y2Above, y2Below;


    //DRAW THE INPUT
    boolean drawLine=true;
    stroke(255,0,0);
    int Y1 = InputData[i];
    int Y2 = InputData[i+1];

    y1Above = (Y1>inputHeight);                     // if both points are outside 
    y1Below = (Y1<0);                               // the min or max, don't draw the 
    y2Above = (Y2>inputHeight);                     // line.  if only one point is 
    y2Below = (Y2<0);                               // outside constrain it to the limit, 
    if(y1Above)                                     // and leave the other one untouched.
    {                                               //
      if(y2Above) drawLine=false;                   //
      else if(y2Below) {                            //
        Y1 = (int)inputHeight;                      //
        Y2 = 0;                                     //
      }                                             //
      else Y1 = (int)inputHeight;                   //
    }                                               //
    else if(y1Below)                                //
    {                                               //
      if(y2Below) drawLine=false;                   //
      else if(y2Above) {                            //
        Y1 = 0;                                     //
        Y2 = (int)inputHeight;                      //
      }                                             //
      else Y1 = 0;                                  //
    }                                               //
    else                                            //
    {                                               //
      if(y2Below) Y2 = 0;                           //
      else if(y2Above) Y2 = (int)inputHeight;       //
    }                                               //

    if(drawLine)
    {
      line(X1,Y1+inputTop, X2, Y2+inputTop);
    }

    //DRAW THE SETPOINT
    drawLine=true;
    stroke(0,255,0);
    Y1 = SetpointData[i];
    Y2 = SetpointData[i+1];

    y1Above = (Y1>(int)inputHeight);                // if both points are outside 
    y1Below = (Y1<0);                               // the min or max, don't draw the 
    y2Above = (Y2>(int)inputHeight);                // line.  if only one point is 
    y2Below = (Y2<0);                               // outside constrain it to the limit, 
    if(y1Above)                                     // and leave the other one untouched.
    {                                               //
      if(y2Above) drawLine=false;                   //
      else if(y2Below) {                            //
        Y1 = (int)(inputHeight);                    //
        Y2 = 0;                                     //
      }                                             //
      else Y1 = (int)(inputHeight);                 //
    }                                               //
    else if(y1Below)                                //
    {                                               //
      if(y2Below) drawLine=false;                   //
      else if(y2Above) {                            //
        Y1 = 0;                                     //
        Y2 = (int)(inputHeight);                    //
      }                                             //
      else Y1 = 0;                                  //
    }                                               //
    else                                            //
    {                                               //
      if(y2Below) Y2 = 0;                           //
      else if(y2Above) Y2 = (int)(inputHeight);     //
    }                                               //

    if(drawLine)
    {
      line(X1, Y1+inputTop, X2, Y2+inputTop);
    }

    //DRAW THE OUTPUT
    drawLine=true;
    stroke(0,0,255);
    Y1 = OutputData[i];
    Y2 = OutputData[i+1];

    y1Above = (Y1>outputHeight);                   // if both points are outside 
    y1Below = (Y1<0);                              // the min or max, don't draw the 
    y2Above = (Y2>outputHeight);                   // line.  if only one point is 
    y2Below = (Y2<0);                              // outside constrain it to the limit, 
    if(y1Above)                                    // and leave the other one untouched.
    {                                              //
      if(y2Above) drawLine=false;                  //
      else if(y2Below) {                           //
        Y1 = (int)outputHeight;                    //
        Y2 = 0;                                    //
      }                                            //
      else Y1 = (int)outputHeight;                 //
    }                                              //
    else if(y1Below)                               //
    {                                              //
      if(y2Below) drawLine=false;                  //
      else if(y2Above) {                           //
        Y1 = 0;                                    //
        Y2 = (int)outputHeight;                    //
      }                                            //  
      else Y1 = 0;                                 //
    }                                              //
    else                                           //
    {                                              //
      if(y2Below) Y2 = 0;                          //
      else if(y2Above) Y2 = (int)outputHeight;     //
    }                                              //

    if(drawLine)
    {
      line(X1, outputTop + Y1, X2, outputTop + Y2);
    }
  }
  strokeWeight(1);
}

void drawButtonArea()
{
  stroke(0);
  fill(100);
  rect(0, 0, ioLeft, windowHeight);
}

void Toggle_AM() {
  if(AMLabel.getValueLabel().getText()=="Manual") 
  {
    AMLabel.setValue("Automatic");
  }
  else
  {
    AMLabel.setValue("Manual");   
  }
}


void Toggle_DR() {
  if(DRLabel.getValueLabel().getText()=="Direct") 
  {
    DRLabel.setValue("Reverse");
  }
  else
  {
    DRLabel.setValue("Direct");   
  }
}
void Pause() {
 pause = true; 
}  
void Play() {
 pause = false; 
} 

// Sending Floating point values to the arduino
// is a huge pain.  if anyone knows an easier
// way please let know.  the way I'm doing it:
// - Take the 6 floats we need to send and
//   put them in a 6 member float array.
// - using the java ByteBuffer class, convert
//   that array to a 24 member byte array
// - send those bytes to the arduino
void Send_To_Arduino()
{
  float[] toSend = new float[6];

  toSend[0] = float(SPField.getText());
  toSend[1] = float(InField.getText());
  toSend[2] = float(OutField.getText());
  toSend[3] = float(PField.getText());
  toSend[4] = float(IField.getText());
  toSend[5] = float(DField.getText());
  Byte a = (AMLabel.getValueLabel().getText()=="Manual")?(byte)0:(byte)1;
  Byte d = (DRLabel.getValueLabel().getText()=="Direct")?(byte)0:(byte)1;
  myPort.write(a);
  myPort.write(d);
  myPort.write(floatArrayToByteArray(toSend));
  justSent=true;
} 


byte[] floatArrayToByteArray(float[] input)
{
  int len = 4*input.length;
  int index=0;
  byte[] b = new byte[4];
  byte[] out = new byte[len];
  ByteBuffer buf = ByteBuffer.wrap(b);
  for(int i=0;i<input.length;i++) 
  {
    buf.position(0);
    buf.putFloat(input[i]);
    for(int j=0;j<4;j++) out[j+i*4]=b[3-j];
  }
  return out;
}


//take the string the arduino sends us and parse it
void serialEvent(Serial myPort)
{
  String read = myPort.readStringUntil(10);
  
  if(outputFileName!="") output.print(str(millis())+ " "+read);
  
  String[] s = split(read, " ");

  if (s.length == 14)
  {
    Setpoint = float(s[1]);           // * pull the information
    Input = float(s[2]);              //   we need out of the
    Output = float(s[3]);    //   string and put it
    //feedoutput = float(s[9]);
    //feederror = float(s[10]);
    SPLabel.setValue(s[1]);           //   where it's needed
    InLabel.setValue(s[2]);           // 
    OutLabel.setValue(trim(s[3]));    //
    PLabel.setValue(trim(s[4]));      //
    ILabel.setValue(((s[5])));      // ki = Ki * sampleTime
    DLabel.setValue(trim(s[6]));      // Kd
      
    AMCurrent.setValue(trim(s[7]));   //
    DRCurrent.setValue(trim(s[8]));
    LogDisplay1 = new String(s[9]);   // Mapped output
    LogDisplay2 = new String(s[10]);  // Kp * (setpoint - feedback (input))
    LogDisplay3 = new String(s[11]);  // PID output >> before mapping
    LogDisplay4 = new String(s[12]);  // ITERM - sum of error
    LogDisplay5 = new String(s[13]);
    if (!pause)
    MyTextarea.append("\nPIDoutput (bMap): "+ LogDisplay3 + " MapedOutput: " + LogDisplay1 +" Kp * Error: " +LogDisplay2 +" Feedback: " + Input + " Setpoint: " + Setpoint+" KI*sampleTime : "+s[5]+" ITERM (Sum of err): "+LogDisplay4 +" dInput*KD/sampleTime: "+ (float(s[6]) *float(LogDisplay5)) );
    
   // MyField4.setText("output: "+ feedoutput + " error: "+ feederror); 
    //MyField4.setText("hi "); 
   // LogDisplay = (s[9]);
    
    if(justSent)                      // * if this is the first read
    {                                 //   since we sent values to 
      SPField.setText(trim(s[1]));    //   the arduino,  take the
      InField.setText(trim(s[2]));    //   current values and put
      OutField.setText(trim(s[3]));   //   them into the input fields
      PField.setText(trim(s[4]));     //
      IField.setText((s[5]));     //
      DField.setText(trim(s[6]));     //
      //MyField4.setText(s[9]);   // for logDisplay
      // mode = trim(s[7]);              //
      AMLabel.setValue(trim(s[7]));         //
      ////dr = trim(s[8]);                //
      DRCurrent.setValue(trim(s[8]));       //
      justSent=false;                 //
    }                                 //

    if(!madeContact) madeContact=true;
  }
  //String p = myPort.readStringUntil(10);
  ////LogDisplay = myPort.readString();
  //  String[] LogDisplay = split(p, "/n");
  //  if (LogDisplay.length == 3)
  //MyTextarea.setText(LogDisplay[1]); 
  
  // MyField4.setText(s[9]);
}