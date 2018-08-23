

              #if ARDUINO >= 100
                #include "Arduino.h"
              #else
                #include "WProgram.h"
              #endif

              #include <PID_v1.h>
              

              double astamp = 0, stamp = 0,Timetaken =0,tempOutput =0,count =0,speed = 0, t = 1.5,lastkp = 0 ,lastki = 0,lastkd = 0;
              double lastOutput=0,  lastInput=0,stepOutput=0,output=0, lastoutput =0,lastsetpoint =0, minsetpoint= 500, maxsetpoint = 12000; //edit minsetpoint= 500
              bool ptime= false , max, min, itime = true;
              bool itime1 = false, rampup = true,steady = false;
              unsigned long plookout = 0,  plookin = 0, savier1 = 0,savier2 = 0;

              /*Constructor (...)*********************************************************
               *    The parameters specified here are those for for which we can't set up 
               *    reliable defaults, so we need to have the user set them.
               ***************************************************************************/
              PID::PID(double* Input, double* Output, double* Setpoint,
                      double Kp, double Ki, double Kd, double acctime,double LogOutput, int ControllerDirection)
              {
                
                  myOutput = Output;
                  myInput = Input;
                  mySetpoint = Setpoint;
                  plookout = millis();

                  //Acctime = acctime;
                inAuto = true; 
                itime1 = false;
                ptime= false;
                itime = true ;
                //PID::SetInputLimits(0,4860);
                PID::SetOutputLimits(0,12450);     //default output limit corresponds to // added max limit => 12450

                                      //the arduino pwm limits

                  SampleTime = 1;             //default Controller Sample Time is 0.1 seconds

                  PID::SetControllerDirection(ControllerDirection);
                  PID::SetTunings(Kp, Ki, Kd, acctime, LogOutput);
                //PID::SetSampleTime(NewSampleTime);

                  lastTime = millis()-SampleTime;     
              }
               /*long PID::averageInput(int times)
              {
                long sum = 0;
                for (int i = 0; i < times; i++)
                {
                  sum += (*myInput);
                }

                return (sum / times);
              }
               */
              /* Compute() **********************************************************************
               *     This, as they say, is where the magic happens.  this function should be called
               *   every time "void loop()" executes.  the function will decide for itself whether a new
               *   pid Output needs to be computed.  returns true when the output is computed,
               *   false when nothing has been done.
               **********************************************************************************/ 
              bool PID::Compute()
              {

                    // Serial.print("Initial ITerm :");
                    // Serial.println(ITerm);
                    // Serial.println();
                    // Serial.println();

                 if(!inAuto) return false;
                 unsigned long now = millis();
                 unsigned long timeChange = (now - lastTime);
                 if(timeChange>=SampleTime)
                 {
                    /*Compute all the working error variables*/
                  //double input = averageInput();
                  
                    double input = *myInput, setpoint = *mySetpoint;
                   
                    double error = setpoint - input;
                    double Nerror = input - setpoint;
                     
                    if(setpoint == 0){ 
                      ITerm = 0;
                      itime1 = false;
                    }
                    //ITerm += error;
                   /*if(ITerm >(10*outMax )) {
                      //ki = 0.9*ki;
                      //ki = ki;
                      ITerm= 10 *outMax;
                    }*/
                    //if (ITerm < outMin) ITerm= outMin;

                    t = 1000 + (Acctime*1000);
                    speed = (error)/t ;

                    dInput = (input -  lastInput);
                    //kd= ki+ SampleTime*speed/(input+1)*1000000;
                    //ki= ki+ SampleTime*speed/(input+1)*1000000000;
                    //if (ki < 0.00009) ki= 0.00003;
                   // if (ki > 0.009) ki= 0.009;
                   // if (kd < 0.9) kd= 0.9;
                   // if (kd > 6.9) kd= 6.9;

                    /*if (input < (setpoint -50) && rampup) {
                      plookout = millis();
                      rampup = true;
                    }
                    else if(input > setpoint - 100) rampup = true;
*/
                     
                     //t = 1500 + (Acctime*1000);
                     kp = lastkp;
                     ki = lastki;
                     kd = lastkd;
                     //rampup = true;


                    if (Nerror > minsetpoint || error > minsetpoint) {
                      ptime = true;
                      itime = false;
//kp += (SampleTime*speed) / ((input+1)*50);

                    }
                    else  ptime = false; 
                    /*if (max && min){
                      ptime = false;
                      max = false;
                      min = false;
                    } 
                    else ptime = true;*/
                   //if (plookin > t) plookin = 0;
                  // if (!itime1) plookin = 1001;
                      
                    if (ptime ) {                                     // ptime is time in which the p control happens
                      
                      //plookin = (millis() - plookout);
                      //if (error > 0) speed = error/t ;
                      //else speed = 0;
                      //outMax = 12000;
                      ITerm  += (error*timeChange)/1000;

                       speed = (error)/t ;
                       /*if(!itime1){
                        //plookin = 200;
                      ki += (SampleTime*speed*plookin) / ((input+1)*80000000);
                      itime1 = true;
                    }
*/                    //rampup = true;
                      // kp = 0.31; ki = 0.0005; kd = 0.95;
                     
                    //if (error > 0){
                      //kp = 0.32; ki = 0.00046; kd = 0.98;
                      if (kp > 0.0003)
                        kp += (SampleTime*speed) / ((input+1)*30);  //edit kp += (SampleTime*speed) / ((input+1)*30);
                      if (kd >0.0095)
                        kd += (SampleTime*speed) / ((input+1)*30); //edit kd += (SampleTime*speed) / ((input+1)*30);
                        if(!itime1){
                        ki += (SampleTime*speed) / ((input + 1)); //edit ki += (SampleTime*speed) / ((input + 1));
                        itime1 = true;
                       }
                         if (ki < 5 && ki > 0.004) ki += (SampleTime*speed) / ((input + 1)*10);

                   // }
                   /* else{
                      kp += (SampleTime*speed) / ((input+1)*10000);
                        kd += SampleTime*speed /(maxsetpoint*1000); 
                        ki += SampleTime*speed/((input + 1)*5000000); 

                    } */


                     //ptime = false;
                    }
                    else if ( !ptime && ((error > 20 && error < minsetpoint) || (Nerror > 20 && Nerror < minsetpoint)) && (!itime) ){
                      //if (error > 0) speed = error/t ;
                      // else speed = 0;
                      //kp = 0.29; ki = 0.0005; kd = 0.91;
                      //savier1 = 0;
                      minsetpoint = 500;  //edit minsetpoint = 500;
                     // kp = lastkp ;
                     // ki = lastki ;
                     // kd = lastkd;
                      //if (error > 90)
                      ITerm  += (error*timeChange)/100;
                      speed = (error)/t ;
                      //outMax = 14000;
                     //if(error > 0) {
                      //kp = 0.33; ki = 0.0005; kd = 0.95;
                     if(kp > 0.0003 && error <0)  
                     kp += (SampleTime*speed) / ((input+1)*100);

                      if(ki > 0.0004 && ki < 5)
                      ki += SampleTime*speed/((input+1)*10); // edit ki += SampleTime*speed/((input+1)*10);
                       //}
                     /*else
                      ki += SampleTime*speed/(maxsetpoint*1500000);*/

                     //kd += SampleTime*speed /(maxsetpoint*10000); 

                      //kp += (SampleTime*speed*plookin) / ((input+1)*80000000);
                      itime1 = false;
                      ptime = false;
                      //rampup = true;
                      //savier1 = 100000;
                    }

                    else if ( !ptime && ((error < 20 && error >=5) || (Nerror < 20 && Nerror >=5)) && !itime1 ){
                    speed = (error)/t ;
                     //if (error > 20)
                     ITerm  += (error*timeChange)/100 ; 

                      //kp = lastkp ;
                      //ki = (lastki) ;
                     // kd = lastkd;
                     if (kp>0.0003 && error < 0)
                      //ki += SampleTime*speed/((input+1)*5000000);
                      kp += (SampleTime*speed)*10 / ((input+1)*100);  //edit kp += (SampleTime*speed)*10 / ((input+1)*100);
                  if(ki > 0.0004 && ki < 5)
                      ki +=  SampleTime*speed/((input+1)*300);  // edit ki +=  SampleTime*speed/((input+1)*300);
                    if (kd>0.0095)

                      kd += (SampleTime*speed) /(input+1); // edit kd += (SampleTime*speed) /((input+1)*1);
                      //else kd += SampleTime*speed /(maxsetpoint*200000);           // added 100 to compensate for the sample time
                      ptime = false;
                      itime = true;
                      savier1 = 0;
                      //rampup = true;
                      //savier2 = 10000000;
                    }  

                   /*else if ( !ptime && ((error < 20 && error >=10) || (Nerror < 20 && Nerror >=10)) && !itime1 ){
                    speed = (error)/t ;
                     //if (error > 20)
                     ITerm  += (error*timeChange)/100 ; 

                      //kp = lastkp ;
                      //ki = (lastki) ;
                     // kd = lastkd;
                      if (kp>0.0003 && error < 0)
                      //ki += SampleTime*speed/((input+1)*5000000);
                      kp += (SampleTime*speed)*10 / ((input+1)*100);  //edit kp += (SampleTime*speed)*10 / ((input+1)*100);
                  if(ki > 0.0004 && ki < 5)
                      ki +=  SampleTime*speed/((input+1)*300);  // edit ki +=  SampleTime*speed/((input+1)*300);
                    if (kd>0.0095)

                      kd += (SampleTime*speed*19) /((input+1)*20); // edit kd += (SampleTime*speed) /((input+1)*1);
                      //else kd += SampleTime*speed /(maxsetpoint*200000);           // added 100 to compensate for the sample time
                      ptime = false;
                      itime = true;
                      savier1 = 0;
                      //rampup = true;
                      //savier2 = 10000000;
                    }  

                     else if ( !ptime && ((error < 10 && error >=5) || (Nerror < 10 && Nerror >=5)) && !itime1 ){
                    speed = (error)/t ;
                     //if (error > 20)
                     ITerm  += (error*timeChange)/100 ; 

                      //kp = lastkp ;
                      //ki = (lastki) ;
                     // kd = lastkd;
                     if (kp>0.0003 && error < 0)
                      //ki += SampleTime*speed/((input+1)*5000000);
                      kp += (SampleTime*speed)*10 / ((input+1)*100);  //edit kp += (SampleTime*speed)*10 / ((input+1)*100);
                  if(ki > 0.0004 && ki < 5)
                      ki +=  SampleTime*speed/((input+1)*300);  // edit ki +=  SampleTime*speed/((input+1)*300);
                    if (kd>0.0095)

                      kd += (SampleTime*speed*9) /((input+1)*10); // edit kd += (SampleTime*speed) /((input+1)*1);
                      //else kd += SampleTime*speed /(maxsetpoint*200000);           // added 100 to compensate for the sample time
                      ptime = false;
                      itime = true;
                      savier1 = 0;
                      //rampup = true;
                      //savier2 = 10000000;
                    }  */
                    else {
                      //if(error > 10)
                      ITerm  = (ITerm)+ ((error*timeChange)/10);
                      if(error > minsetpoint)minsetpoint = error;
                      itime1 = true;
                      ptime = false;
                      itime = true;
                      //kp = lastkp ;
                        //ki = lastki ;
                        // = lastkd ;
                        //savier2 = 0;
                       //rampup =false; 
                       // error = error;
                       // dInput = dInput;
                      if(( dInput > 20 || (-1*dInput)>20)) kd += (SampleTime*speed) /((setpoint+1)*10); //edit if(( dInput > 20 || (-1*dInput)>20)) kd += (SampleTime*speed) /((setpoint+1)*10);
                    } 

                    /*   
                     if (setpoint < 3500){
                      //kp = 0;
                      //kd = 0;
                      dInput = 0;
                      error = 0;
                     } 
                     //if (ITerm < -1*outMax) ITerm = -1*outMax;*/

                    if (setpoint == 0) {
                       kp = 0.0009; ki = 0.001; kd = 0.095;
                    }


                    /*Compute PID Output*/
                    //double output = setpoint + kp * error + ki * ITerm - kd * dInput;
                   
                    //double output =  kp * error + ki * ITerm - kd * dInput;
                    
                    //double output =  setpoint + (ki * ITerm)/10 + (kp * (error ))/50 - ((kd * (dInput))/50);
                    //else if (setpoint <= 3499)

                    //if(rampup)
                    //if(error == 0)
                    //{
                    // output =  setpoint;  // edit ki/10 kp/10 kd/1 ;
                   // }
                    //else
                   // {
                     output =  setpoint + (kp * (error )/10) + ((ki * ITerm)/10) - ((kd *dInput));  // edit ki/10 kp/10 kd/1 
                   // }
                    // else {
                      //output = lastoutput;
                      //if(error > 10|| Nerror > 10) ptime = true;
                    //}
                     //output =    setpoint  ;

                    //double output =  kp * error ;
                    //double output =  kp * error + ki * ITerm;
                    lastkp = kp;
                    lastki = ki;
                    lastkd = kd;
                    dispKp = kp;   
                    dispKi = ki;
                    dispKd = kd; 



  /*20-Dec

                     if (setpoint > 5499 && setpoint < 6200){
                   outMax = 10750;
                   if (input > setpoint ) outMax +=10;
                      else outMax -= 10;
                   //kp = 0.4; 
                 }
                  else if (setpoint > 6200 && setpoint < 6999) {
                    outMax = 10550; 
                    if (input > setpoint ) outMax +=10;
                      else outMax -= 10;
                    //kp = 0.6;
                  }
                  else if (setpoint > 6999) {
                    outMax = 9500; 
                    if (input > setpoint ) outMax +=10;
                      else outMax -= 10;
                    //kp = 0.6;
                  }
                   else if (setpoint > 3499 && setpoint < 5499) {
                    outMax = 13350; //14500
                    if (input > setpoint ) outMax +=10;
                      else outMax -= 10;
                    //kp = 0.6;
                  }
                   else if (setpoint > 2000 && setpoint < 3499) { //
                    outMax = 15500; 
                    if (input > setpoint ) outMax +=10;
                      else outMax -= 10;
                    //kp = 0.6;
                  }

                  else{
                   outMax = 17050; //37250
                   // if (input > setpoint ) outMax +=1;
                   //    else outMax -= 1;
                   //kp = 0.1;
                 }
  20-Dec*/            
                    

                     //output = setpoint + kp * error;
                   
                     //output = setpoint; 
                    
                    // Serial.print("output:");
                    // Serial.print(output);
                    // Serial.print(" kp*error:"); 
                    // Serial.println(kp*error); 
                    //Serial.print(" ITerm :");
                    //Serial.print(ITerm);
                    //Serial.print(" kd * dInput :");
                    //Serial.println(kd*dInput);
                    //Serial.print(" dInput :");
                    //Serial.println(dInput);
                                       
                  /*  if   (lastoutput > output){
                      min = true; 
                    }
                    else if (output > lastoutput )
                      max = true;  */
                  if(astamp == 0)  {
                  stamp = millis();  // added&& (Acctime >=2)
                  steady = true;
                }

                    lastoutput = output;
                    logoutput = output ; 


                  if(output > outMax) {
                    //kp = 0.99*kp;
                    //kd = 1.01*kd;
                    output = outMax;
                  }
                  else if(output < outMin) output = outMin;

                  // output = map(output,outMin,outMax,1000,7000);
                  output = ((output - outMin) * (6100.0)/ (outMax - outMin))+ 1040.0; // edit output = ((output - outMin) * (6040.0)/ (outMax - outMin))+ 1040.0;

                  if (Acctime >= 2 ){ //ACCELERATION TIME IS GIVEN SO CALCULAE STEPS // added >=
                      Timetaken = millis() - stamp;
                      stamp = millis();
                      astamp += Timetaken/1000;
                      //astamp += 1;
                      //Serial.print("astamp ");
                     // Serial.println(astamp);
                     // Serial.print("Acctime ");
                     // Serial.println(Acctime);
                      //Serial.print("Timetaken");
                      //Serial.println(Timetaken/1000);
                      //stamp = astamp;
                      if(count == 1 ) {
                        stepOutput = tempOutput;
                        lastOutput = tempOutput;
                        count = 0;
                      }
                     if( astamp >= Acctime) //aCCELERATION time MET SO EXIT
                     {
                      Acctime = 0;
                      astamp = 0;
                      steady = true; 
                      //tempOutput = output;
                      stepOutput = output;
                      lastOutput = output;
                      count = 1;
                     }
                     else {
                      //output = constrain(output,lastOutput,outMax);
                      //stepOutput = stepOutput + ((output-lastOutput)*astamp)/(Acctime);
                       stepOutput = lastOutput + ((output-lastOutput)*astamp)/(Acctime);// change
                      /* Serial.print("stepOutput:"); 
                       Serial.print(stepOutput);
                       Serial.print(" output:");
                       Serial.print(output); 
                       Serial.print(" lastOutput:");
                       Serial.print(lastOutput);
                       Serial.print(" astamp:"); 
                       Serial.print(astamp); 
                       Serial.print(" Acctime");
                       Serial.println(Acctime);*/
                       tempOutput = stepOutput;
                       steady = false;  
                     }
                        //if (input > setpoint) tempOutput = output;
                        //Serial.println(output);
                        //Serial.print(" ");
                      
                   } else {  // brpt-1
                    lastOutput = output;
                    //stepOutput = output;
                    //stamp = millis();
                    //steady = true;  

                    tempOutput = output;
                   } 
                    //*myOutput = output;

                  *myOutput = tempOutput;
                  // Serial.print("tempOutput:");
                  //   Serial.println(tempOutput);

                  
                    /*Remember some variables for next time*/
                     lastInput = input;
                    lastTime = now;
                   lastsetpoint = setpoint;
                    
                    if (input > (setpoint))
                  return true;
                 }
                 else return false;
              }


              /* SetTunings(...)*************************************************************
               * This function allows the controller's dynamic performance to be adjusted. 
               * it's called automatically from the constructor, but tunings can also
               * be adjusted on the fly during normal operation
               ******************************************************************************/ 
              void PID::SetTunings(double Kp, double Ki, double Kd, double acctime, double LogOutput)
              {
                 if (Kp<0 || Ki<0 || Kd<0 || acctime <0) return;
               
                    Acctime = acctime, logoutput = LogOutput;
                 
                 double SampleTimeInSec = ((double)SampleTime)/1000;  
                 //kp = Kp;
                 //ptime = true; 
                 //ki = Ki * SampleTimeInSec;
                 //kd = Kd / SampleTimeInSec;
                //if (lastsetpoint != *mySetpoint) // its going down because of this
                 //ITerm = 8400;
                 /*lastkp = Kp;
                 lastki = Ki;
                 lastkd = Kd;*/
                 ITerm = 0;
                 

               if (*mySetpoint == 0){ // its going down because of this
                 ITerm = 0;
                 ki = 0;
                 kp = 0;
                 kd = 0;
                }

                 dispKi = ki;
                 dispKd = kd;
               
                if(controllerDirection ==REVERSE)
                 {
                    kp = (0 - kp);
                    ki = (0 - ki);
                    kd = (0 - kd);
                 }
              }
                
              /* SetSampleTime(...) *********************************************************
               * sets the period, in Milliseconds, at which the calculation is performed  
               ******************************************************************************/
              void PID::SetSampleTime(double NewSampleTime)
              {
                 if (NewSampleTime > 0)
                 {
                    double ratio  = (double)NewSampleTime
                                    / (double)SampleTime;
                    ki *= ratio;
                    kd /= ratio;
                    SampleTime = (unsigned long)NewSampleTime;
                 }
              }
               
              /* SetOutputLimits(...)****************************************************
               *     This function will be used far more often than SetInputLimits.  while
               *  the input to the controller will generally be in the 0-1023 range (which is
               *  the default already,)  the output will be a little different.  maybe they'll
               *  be doing a time window and will need 0-8000 or something.  or maybe they'll
               *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
               *  here.
               **************************************************************************/
              void PID::SetOutputLimits(double Min, double Max)
              {
                 //if(Min >= Max) return;
                 outMin = Min;
                 outMax = Max;
               
                 if(inAuto)
                 {
                   if(*myOutput > outMax) *myOutput = outMax;
                   else if(*myOutput < outMin) *myOutput = outMin;
                 
                   if(ITerm > outMax) ITerm= outMax;
                   else if(ITerm < outMin) ITerm= outMin;
                 }
              }
              void PID::SetInputLimits(double Min, double Max)
              {
                 if(Min >= Max) return;
                 inmin = Min;
                 inmax = Max;
               
                 if(inAuto)
                 {
                   if(*myInput > inmax) *myInput = inmax;
                   else if(*myInput < inmin) *myInput = inmin; 
                   
                 }
              }


              /* SetMode(...)****************************************************************
               * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
               * when the transition from manual to auto occurs, the controller is
               * automatically initialized
               ******************************************************************************/ 
              void PID::SetMode(int Mode)
              {
                  bool newAuto = (Mode == AUTOMATIC);
                  if(newAuto == !inAuto)
                  {  /*we just went from manual to auto*/
                      PID::Initialize();
                  }
                  inAuto = newAuto;
              }
               
              /* Initialize()****************************************************************
               *  does all the things that need to happen to ensure a bumpless transfer
               *  from manual to automatic mode.
               ******************************************************************************/ 
              void PID::Initialize()
              {
                 ITerm = *myOutput;
                 lastInput =*myInput;
                 if(ITerm > outMax) ITerm = outMax;
                 else if(ITerm < outMin) ITerm = outMin;
              }

              /* SetControllerDirection(...)*************************************************
               * The PID will either be connected to a DIRECT acting process (+Output leads 
               * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
               * know which one, because otherwise we may increase the output when we should
               * be decreasing.  This is called from the constructor.
               ******************************************************************************/
              void PID::SetControllerDirection(int Direction)
              {
                 if(inAuto && Direction !=controllerDirection)
                 {
                  kp = (0 - kp);
                    ki = (0 - ki);
                    kd = (0 - kd);
                 }   
                 controllerDirection = Direction;
              }


              /* Status Funcions*************************************************************
               * Just because you set the Kp=-1 doesn't mean it actually happened.  these
               * functions query the internal state of the PID.  they're here for display 
               * purposes.  this are the functions the PID Front-end uses for example
               ******************************************************************************/
              double PID::GetKp(){ return  dispKp; }
              double PID::GetKi(){ return  dispKi;}
              double PID::GetKd(){ return  dispKd;}
              double PID::Getacctime(){ return  Acctime;}
              double PID::GetLogOutput(){ return  logoutput;}
              bool PID::Getsteady(){ return  steady;}
              double PID::GetITerm(){ return  ITerm;}
              double PID::GetdInput(){ return  dInput;}
              int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
              int PID::GetDirection(){ return controllerDirection;}

