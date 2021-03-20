
boolean btFlag = false;
 
 void FlagWave()
 {
  btFlag = true;
  
  
 }


 long DegreesToDutyCycle(int deg) 
 {
  const long minDutyCycle = 1850;            // duty cycle for 0 degrees
  const long maxDutyCycle = 8050;            // duty cycle for 180 degrees

  long dutyCycle = map(deg, 0, 180, minDutyCycle, maxDutyCycle);  // convert to duty cycle

  return dutyCycle;
 }
