
#define ADX_ADC_REF_VOLT 5  //5V adc ref voltage
#define ADX_ADC_AMPLITUDE 1024 //max amplitude 1024
#define ADX_ADC_SENSE 0.25 //default adc sensitivity = 0.25 per g
#define ZERO_X  1.22 //accleration of X-AXIS is 0g, the voltage of X-AXIS is 1.22v
#define ZERO_Y  1.22 //
#define ZERO_Z  1.25 //

const int xpin = A1;
const int ypin = A2;
const int zpin = A3;
//const float thresholdVal = 3.5;

//float threshold = 4.1; //160

int trueSteps = 0;
boolean presumedSteps[10] = {false};
int presumedStepsCounter = 0;
boolean startedWalking = false;
boolean presumedFirstStep = false;
boolean checkingConsistency = false;
int consistencyCounter = 0;
boolean consistency[10] = {false};
float currentAcc = 0.0;
float previousAcc = 0.0;
float differenceThreshold = 0.4;

int tenSteps[10] = {0};
int16_t tenStepsCounter = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(xpin, INPUT);
  pinMode(ypin, INPUT);
  pinMode(zpin, INPUT);

  //get x, y, z
  int16_t x = analogRead(xpin);
  int16_t y = analogRead(ypin);
  int16_t z = analogRead(zpin);

  //print x,y,z
  //  Serial.print(x);
  //  Serial.print(",");
  //  Serial.print(y);
  //  Serial.print(",");
  //  Serial.print(z);
  //  Serial.println();

  //get voltage values
  float xv = (float)x / ADX_ADC_AMPLITUDE * ADX_ADC_REF_VOLT;
  float yv = (float)y / ADX_ADC_AMPLITUDE * ADX_ADC_REF_VOLT;
  float zv = (float)z / ADX_ADC_AMPLITUDE * ADX_ADC_REF_VOLT;

  //compute g values (acceleration)
  float ax = (xv - ZERO_X)/ADX_ADC_SENSE;
  float ay = (yv - ZERO_Y)/ADX_ADC_SENSE;
  float az = (zv - ZERO_Z)/ADX_ADC_SENSE;

  //compute total acceleration vector
  currentAcc = sqrt(ax*ax + ay*ay + az*az);
//  Serial.print("Difference Threshold is: ");
//  Serial.println(differenceThreshold);
  
}

void loop()
{ 
  //get x, y, z
  int16_t x = analogRead(xpin);
  int16_t y = analogRead(ypin);
  int16_t z = analogRead(zpin);

  //get voltage values
  float xv = (float)x / ADX_ADC_AMPLITUDE * ADX_ADC_REF_VOLT;
  float yv = (float)y / ADX_ADC_AMPLITUDE * ADX_ADC_REF_VOLT;
  float zv = (float)z / ADX_ADC_AMPLITUDE * ADX_ADC_REF_VOLT;

  //compute g values (acceleration)
  float ax = (xv - ZERO_X)/ADX_ADC_SENSE;
  float ay = (yv - ZERO_Y)/ADX_ADC_SENSE;
  float az = (zv - ZERO_Z)/ADX_ADC_SENSE;

  //compute total acceleration vector
  float totalAccVec = sqrt(ax*ax + ay*ay + az*az);
  previousAcc = currentAcc;
  currentAcc = totalAccVec;
  float currentDiff = currentAcc - previousAcc;
  currentDiff = sqrt(currentDiff * currentDiff); //get absolute value 

  //if yet to start walking and presumedStepsCounter is up to 10 steps
//  Serial.println(presumedStepsCounter);
  if(presumedStepsCounter == 10 && startedWalking == false){
    int realSteps = 0;
    for(int i=0; i<10; i++){
      realSteps += (presumedSteps[i] == true) ? 1 : 0;
      //reset buffer
      presumedSteps[i] = false;
    }
    if(realSteps > 4){ //check if there are up to 4 out of 10 that are greater than threshold
      //walking detected
      trueSteps += realSteps;
      startedWalking = true;
//      Serial.println("-----------------------------------------WALKING DETECTED .");
      differenceThreshold += 0.1; //increase the threshold
    } else {
      startedWalking = false; 
    }

    presumedStepsCounter = 0;
  }

  if(presumedFirstStep == true && startedWalking == false){
    presumedSteps[presumedStepsCounter] = (currentDiff > differenceThreshold) ? true : false;
    presumedStepsCounter++;
  }
  
  if((currentDiff > differenceThreshold) && (startedWalking == true) && (checkingConsistency == false)){
    //dont just increment, check consistency in next 10 steps
    checkingConsistency = true;
    consistency[0] = true;
//    Serial.print("-----------------------------------STEPS: ");
//    Serial.println(trueSteps);
  }

  //if started walking, checkingConsistency is true
  if(checkingConsistency == true){
    if(consistencyCounter < 9){ //if consistency counter is not up to ten, keep counting
      consistencyCounter++;
      consistency[consistencyCounter] = (currentDiff > differenceThreshold) ? true : false; //set which of 10 steps are consistent
    }
    
    if(consistencyCounter == 9){ //if consistencycounter is == 9, i.e 10th step
      //check how many steps were consistent
      int consistentSteps = 0;
      for(int i=0; i<10; i++){
        consistentSteps += (consistency[i] == true) ? 1 : 0;
        //reset consistency
        consistency[i] = 0;
      }
//      Serial.print(" ---------------------------CONSISTENT STEPS: ");
//      Serial.println(consistentSteps);
      //check if to add the number of consistent steps
      if(consistentSteps >= 3){ //there were at least 3 consistent steps
        trueSteps += consistentSteps; //add to true steps
      } else {
          //not walking  
          startedWalking = false; 
          differenceThreshold = differenceThreshold - 0.1; //decrease threshold to detect steps again
          presumedFirstStep = false; //wait to detect step again
      }
      checkingConsistency = false; //stop checking consistency
      consistencyCounter = 0; //reset consistency counter
    }
  }
  
  if(currentDiff > differenceThreshold){
    presumedFirstStep = true;
  }

  delay(200);
}
