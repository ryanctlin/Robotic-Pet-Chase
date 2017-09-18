#include <Servo.h> //import servo control library
#include <AFMotor.h> //import motor control library

//++++++++++++++++++++++++++++++++++++++++++++++++++++HARDWARE SETUP+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Define motors as objects
AF_DCMotor Lmotor(2);
AF_DCMotor Rmotor(1);

Servo aServo; //Define servo as object
int servoPin = 10; //Set servo pin for control

double diff = 0.9; //differntial multiplier applied to right motor to compensate for torque difference

//set ultrasound pin
int soundPin = 9;

//set touchbutton pin
int touchPin = 3;

void setup() {
  //Start serial monitor for debugging
  Serial.begin(9600);
  Serial.println("Start");

  //turn on motor
  Lmotor.setSpeed(200);
  Rmotor.setSpeed(200);

  Lmotor.run(RELEASE);
  Rmotor.run(RELEASE);

  //set up ultrasound range finder
  pinMode(soundPin, OUTPUT);
  digitalWrite(soundPin, HIGH); // Trig pin is normally HIGH

  //attach servo on corresponding pin
  aServo.attach(servoPin);

  //set up touchbutton
  pinMode(touchPin, INPUT);
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++ULTRASONIC SENSOR FUNCTIONS+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//-----------------------Function to obtain a distance value from ultrasonic sensor-----------------------------------
float rawDistance(int pin)
{
  unsigned long time;
  unsigned long sizeofpulse;
  float range;

  pinMode(pin, OUTPUT); //return digital pin to OUTPUT mode after reading
  digitalWrite(pin, LOW);
  delayMicroseconds(25);
  digitalWrite(pin, HIGH); //Trig pin pulsed LOW for 25usec
  time = micros(); //record timer
  pinMode(pin, INPUT); //change pin to INPUT to read the echo pulse
  sizeofpulse = pulseIn(pin, LOW, 18000); //should be approx 150usec, timeout at 18msec
  time = micros() - time - sizeofpulse; // amount of time elapsed since we sent the trigger pulse and detect the echo pulse, then subtract the size of the echo pulse
  range = (time * 340.29 / 2 / 10000) - 3; // convert to distance in centimeters
  return range;
}

//---------------------Function to average ultrasonic sensor data for noise reduction-------------------------------
float distance()
{
  float average = 0; //reset variable to store average range
  for (int i = 0; i < 3; i++) { //loop 3 times to obtain an average reading
    average += rawDistance(soundPin);
  }
  average = average / 3; //compute average range
  return average;
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++MOTOR FUNCTIONS+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//--------------------Function to travel forward--------------------------------------
void Forward() {
  //Set both motors to forward
  Lmotor.run(FORWARD);
  Rmotor.run(FORWARD);

  Rmotor.setSpeed(200 * diff);
  Lmotor.setSpeed(200);

  Serial.println("Status: Forward");
}

//-------------------Function to implement left turn in the forward direction with radius defined by speedDiff------------------------------
void LTurnF(int speedDiff) {
  //Set both motors to forward
  Lmotor.run(FORWARD);
  Rmotor.run(FORWARD);

  //Set up speed differential for turn
  Rmotor.setSpeed(200 * diff);
  Lmotor.setSpeed(200 - speedDiff);

  Serial.println("Status: Forward LTurn");
}

//--------------------Function to implement right turn in the forward direction with radius defined by speedDiff------------------------------
void RTurnF(int speedDiff) {
  //Set both motors to forward
  Lmotor.run(FORWARD);
  Rmotor.run(FORWARD);

  //Set up speed differential for turn
  Lmotor.setSpeed(200);
  Rmotor.setSpeed(200 * diff - speedDiff);

  Serial.println("Status: Forward RTurn");
}

//--------------------Function to travel in reverse--------------------------------------
void Reverse() {
  //Set both motors to forward
  Lmotor.run(BACKWARD);
  Rmotor.run(BACKWARD);

  Rmotor.setSpeed(200 * diff);
  Lmotor.setSpeed(200);

  Serial.println("Status: Reverse");
}

//----------------------Function to implement left turn in the reverse direction with radius defined by speedDiff----------------------------------
void LTurnR(int speedDiff) {
  //Set both motors to reverse
  Lmotor.run(BACKWARD);
  Rmotor.run(BACKWARD);

  //Set up speed differential for turn
  Rmotor.setSpeed(200 * diff);
  Lmotor.setSpeed(200 - speedDiff);

  Serial.println("Status: Reverse LTurn");
}

//--------------------------Function to implement right turn in the reverse direction with radius defined by speedDiff---------------------------------
void RTurnR(int speedDiff) {
  //Set both motors to reverse
  Lmotor.run(BACKWARD);
  Rmotor.run(BACKWARD);

  //Set up speed differential for turn
  Lmotor.setSpeed(200);
  Rmotor.setSpeed(200 * diff - speedDiff);

  Serial.println("Status: Reverse RTurn");
}

//--------------------------------Function to stop car-------------------------------------------------------------------------
void Stop() {
  //Set both motors to release
  Lmotor.run(RELEASE);
  Rmotor.run(RELEASE);

  Serial.println("Status: Stop");
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++BUTTON FUNCTIONS+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//------------------------Function to check if button is pressed------------------------------------------
int checkButton() {
  int pressedB = 0; //reset variable to store status of whether pressed or not

  if (digitalRead(touchPin) == HIGH) { //button is not pressed
    pressedB = 0; //set variable to off
    Serial.println("Not Pressed");
  }
  else if (digitalRead(touchPin) == LOW) { //button is pressed
    pressedB = 1; //set variable to on
    Serial.println("Pressed");
  }
  return pressedB; //return status variable
}


//++++++++++++++++++++++++++++++++++++++++++++++++PET CHASE FUNCTIONS++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int level = 0; //Variable to store current difficulty level
int caught = 0; //varible to check whether the pet has been caught
int pressed; //variable to store button status
double check; //variable to store current range in front of sensor

//------------------------Function to evade obstacles------------------------------------------
void evade() {
  int currentCycle = 0; //counter for number of while loop iterations
  aServo.write(180); //turn sensor to the right
  check = distance(); //variable to store current distance in front of sensor
  if (check >= 30) { // if there are no obstacles to the right
    while (currentCycle < 110 && caught == 0) { //iterate until specified turn is achieved or button is pressed
      //turn right
      RTurnF(135);
      delay(10);
      currentCycle++; //increment current iteration counter
      caught = checkButton(); //check if caught
    }
  }

  else { //if there are obstacles to the right
    aServo.write(0); //turn sensor to the left
    check = distance(); //update reading
    if (check >= 30) { // if there are no obstacles to the left
      while (currentCycle < 100 && caught == 0) { //iterate until specified turn is achieved or button is pressed
        //turn left
        LTurnF(150);
        delay(10);
        currentCycle++; //increment current iteration counter
        caught = checkButton(); //check if caught
      }
    }
    else { //if there are obstacles to the left as well
      while (currentCycle < 100 && caught == 0) { //iterate until specified reverse is achieved or button is pressed
        //turn left
        Reverse();
        delay(10);
        currentCycle++; //increment current iteration counter
        caught = checkButton(); //check if caught
      }
    }
  }
}

//------------------------Function to give chase------------------------------------------
void chase() {
  int action = random(1, 7); //generate random number to pick action
  int turnFactor = random(100, 200); //generate random number to control radius for turns
  double difficultyFactor[] = {1, 0.8, 0.65, 0.57, 0.49, 0.42, 0.39, 0.34, 0.30};
  int duration = (random(2000, 5000) * difficultyFactor[level - 1]); //generate random number for duration of each action, multiply by element in difficultyFactor array corresponding to current level
  
  Serial.println(action);
  
  //Selection of actions
  double loopCycle; //variable to store number of while loop iterations to achieve desired duration
  int currentCycle = 0; //variable to store current number of while loop iterations
  if (action == 1) {
    loopCycle = (duration * 1.5) / 10; //multiply duration by 1.5 to slightly increase forward duration compared to turns, divide by 10 to compute number of 10ms while loop iterations required
    Serial.println(loopCycle);
    while (currentCycle < loopCycle && caught == 0) { //iterate until specified duration is achieved or button is pressed
      currentCycle++; //increment current iteration counter
      Forward();
      delay(10);
      caught = checkButton(); //check if pet has been caught
    }
  }
  else if (action == 2) {
    loopCycle = duration / 10; //divide duration by 10 to compute number of 10ms while loop iterations required
    while (currentCycle < loopCycle && caught == 0) { //iterate until specified duration is achieved or button is pressed
      currentCycle++; //increment current iteration counter
      LTurnF(turnFactor);
      delay(10);
      caught = checkButton(); //check if pet has been caught
    }
  }
  else if (action == 3) {
    loopCycle = duration / 10; //divide duration by 10 to compute number of 10ms while loop iterations required
    while (currentCycle < loopCycle && caught == 0) { //iterate until specified duration is achieved or button is pressed
      currentCycle++; //increment current iteration counter
      RTurnF(turnFactor);
      delay(10);
      caught = checkButton(); //check if pet has been caught
    }
  }
  else if (action == 4) {
    loopCycle = (duration * 1.5) / 10; //multiply duration by 1.5 to slightly increase reverse duration compared to turns, divide by 10 to compute number of 10ms while loop iterations required
    while (currentCycle < loopCycle && caught == 0) { //iterate until specified duration is achieved or button is pressed
      currentCycle++; //increment current iteration counter
      Reverse();
      delay(10);
      caught = checkButton(); //check if pet has been caught
    }
  }
  else if (action == 5) {
    loopCycle = duration / 10; //divide duration by 10 to compute number of 10ms while loop iterations required
    while (currentCycle < loopCycle && caught == 0) { //iterate until specified duration is achieved or button is pressed
      currentCycle++; //increment current iteration counter
      LTurnR(turnFactor);
      delay(10);
      caught = checkButton(); //check if pet has been caught
    }
  }
  else if (action == 6) {
    loopCycle = duration / 10; //divide duration by 10 to compute number of 10ms while loop iterations required
    while (currentCycle < loopCycle && caught == 0) { //iterate until specified duration is achieved or button is pressed
      currentCycle++; //increment current iteration counter
      RTurnR(turnFactor);
      delay(10);
      caught = checkButton(); //check if pet has been caught
    }
  }
  else if (action == 7) {
    loopCycle = (duration * 0.3) / 10; //multiply duration by 0.3 to slightly decrease stop duration compared to turns, divide by 10 to compute number of 10ms while loop iterations required
    while (currentCycle < loopCycle && caught == 0) { //iterate until specified duration is achieved or button is pressed
      currentCycle++; //increment current iteration counter
      Stop();
      delay(10);
      caught = checkButton(); //check if pet has been caught
    }
  }
}

//------------------------------------------Main program------------------------------------------
void loop() {
  Serial.println(level); //for debugging purposes
  aServo.write(90); //turn sensor forward
  pressed = checkButton(); //check button status and store in variable pressed
  if (caught == 1) { //regardless of current button status, check if the previously executed function exited due to being caught
    pressed = 1; //set pressed to 1 to trigger correct response
    caught = 0; //reset caught variable
  }

  if ((pressed == 0) && (level == 0)) //if button is not pressed and the user has not yet started the game
  {
    Serial.println("Press button to start"); //prompt user to start
    delay(100); //idle until button is hit
  }
  else if ((pressed == 1) && (level == 0)) { //if button is pressed and the user has not yet started the game
    Serial.println("Start game");
    level = 1; //start game at level 1
    delay(2000); //delay before starting
  }
  else if ((pressed == 0) && (level >= 1)) { //if button is not pressed and the user has started the game
    check = distance();
    if (check < 30) { //check if there is an obstacle in front
      Serial.println("Evasive maneuvers");
      evade(); //call evade function to dodge
    }
    else { //if no obstacles
      Serial.println("Catch me!");
      chase(); //execute randomly generated action with a time duration inversely proportional to the current difficulty level
    }
  }
  else if ((pressed == 1) && (level >= 1)) { //if button is pressed and the user has started the game
    Stop(); //stop car
    Serial.println("You caught me!");
    if (level < 9) { //cap difficulty level at 9
      level++; //increase difficulty level
    }
    else if (level == 9) { //reset difficulty level
      level = 1;
    }
    Serial.println("Press button again to continue"); //prompt user to press button again to restart chase
    delay(500); //delay to prevent continuous button pressing
    
    //idle until button is pressed again, use do/while so that button status is updated at least once
    do {
      pressed = checkButton(); //update button status
      delay(50);
    }
    while (pressed == 0);
    
    delay(1000);
  }
}
