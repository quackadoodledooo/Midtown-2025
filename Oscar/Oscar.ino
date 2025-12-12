#include "Constants.h"
#include "PID.h"

PID pivotPID(pivotKp, pivotKi, pivotKd, pivotMin, pivotMax);

NoU_Motor Drive1(4);
NoU_Servo Turn1(2, 500, 2500);
NoU_Motor Drive2(5);
NoU_Servo Turn2(1, 500, 2500);
NoU_Motor Drive3(8);
NoU_Servo Turn3(3, 500, 2500);
NoU_Motor Drive4(1);
NoU_Servo Turn4(4, 500, 2500);

NoU_Motor algae1(2);
NoU_Motor algae2(3);
NoU_Motor coral(6);
NoU_Motor pivot(7);

NoU_Servo elevatorLeft(5);
NoU_Servo elevatorRight(6);

//The gyroscope sensor is by default precise, but not accurate. This is fixable by adjusting the angular scale factor.
//Tuning procedure:
//Rotate the robot in place 5 times. Use the Serial printout to read the current gyro angle in Radians, we will call this "measured_angle".
//measured_angle should be nearly 31.416 which is 5*2*pi. Update measured_angle below to complete the tuning process.
int measured_angle = 27.562;
int angular_scale = (5.0 * 2.0 * PI) / measured_angle;

void setup() {
  NoU3.begin();
  STATE = START;
  PestoLink.begin("Midtown #40 Oscar");
  Serial.begin(115200);
  NoU3.calibrateIMUs();
  pivot.beginEncoder();
  Drive1.setInverted(true);
  Drive2.setInverted(true);
  pivot.setInverted(true);
  xTaskCreatePinnedToCore(taskUpdateSwerve, "taskUpdateSwerve", 4096, NULL, 2, NULL, 1);
  NoU3.setServiceLight(LIGHT_DISABLED);

  //pinMode(6, INPUT_PULLUP);
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 500);
  FastLED.setBrightness(20);
  FastLED.clear();
  FastLED.show();

  //Motor Configs
  Drive1.setMotorCurve(0.6, 1, 0, 2);
  Drive2.setMotorCurve(0.6, 1, 0, 2);
  Drive3.setMotorCurve(0.6, 1, 0, 2);
  Drive4.setMotorCurve(0.6, 1, 0, 2);
}

void moveWithPreset(int pivotHeight, int elevatorHeight, int servoDirection, int time, float speed) {
  Turn4.write(servoDirection);
  Turn3.write(servoDirection);
  Turn2.write(servoDirection);
  Turn1.write(servoDirection);
  servoGoal = elevatorHeight;
  delay(250);
  pivotGoal = pivotHeight;
  Drive4.set(speed);
  Drive3.set(speed);
  Drive2.set(speed);
  Drive1.set(speed);
  delay(time-250);
  Drive4.set(0);
  Drive3.set(0);
  Drive2.set(0);
  Drive1.set(0);
}

void leftL4Barge() {

}



void rightL4Barge() {

}



void centerL4Barge() {
<<<<<<< HEAD
  //move forward for 4 seconds while moving elevator and pivot to scoring position;
  moveWithPreset(pivotL4, servoL4, 90, 4000, -1);
  
  //score coral;
  coral.set(1);
  delay(1000);
  coral.set(0);
  
  //move backward at an angle while adjusting elevator to dealigify
  moveWithPreset(pivotAL3, servoAL2, 70, 2000, 1);

  // move forward while pulling in algae
  algae1.set(1);
  algae2.set(1);
  moveWithPreset(pivotAL3, servoAL3, 90, 500, -1);
  delay(2000);
  algae1.set(0.15);
  algae2.set(0.15);

  //move backward
  moveWithPreset(pivotSTOW, servoSTOW, 90, 1000, 1);

  //move right
  moveWithPreset(pivotBarge, servoBarge, 0, 1000, 1);

  // move backward 
  moveWithPreset(pivotBarge, servoBarge, 90, 1000, 1);

=======
  autoAngle = 0 * (PI/180);
  autoDrive = 1;
  servoGoal = servoL4;
  delay(100);
  pivotGoal = pivotL4;
  delay(1000);
  autoDrive = 0; 
  coral.set(1);
  delay(250);
  coral.set(0);
  autoDrive = -1;
  autoAngle = 10 * (PI/180);
  delay(700);
  autoDrive = 0;
  autoAngle = 0 * (PI/180);
  pivotGoal = pivotAL3;
  servoGoal = servoAL3;
  delay(200);
  autoDrive = 1;
  algae1.set(1);
  algae2.set(1);
  delay(1000);
  autoDrive = 0.25;
  algae1.set(0.15);
  algae2.set(0.15);
  delay(500);
  autoDrive = 0;
  autoAngle = 90 * (PI/180);
  delay(250);
  autoDrive = 1;
  pivotGoal = pivotBarge;
  servoGoal = servoBarge;
  delay(2000);
  autoDrive = 0;
  autoAngle = 0 * (PI/180);
  delay(250);
  autoDrive = -1; 
  delay(800);
  autoDrive = 0; 
>>>>>>> 782b6968de17b7189368238a43f63efda4850feb
  algae1.set(-1);
  algae2.set(-1);
  delay(500);
  algae1.set(0);
  algae2.set(0);
}



void leftL4Load() {
}



void rightL4Load() {
  
  //move forward for 3 seconds while moving elevator and pivot to scoring position;
  moveWithPreset(pivotL4, servoL4, 90, 3000, -1);

  //score coral;
  coral.set(1);
  delay(500);
  coral.set(0);
  
  //move backward while stowing pivot/elevator
  moveWithPreset(pivotSTOW, servoSTOW, 90, 2000, 1);

  //turn
  Turn4.write(135);
  Turn3.write(45);
  Turn2.write(135);
  Turn1.write(45);
  delay(250);
  Drive4.set(1);
  Drive3.set(-1);
  Drive2.set(1);
  Drive1.set(-1);
  delay(500);
  Drive4.set(0);
  Drive3.set(0);
  Drive2.set(0);
  Drive1.set(0);

  //move towards coral station with funnel pointing towards station
  Turn4.write(135);
  Turn3.write(135);
  Turn2.write(135);
  Turn1.write(135);
  delay(250);
  Drive4.set(1);
  Drive3.set(1);
  Drive2.set(1);
  Drive1.set(1);
  delay(5000);
  Drive4.set(0);
  Drive3.set(0);
  Drive2.set(0);
  Drive1.set(0);
}



void leave() {
  Drive4.set(1);
  Drive3.set(1);
  Drive2.set(1);
  Drive1.set(1);
  delay(2000);
  Drive4.set(0);
  Drive3.set(0);
  Drive2.set(0);
  Drive1.set(0);
}

/**
  AUTOS
  1 - Left one L4, load coral
  3 - Right one L4, load coral
  2 - One L4, algae in barge
  4 - Left one L4, barge, load coral
  5 - leave
  6 - Right one L4, barge, load coral 
  */

void runAuto() {
  if ((PestoLink.keyHeld(Key::Numpad1) || PestoLink.keyHeld(Key::Digit1) && !autoRan)) { //center, one l4, barge
    autoRan = true;
    leftL4Load();
  } else if ((PestoLink.keyHeld(Key::Numpad2) || PestoLink.keyHeld(Key::Digit2) && !autoRan)) { //left one l4, barge
    autoRan = true;
    centerL4Barge();
  } else if ((PestoLink.keyHeld(Key::Numpad3) || PestoLink.keyHeld(Key::Digit3) && !autoRan)) { //right one l4, load coral
    autoRan = true;
    rightL4Load();
  } else if ((PestoLink.keyHeld(Key::Numpad4) || PestoLink.keyHeld(Key::Digit4) && !autoRan)) { // left one l4, load coral
    autoRan = true;
    leftL4Barge();
  } else if ((PestoLink.keyHeld(Key::Numpad5) || PestoLink.keyHeld(Key::Digit5) && !autoRan)) { //leave
    autoRan = true;
    leave();
  } else if ((PestoLink.keyHeld(Key::Numpad6) || PestoLink.keyHeld(Key::Digit6) && !autoRan)) { //
    autoRan = true;
    rightL4Barge();
  }

}

void setLEDS() {
  if (STATE == START) {
    leds[0] = CRGB::Yellow;
  } else if (STATE == CORAL) {
    leds[0] = CRGB::White;
  } else if (STATE == ALGAE) {
    leds[0] = CRGB::Blue;
  }

}

void loop() {
  if(STATE == START){
    runAuto();
  }
  if(PestoLink.keyHeld(Key::B)) ESP.restart();

  FastLED.show();

  /**
  CONTROLS
  Up/Down arrows - Manual elevator control
  Left/Right arrows - Manual pivot control
  Left bumper - Switch modes
  Right bumper - Prepare to score
  CORAL MODE:
    A - L2
    B - L3
    Y - L4
    X - Stow
    Left trigger - Intake
    Right trigger - Outtake
  ALGAE MODE:
    Y - L2
    A - L3
    B - Barge
    Left trigger - intake
    Right trigger - outtake
  */

  if(PestoLink.keyHeld(Key::ArrowUp)) {
    servoGoal++;
  }
  if(PestoLink.keyHeld(Key::ArrowDown)) {
    servoGoal--;
  }
  if(PestoLink.keyHeld(Key::ArrowLeft)) {
    pivotGoal--;
  }
  if(PestoLink.keyHeld(Key::ArrowRight)) {
    pivotGoal++;
  }
  //switch modes
  if(PestoLink.buttonHeld(leftBumper)) {
    if(millis() - lastModeSwitch >= 400){
      if(STATE == CORAL) {
         STATE = ALGAE;
         lastModeSwitch = millis(); 
      }else{
         STATE = CORAL;
         lastModeSwitch = millis();
      } 
    }
  }

  if(PestoLink.buttonHeld(rightBumper)) { //Prepare to score
        servoGoal = servoReady;
        delay(500);
        pivotGoal = pivotReady;
  }

  if(STATE == CORAL) { // CORAL MODE PRESETS
   algae1.set(.15);
   algae2.set(.15);
    if(PestoLink.buttonHeld(buttonA)) { //L2
      if(servoGoal == servoSTOW && pivotGoal == pivotSTOW){
        servoGoal = servoReady;
        pivotGoal = pivotReady;
        delay(600);
        servoGoal = servoL2;
        pivotGoal = pivotL2;
      }else {
      servoGoal = servoL2;
      pivotGoal = pivotL2;
      }
    }
    if(PestoLink.buttonHeld(buttonB)) { //L3
      if(servoGoal == servoSTOW && pivotGoal == pivotSTOW){
        servoGoal = servoReady;
        pivotGoal = pivotReady;
        delay(600);
        servoGoal = servoL3;
        pivotGoal = pivotL3;
      }else {
      servoGoal = servoL3;
      pivotGoal = pivotL3;
      }
    }
    if(PestoLink.buttonHeld(buttonX)) { //Stow
      servoGoal = servoReady;
      pivotGoal = pivotReady;
      delay(1100);
      servoGoal = servoSTOW;
      pivotGoal = pivotSTOW;
    }
    if(PestoLink.buttonHeld(buttonY)) { //L4
      if(servoGoal == servoSTOW && pivotGoal == pivotSTOW){
        servoGoal = servoReady;
        pivotGoal = pivotReady;
        delay(600);
        servoGoal = servoL4;
        pivotGoal = pivotL4;
      }else {
      servoGoal = servoL4;
      pivotGoal = pivotL4;
      }
    }
    if(PestoLink.buttonHeld(downDPad)) { //L1
      if(servoGoal == servoSTOW && pivotGoal == pivotSTOW){
        servoGoal = servoReady;
        pivotGoal = pivotReady;
        delay(600);
        servoGoal = servoL1;
        pivotGoal = pivotL1;
      }else {
      servoGoal = servoL1;
      pivotGoal = pivotL1;
      }
    }

  } else if (STATE == ALGAE) { // ALGAE MODE PRESETS
    if(PestoLink.buttonHeld(buttonA)) { //L2 algae
      if(servoGoal == servoSTOW && pivotGoal == pivotSTOW){
        servoGoal = servoReady;
        pivotGoal = pivotReady;
        delay(600);
        servoGoal = servoAL2;
        pivotGoal = pivotAL2;
      }else {
      servoGoal = servoAL2;
      pivotGoal = pivotAL2;
      }
    }
    if(PestoLink.buttonHeld(buttonB)) { //L3 algae
      if(servoGoal == servoSTOW && pivotGoal == pivotSTOW){
        servoGoal = servoReady;
        pivotGoal = pivotReady;
        delay(600);
        servoGoal = servoAL3;
        pivotGoal = pivotAL3;
      }else {
      servoGoal = servoAL3;
      pivotGoal = pivotAL3;
      }
    }
    if(PestoLink.buttonHeld(buttonY)) { //Barge
      if(servoGoal == servoSTOW && pivotGoal == pivotSTOW){
        servoGoal = servoReady;
        pivotGoal = pivotReady;
        delay(600);
        servoGoal = servoBarge;
        pivotGoal = pivotBarge;
      }else {
      servoGoal = servoBarge;
      pivotGoal = pivotBarge;
      }
    }
    if(PestoLink.buttonHeld(buttonX)) { //Stow
      servoGoal = servoReady;
      pivotGoal = pivotReady;
      delay(800);
      servoGoal = servoSTOW;
      pivotGoal = pivotSTOW;
      }
  }
  /*
  if(pivotGoal>720) pivotGoal = 720;
  if(pivotGoal<0) pivotGoal = 0;
  if(servoGoal>150) servoGoal = 150;
  if(servoGoal<0) servoGoal = 0;
  */
  previousTime = currentTime; 
}

void taskUpdateSwerve(void* pvParameters) {
  while (true) {
  NoU3.updateServiceLight();
  heading = NoU3.yaw * angular_scale;
  roll = NoU3.roll * angular_scale;
  pitch = NoU3.pitch * angular_scale;
  currentTime = millis();
  pivotPosition = ((pivot.getPosition()/1793.103)*360);
  setLEDS();

  //start mode button
  if(PestoLink.keyHeld(Key::X)){
    STATE = START;
  }
  float batteryVoltage = NoU3.getBatteryVoltage();
  PestoLink.printBatteryVoltage(batteryVoltage);


    // Set up Gyro and its variables
    theta = NoU3.yaw*(180/PI) - headingOffset;

    // get magnitude and direction and assign to drivetrainVectors array, add offsets
    // set turn vector magnitude
    // set RSL based on whether a gamepad is connected
    // ANGLES IN RADIANS
    if (PestoLink.isConnected()) {

      driveAngle = autoActive ? (autoAngle) : (atan2(PestoLink.getAxis(1), PestoLink.getAxis(0)));
      driveMag = autoActive ? (autoDrive) : (sqrt(pow(PestoLink.getAxis(1), 2) + pow(PestoLink.getAxis(0), 2)));
      turnMag = autoActive ? (autoTurn) : (PestoLink.getAxis(2));

      drivetrainVectors[0][0] = driveMag;
      drivetrainVectors[0][1] = driveAngle + ((mod1Offset + theta) * (PI / 180));

      drivetrainVectors[1][0] = driveMag;
      drivetrainVectors[1][1] = driveAngle + ((mod2Offset + theta) * (PI / 180));

      drivetrainVectors[2][0] = driveMag;
      drivetrainVectors[2][1] = driveAngle + ((mod3Offset + theta) * (PI / 180));

      drivetrainVectors[3][0] = driveMag;
      drivetrainVectors[3][1] = driveAngle + ((mod4Offset + theta) * (PI / 180));

      NoU3.setServiceLight(LIGHT_ENABLED);
    } else {
      NoU3.setServiceLight(LIGHT_DISABLED);
    }

    //Heading Offset Control

    if (PestoLink.isConnected() && PestoLink.buttonHeld(10) && PestoLink.buttonHeld(11)) {
      headingOffset = NoU3.yaw*(180/PI);
    }



    //Vector Addition
    //Finds the component form of the current drive vector on the unit circle for each individual module
    //Uses the fact that turn vector is always 0 degrees, adds it to x coordinate.
    //reconverts back into magnitude and directon form
    //ANGLES IN DEGREES

    double xCord1 = drivetrainVectors[0][0] * cos(drivetrainVectors[0][1]) + turnMag;
    double yCord1 = drivetrainVectors[0][0] * sin(drivetrainVectors[0][1]);

    double xCord2 = drivetrainVectors[1][0] * cos(drivetrainVectors[1][1]) + turnMag;
    double yCord2 = drivetrainVectors[1][0] * sin(drivetrainVectors[1][1]);

    double xCord3 = drivetrainVectors[2][0] * cos(drivetrainVectors[2][1]) + turnMag;
    double yCord3 = drivetrainVectors[2][0] * sin(drivetrainVectors[2][1]);

    double xCord4 = drivetrainVectors[3][0] * cos(drivetrainVectors[3][1]) + turnMag;
    double yCord4 = drivetrainVectors[3][0] * sin(drivetrainVectors[3][1]);

    double cordArray[4] = { abs(xCord1), abs(xCord2), abs(xCord3), abs(xCord4) };

    //Find max x coordinate (since only adding to x)
    double massiveCord = 0.0;
    for (int i = 0; i < 3; i++) {
      if (cordArray[i] > massiveCord) {
        massiveCord = cordArray[i];
      }
    }
    //scales all components to the one with the largest magnitude
    if (massiveCord > 1 && massiveCord != 0) {
      xCord1 /= massiveCord;
      yCord1 /= massiveCord;
      xCord2 /= massiveCord;
      yCord2 /= massiveCord;
      xCord3 /= massiveCord;
      yCord3 /= massiveCord;
      xCord4 /= massiveCord;
      yCord4 /= massiveCord;
    }

    //component form --> magnitude and direction form as semifinal values
    drivetrainVectors[0][1] = atan2(yCord1, xCord1) * (180 / PI);
    drivetrainVectors[0][0] = sqrt(pow(xCord1, 2) + pow(yCord1, 2));
    drivetrainVectors[1][1] = atan2(yCord2, xCord2) * (180 / PI);
    drivetrainVectors[1][0] = sqrt(pow(xCord2, 2) + pow(yCord2, 2));
    drivetrainVectors[2][1] = atan2(yCord3, xCord3) * (180 / PI);
    drivetrainVectors[2][0] = sqrt(pow(xCord3, 2) + pow(yCord3, 2));
    drivetrainVectors[3][1] = atan2(yCord4, xCord4) * (180 / PI);
    drivetrainVectors[3][0] = sqrt(pow(xCord4, 2) + pow(yCord4, 2));

    //Wrapping functions for 360 degree motion with 180 degree servos
    if (drivetrainVectors[0][1] < -1) {
      drivetrainVectors[0][1] += 181;
      drivetrainVectors[0][0] *= -1;
    }
    if (drivetrainVectors[0][1] > 181) {
      drivetrainVectors[0][1] -= 181;
      drivetrainVectors[0][0] *= -1;
    }

    if (drivetrainVectors[1][1] < -1) {
      drivetrainVectors[1][1] += 181;
      drivetrainVectors[1][0] *= -1;
    }
    if (drivetrainVectors[1][1] > 181) {
      drivetrainVectors[1][1] -= 181;
      drivetrainVectors[1][0] *= -1;
    }

    if (drivetrainVectors[2][1] < -1) {
      drivetrainVectors[2][1] += 181;
      drivetrainVectors[2][0] *= -1;
    }
    if (drivetrainVectors[2][1] > 181) {
      drivetrainVectors[2][1] -= 181;
      drivetrainVectors[2][0] *= -1;
    }

    if (drivetrainVectors[3][1] < -1) {
      drivetrainVectors[3][1] += 181;
      drivetrainVectors[3][0] *= -1;
    }
    if (drivetrainVectors[3][1] > 181) {
      drivetrainVectors[3][1] -= 181;
      drivetrainVectors[3][0] *= -1;
    }


    //write to drivetrain + deadzone
    if (PestoLink.isConnected() && (abs(PestoLink.getAxis(0)) + abs(PestoLink.getAxis(1)) + abs(PestoLink.getAxis(2))) > 0.02) {
      Turn1.write(int(drivetrainVectors[0][1]));
      Drive1.set(drivetrainVectors[0][0]);
      Turn2.write(int(drivetrainVectors[1][1]));
      Drive2.set(drivetrainVectors[1][0]);
      Turn3.write(int(drivetrainVectors[2][1]));
      Drive3.set(drivetrainVectors[2][0]);
      Turn4.write(int(drivetrainVectors[3][1]));
      Drive4.set(drivetrainVectors[3][0]);
    } else {
      Turn1.write(0);
      Drive1.setBrakeMode(true);
      Drive1.set(0);
      Turn2.write(0);
      Drive2.setBrakeMode(true);
      Drive2.set(0);
      Turn3.write(0);
      Drive3.setBrakeMode(true);
      Drive3.set(0);
      Turn4.write(0);
      Drive4.setBrakeMode(true);
      Drive4.set(0);
    }
    elevatorLeft.write(servoGoal);
    elevatorRight.write((-1 * servoGoal) + 180);
  
    pivotError = pivotGoal - pivotPosition;
    pivot.set(pivotPID.update(pivotError));
    Serial.println(pivotPosition);
    Serial.println(pivotGoal);
    
    if (STATE == CORAL) {
      if(PestoLink.buttonHeld(leftTrigger)) { //INTAKE
        PestoLink.rumble();
        coral.set(-1);
      }else if(PestoLink.buttonHeld(rightTrigger)){ //OUTTAKE
        coral.set(1);
      }else{
        coral.set(0);
      }
    } else if (STATE == ALGAE) {
      if(PestoLink.buttonHeld(leftTrigger)){ //INTAKE
        algae1.set(1);
        algae2.set(1);
      }else if(PestoLink.buttonHeld(rightTrigger)){// OUTTAKE
        algae1.set(-1);
        algae2.set(-1);
      }else{
        algae1.set(.15);
        algae2.set(.15);
    }
    }

    vTaskDelay(pdMS_TO_TICKS(10));  //this line is like arduino delay() but for rtos tasks
  }
}