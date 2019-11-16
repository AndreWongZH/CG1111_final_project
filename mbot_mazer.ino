/**
 * @file    mbot_mazer.ino
 * @author  Group 1-1-A
 * @date    16/11/2019
 * @brief   Description: this file is the code for Mbot to complete the maze
 *
 * Function List:
 *      1. void sensedBlack(void)
 *      2. void readColor(void)
 *      3. int getColorReading(void)
 *      4. void irSensor(void)
 *      5. void readDistance(void)
 *      6. void calibrateCenter(void)
 *      7. int getCenterLeft(void)
 *      8. int getCenterRight(void)
 *      9. void turnLeft(void)
 *      10. void turnRight(void)
 *      11. void uTurn(void)
 *      12. void goStraight(void)
 *      13. void reverse(void)
 *      14. void turnRobot(void)
 *      15. int getLowSound(void)
 *      16. int getHighSound(void)
 *      17. void play(void)
 */

// add in library for MBot
#include "MeMCore.h"

// Constants for victory song
#define NOTE_A 550
#define NOTE_As 582
#define NOTE_B 617
#define NOTE_C 654
#define NOTE_D 734
#define NOTE_Ds 777
#define NOTE_E 824
#define NOTE_F 873
#define NOTE_Fs 925
#define NOTE_G 980
#define NOTE_Gs 1003
#define NOTE_A2 1100
#define NOTE_A2s 1165
#define NOTE_B2 1234
#define NOTE_C3 1308
#define NOTE_C3S 1385
#define NOTE_D3 1555
#define TEMPO 1110

int melody[] = {NOTE_B, NOTE_E, NOTE_G,NOTE_F, NOTE_E, NOTE_B2, NOTE_A2, NOTE_Fs, NOTE_E, NOTE_G, NOTE_F, NOTE_Ds, NOTE_F, NOTE_B};

double noteDurations[] = {1.0, 1.5, 0.5, 1, 2, 1, 2.5, 2.5, 1.5, 0.5, 1, 2, 1, 2.5};

//----------------------------------------------------------------------------------------------------------------------------------

MeUltrasonicSensor ultraSensor(PORT_1);
MeLineFollower lineFinder(PORT_2);
MeDCMotor rightwheel(M1);
MeDCMotor leftwheel(M2);

// normal moving speed
int16_t motorSpeed = 170;

// additional turning speed
int16_t turningSpeed = 90;

//----------------------------------------------- FOR COLOR ------------------------------------------------------------------------
MeRGBLed rgbled_7(7, 7==7?2:4);
MeLightSensor lightsensor(6);

// to hold colour arrays
int colourArray[] = {0,0,0};

// used for calibration
int whiteArray[] = {563,394,445};
int blackArray[] = {204,146,161};
int greyDiff[] = {359,248,284};

int red;
int green;
int blue;

// number of milliseconds to wait before taking another reading
int lightsensorWait = 5;
// number of readings taken
int times = 5;

//----------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------- FOR IR SENSOR --------------------------------------------------------------------
MePort IR(PORT_3);

// proportional constant for PID controller
double pconstantLeft = 1.25;
double pconstantRight = 1.16;

int16_t irLeft;
int16_t leftError;

int16_t irRight;
int16_t rightError;

int16_t centerLeft;
int16_t centerRight;
//----------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------- FOR MIC SENSOR --------------------------------------------------------------------
MePort Mic(PORT_4);

int lowFreq;
int highFreq;

//----------------------------------------------------------------------------------------------------------------------------------
void setup()
{
    Serial.begin(9600);
    calibrateCenter();
    goStraight();
}

void loop()
{
    if (lineFinder.readSensors() < 3){
        sensedBlack();
    }

    irSensor();
}

//--------------------------------------------Colour sensor functions---------------------------------------------------------------

/**
 * command the Mbot to stop moving and start reading the colour paper above the Mbot.
 * After reading the colour, make an appropriate turn
 */
void sensedBlack() {
    rightwheel.stop();
    leftwheel.stop();
    delay(200);
    readColor();
    // compare colour and then turn accordingly
    turnRobot();
}

/**
 * get the average reading from the light sensor when red, green and blue light is emitted
 * store this value in an array
 */
void readColor() {
    // scan red
    rgbled_7.setColor(0,255,0,0);
    rgbled_7.show();
    delay(30);
    red = ((getColorReading() - blackArray[0]) / greyDiff[0]) * 1023;
    colourArray[0] = red;
  
    // scan green
    rgbled_7.setColor(0,0,255,0);
    rgbled_7.show();
    delay(30);
    green = ((getColorReading() - blackArray[1]) / greyDiff[1]) * 1023;
    colourArray[1] = green;
  
    // scan blue
    rgbled_7.setColor(0,0,0,255);
    rgbled_7.show();
    delay(30);
    blue = ((getColorReading() - blackArray[2]) / greyDiff[2]) * 1023;
    colourArray[2] = blue;
}

/**
 * return the average of 5 readings from the lightsensor
 */
int getColorReading(){
    int reading;
    int total = 0;
    int i;

    for(i = 0; i < times; i++){
        reading = lightsensor.read();
        total = reading + total;
        delay(lightsensorWait);
    }

    return total/times;
}

//---------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------IR functions----------------------------------------------------

/**
 * First calls the readDistance to get the current IR values
 * If the error difference is less than -15, adjust the speed of the motor for 30 milliseconds
 * Finally move straight again
 */
void irSensor() {
    readDistance();

    // if moving towards the right
    if (rightError < -15) {
        // decrease leftwheel speed for a few seconds
        leftwheel.run(-motorSpeed - ((double) rightError * pconstantRight));
    }
    // if moving towards the left
    else if (leftError < -15) {
        // decrease rightwheel speed for a few seconds
        rightwheel.run(motorSpeed + ((double) leftError * pconstantLeft));
    }
    if (lineFinder.readSensors() < 3) {
        sensedBlack();
    }
    delay(30);
    goStraight();
}

/**
 * read the current IR reading from each IR sensor and then calculates the error difference from the center values
 */
void readDistance() {
    irRight = IR.aRead1();
    rightError = irRight - centerRight;
    irLeft = IR.aRead2();
    leftError = irLeft - centerLeft;
}

/**
 * run at the setup phase to calculate the left and right IR sensor values at the center
 */
void calibrateCenter() {
    centerLeft = getCenterLeft();
    centerRight = getCenterRight();
}

/**
 * return the average of the 50 values of the left IR sensor
 */
int getCenterLeft(){
    int reading;
    int total = 0;
    int num = 50;
    int i;

    for(i = 0; i < num; i++){
        reading = IR.aRead2();
        total = reading + total;
    }

    return total/num;
}

/**
 * return the average of the 50 values of the right IR sensor
 */
int getCenterRight(){
    int reading;
    int total = 0;
    int num = 50;
    int i;

    for(i = 0; i < num; i++){
        reading = IR.aRead1();
        total = reading + total;
    }

    return total/num;
}

//---------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------Movement commands---------------------------------------------------------------

/**
 * command the Mbot to turn left
 * if Mbot is too close to an object infront, reverse first and then turn
 */
void turnLeft() {
      if (ultraSensor.distanceCm() < 4.5) {
        reverse();
    }
    rightwheel.run(motorSpeed+turningSpeed);
    leftwheel.run(motorSpeed);
    delay(310);
}

/**
 * command the Mbot to turn right
 * if Mbot is too close to an object infront, reverse first and then turn
 */
void turnRight() {
    if (ultraSensor.distanceCm() < 4.5) {
        reverse();
    }
    rightwheel.run(-motorSpeed);
    leftwheel.run(-motorSpeed-turningSpeed+35);
    delay(320);
}

/**
 * command the Mbot to uturn
 * if Mbot is too close to an object infront, reverse first and then turn
 */
void uTurn() {
    if (ultraSensor.distanceCm() < 4.5) {
        reverse();
    }
    rightwheel.run(motorSpeed+turningSpeed);
    leftwheel.run(motorSpeed+turningSpeed);
    delay(480); 
}

/**
 * command the Mbot to go straight
 */
void goStraight() {
    rightwheel.run(motorSpeed);
    leftwheel.run(-motorSpeed-20);
}

/**
 * command the Mbot to reverse
 */
void reverse() {
    rightwheel.run(-motorSpeed);
    leftwheel.run(motorSpeed+15);
    delay(200);
}

/**
 * command the Mbot to turn a spefic direction based on the colour or sound
 * if Mbot is executing a double right or left turn, the second turn will only be executed if the Mbot is a certain distance away from the wall
 */
void turnRobot() {
    // if lightblue then do double right turn
    if (colourArray[0] > 850 && colourArray[0] < 1220 && colourArray[1] > 1200 && colourArray[1] < 1800 && colourArray[2] > 1200 && colourArray[2] < 1710) {
        turnRight();
        while (ultraSensor.distanceCm() > 7) {
            goStraight();
        }
        turnRight();
    }

    // if yellow then turn 180 degree on the spot
    else if (colourArray[0] > 1300 && colourArray[0] < 1770 && colourArray[1] > 1200 && colourArray[1] < 1850 && colourArray[2] > 800 && colourArray[2] < 1400) {
        uTurn();
    }

    // if purple then do double left turn
    else if (colourArray[0] > 740 && colourArray[0] < 1100 && colourArray[1] > 800 && colourArray[1] < 1300 && colourArray[2] > 1000 && colourArray[2] < 1500) {
        turnLeft();
        while (ultraSensor.distanceCm() > 7) {
            goStraight();
        }
        turnLeft();
    }

    // if red then turn left
    else if (colourArray[0] > 1000 && colourArray[1] < 1000 && colourArray[2] < 1000) {
        turnLeft();
    }

    // if green then turn right
    else if (colourArray[0] < 1000 && colourArray[1] > 700  && colourArray[2] < 1000) {
        turnRight();
    }

    // if less black then do sound challenge
    else if (colourArray[0] > 240 && colourArray[0] < 420&& colourArray[1] > 240 && colourArray[1] < 520 && colourArray[2] > 300 && colourArray[2] < 530) {
        lowFreq = getLowSound();
        highFreq = getHighSound();

        if (lowFreq >= 9) {
            turnLeft();
        }
        else if (highFreq >= 9) {
            turnRight();
        }
        // if no detection then most likely there it has reach the end point
        else {
            play();
            delay(10000);
        }
    }

    // if very black then play victory song
    else if (colourArray[0] > 200 && colourArray[0] < 350 && colourArray[1] > 200 && colourArray[1] < 400  && colourArray[2] > 230 && colourArray[2] < 440) {
        play();
        delay(10000);
    }

    // continue to move straight
    goStraight();
}

//---------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------Sound processing---------------------------------------------------------------

/**
 * sample the values from the band pass microphone 1000 times and return the highest reading
 */
int getLowSound() {
    int highest = 0;
    int value;
    int times = 1000;
    int i;

    for (i = 0; i < times; i++) {
        value = Mic.aRead1();
        if (value > highest) {
            highest = value;
        }
    }
    return highest;
}

/**
 * sample the values from the high pass microphone 1000 times and return the highest reading
 */
int getHighSound() {
    int highest = 0;
    int value;
    int times = 1000;
    int i;

    for (i = 0; i < times; i++) {
        value = Mic.aRead2();
        if (value > highest) {
            highest = value;
        }
    }
    return highest;
}

//---------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------Victory Song--------------------------------------------------------------------

/**
 * plays the victory song indefinitely
 */
void play() {
    int note;
    while(true) {
        for (note = 0; note < 14; note++) {
            tone(8, melody[note], TEMPO * noteDurations[note]);
            delay(TEMPO * noteDurations[note] * 1.30);
            noTone(8);
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------------
