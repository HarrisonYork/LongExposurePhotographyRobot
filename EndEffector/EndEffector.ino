#include <Servo.h>

// LED pins
const int redLED = 13;
const int greenLED = 12;
const int blueLED = 11;

// transistor pins
const int t1 = 4;
const int t2 = 5;
const int t3 = 7;
const int t4 = 8;

// servo pins
Servo s1; // 6
Servo s2; // 9
Servo s3; // 3
Servo s4; // 10

// button pins
const int b1 = 43;
const int b2 = 45;
const int b3 = 47;
const int b4 = 49; 

// button states, HIGH is pressed, LOW is unpressed
int b1State = 0;
int b2State = 0;
int b3State = 0;
int b4State = 0;

// LED color codes
int off[] = {0, 0, 0};
int red[] = {255, 0, 0};
int orange[] = {255, 120, 0};
int yellow[] = {255, 255, 0};
int green[] = {0, 255, 0};
int blue[] = {0, 0, 255};
int purple[] = {255, 0, 255};
int bluegreen[] = {0, 255, 255};
int white[] = {255, 255, 255};

void setup() {
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);

  pinMode(t1, OUTPUT);
  pinMode(t2, OUTPUT);
  pinMode(t3, OUTPUT);
  pinMode(t4, OUTPUT);

  pinMode(b1, INPUT);
  pinMode(b2, INPUT);
  pinMode(b3, INPUT);
  pinMode(b4, INPUT);

  s1.attach(6);
  s2.attach(9);
  s3.attach(3);
  s4.attach(10);

  openRobot();
}

void loop() {
  // read button states and start program corresponding to button
  b1State = digitalRead(b1);
  b2State = digitalRead(b2);
  b3State = digitalRead(b3);
  b4State = digitalRead(b4);
  
  if (b1State == HIGH) {
    button1();
  } 
  if (b2State == HIGH) {
    button2();
  } 
  if (b3State == HIGH) {
    button3();
  } 
  if (b4State == HIGH) {
    button4();
  }
  delay(1000);
}

// startup sequence
void startup() {
  switchLED(0, 0, 0, 0, red);
  delay(400);

  writeLED(off);
  delay(400);

  writeLED(red);
  delay(400);

  writeLED(off);
  delay(400);

  writeLED(red);
  delay(400);

  writeLED(off);
  delay(400);

  writeLED(green);
  delay(400);

  writeLED(off);
  delay(400);
}

// demo sequence
void button1() {
  startup();

  switchLED(0, 0, 0, 0, white);
  int flex_count = 0;
  while (flex_count < 3) {
    writeLED(white);
    openRobot();
    delay(1000);

    writeLED(red);
    closeRobot();
    delay(1000);
    flex_count += 1;
  }

  writeLED(white);
  openRobot();

  // alternate open and closed
  multicolor(red, red, red, red, 300);
  
  s1.write(160);
  multicolor(blue, red, red, red, 300);

  s2.write(160);
  multicolor(red, green, red, red, 300);

  s3.write(160);
  multicolor(red, red, orange, red, 300);
  
  s4.write(160);
  multicolor(red, red, red, purple, 300);

  int moveTo = 150;
  while (moveTo >= 0) {
    switchLED(0, 1, 1, 1, blue);
    s1.write(moveTo);
    delay(200);

    switchLED(1, 0, 1, 1, green);
    s2.write(moveTo);
    delay(200);

    switchLED(1, 1, 0, 1, orange);
    s3.write(moveTo);
    delay(200);

    switchLED(1, 1, 1, 0, purple);
    s4.write(moveTo);
    delay(200);

    moveTo -= 20;
  }  

  switchLED(0, 0, 0, 0, white);
  openRobot();
  delay(500);

  writeLED(blue);
  closeRobot();
  delay(500);

  writeLED(green);
  openRobot();
  delay(500);

  writeLED(red);
  closeRobot();
  delay(500);

  writeLED(white);
  openRobot();
  delay(500);

  writeLED(off);

  int flex_count2 = 0;
  moveTo = 160;
  while (flex_count2 < 3) {
    switchLED(0, 1, 1, 1, blue);
    s1.write(moveTo);
    delay(200);

    switchLED(1, 0, 1, 1, green);
    s2.write(moveTo);
    delay(200);

    switchLED(1, 1, 0, 1, orange);
    s3.write(moveTo);
    delay(200);

    switchLED(1, 1, 1, 0, purple);
    s4.write(moveTo);
    delay(200);

    if (moveTo == 160) {
      moveTo = 0;
    } else {
      moveTo = 160;
    }
    flex_count2 += 1;
  }


  writeLED(white);
  openRobot();
  delay(1000);

  switchLED(0, 0, 1, 1, blue);
  moveAll(160, 160, 0, 0);
  delay(500);

  openRobot();
  delay(500);

  switchLED(1, 1, 0, 0, red);
  moveAll(0, 0, 160, 160);
  delay(500);

  openRobot();
  delay(500);

  switchLED(1, 0, 0, 1, orange);
  moveAll(0, 160, 160, 0);
  delay(500);

  openRobot();
  delay(500);

  switchLED(0, 1, 1, 0, blue);
  moveAll(160, 0, 0, 160);
  delay(500);

  openRobot();
  delay(500);

  switchLED(1, 0, 1, 0, green);
  moveAll(0, 160, 0, 160);
  delay(500);

  openRobot();
  delay(500);

  switchLED(0, 1, 0, 1, purple);
  moveAll(160, 0, 160, 0);
  delay(500);

  openRobot();
  switchLED(0, 0, 0, 0, white);

  while (!breakcheck(white, white, white, white, 1000, b1)) {
    continue;
  }

  writeLED(off);
}

// just lights
void button2() {
  startup();

  breakcheck(blue, green, red, yellow, 1000, b2);
  breakcheck(green, red, yellow, blue, 1000, b2);
  breakcheck(red, yellow, blue, green, 1000, b2);
  breakcheck(red, blue, green, yellow, 1000, b2);

  breakcheck(purple, white, bluegreen, orange, 1000, b2);
  breakcheck(white, bluegreen, orange, purple, 1000, b2);
  breakcheck(bluegreen, orange, purple, white, 1000, b2);
  breakcheck(orange, purple, white, bluegreen, 1000, b2);

  while (!breakcheck(blue, green, red, yellow, 1000, b2)) {
    continue;
  }
}

// movement and light pattern
void button3() {
  startup();
  
  switchLED(0, 1, 1, 1, red);
  s1.write(100);
  delay(200);
  s1.write(0);
  delay(200);
  s1.write(100);
  delay(200);
  s1.write(0);
  delay(200);

  s1.write(100);
  delay(200);
  s1.write(0);
  delay(600);

  switchLED(1, 0, 1, 1, blue);
  s2.write(100);
  delay(200);
  s2.write(0);
  delay(200);
  s2.write(100);
  delay(200);
  s2.write(0);
  delay(200);

  s2.write(100);
  delay(200);
  s2.write(0);
  delay(600);

  switchLED(1, 1, 0, 1, orange);
  s3.write(100);
  delay(400);
  switchLED(1, 1, 0, 0, orange);
  s4.write(100);
  delay(400);

  switchLED(0, 0, 0, 0, red);
  s3.write(0);
  s4.write(0);
  delay(400);

  switchLED(0, 1, 1, 1, green);
  s1.write(100);
  delay(200);
  s1.write(0);
  delay(200);
  s1.write(100);
  delay(200);
  s1.write(0);
  delay(200);

  switchLED(0, 0, 0, 0, blue);
  s1.write(100);
  s2.write(100);
  delay(400);

  switchLED(0, 0, 0, 0, green);
  s3.write(100);
  s4.write(100);
  delay(200);

  switchLED(0, 0, 0, 0, red);

  s1.write(0);
  s2.write(0);
  s3.write(0);
  s4.write(0);

  delay(800);

  switchLED(0, 0, 0, 0, blue);
  s1.write(100);
  s2.write(100);
  s3.write(100);
  s4.write(100);
  delay(400);

  s1.write(0);
  s2.write(0);
  s3.write(0);
  s4.write(0);
  delay(200);

  s1.write(100);
  s2.write(100);
  s3.write(100);
  s4.write(100);
  delay(600);

  s1.write(0);
  s2.write(0);
  s3.write(0);
  s4.write(0);
  delay(200);

  s1.write(100);
  s2.write(100);
  s3.write(100);
  s4.write(100);
  delay(200);

  s1.write(0);
  s2.write(0);
  s3.write(0);
  s4.write(0);
  delay(400);

  s1.write(100);
  s2.write(100);
  s3.write(100);
  s4.write(100);
  delay(200);

  s1.write(0);
  s2.write(0);
  s3.write(0);
  s4.write(0);
  delay(600);
}

// single blue light
void button4() {
  startup();
  switchLED(1, 1, 0, 1, blue);
  delay(120000);
}

// control LEDs on/off and single color
void switchLED(int a1, int a2, int a3, int a4, int color[]) {
  digitalWrite(t1, a1);
  digitalWrite(t2, a2);
  digitalWrite(t3, a3);
  digitalWrite(t4, a4);
  writeLED(color);
}

// write color to LED
void writeLED(int color[]) {
  analogWrite(redLED, color[0]);
  analogWrite(greenLED, color[1]);
  analogWrite(blueLED, color[2]);
}

void closeRobot() {
  s1.write(160);
  s2.write(160);
  s3.write(160);
  s4.write(160);
}

void openRobot() {
  s1.write(0);
  s2.write(0);
  s3.write(0);
  s4.write(0);
}

// write values to all servos
void moveAll(int a1, int a2, int a3, int a4) {
  s1.write(a1);
  s2.write(a2);
  s3.write(a3);
  s4.write(a4);
}

// set each LED as different color for certain amount of time
void multicolor(int color1[], int color2[], int color3[], int color4[], int time) {
  int counter = 0;
  while (time >= 0) {
    switch (counter % 4) {
      case 0: {
        switchLED(0, 1, 1, 1, color1);
      }
      break;
      case 1: {
        switchLED(1, 0, 1, 1, color2);
      }
      break;
      case 2: {
        switchLED(1, 1, 0, 1, color3);
      }
      break;
      case 3: {
        switchLED(1, 1, 1, 0, color4);
      }
      break;
    }
    counter += 1;
    time -= 1;
    delay(4);
  }
}

// set each LED as different color for certain amount of time while checking for a button press
// button press returns 1
int breakcheck(int color1[], int color2[], int color3[], int color4[], int time, int check) {
  int counter = 0;
  int reading = digitalRead(check);
  while (time >= 0 && reading != HIGH) {
    switch (counter % 4) {
      case 0: {
        switchLED(0, 1, 1, 1, color1);
      }
      break;
      case 1: {
        switchLED(1, 0, 1, 1, color2);
      }
      break;
      case 2: {
        switchLED(1, 1, 0, 1, color3);
      }
      break;
      case 3: {
        switchLED(1, 1, 1, 0, color4);
      }
      break;
    }
    counter += 1;
    time -= 1;
    delay(4);
    reading = digitalRead(check);
  }

  if (reading == HIGH) {
    switchLED(1, 1, 1, 1, off);
    return 1;
  }
  return 0;
}


