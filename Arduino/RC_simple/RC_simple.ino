/*
 RC PulseIn Serial Read out
 By: Nick Poole
 SparkFun Electronics
 Date: 5
 License: CC-BY SA 3.0 - Creative commons share-alike 3.0
 use this code however you'd like, just keep this license and
 attribute. Let me know if you make hugely, awesome, great changes.
 */

int ch1;  // Here's where we'll keep our channel values
int ch2;
int ch3;
int ch4;
int ch5;
int ch6;


#define ch01_aux 1
#define ch02_gear 2
#define ch03_rudd 3
#define ch04_elev 4
#define ch05_aeri 5
#define ch06_x 6


void setup() {

  pinMode(ch01_aux, INPUT);  // Set our input pins as such
  pinMode(ch02_gear, INPUT);
  pinMode(ch03_rudd, INPUT);
  pinMode(ch04_elev, INPUT);
  pinMode(ch05_aeri, INPUT);
  pinMode(ch06_x, INPUT);

  Serial.begin(9600);  // Pour a bowl of Serial
}

void loop() {

  ch1 = pulseIn(ch01_aux, HIGH, 25000);   // Read the pulse width of
  ch2 = pulseIn(ch02_gear, HIGH, 25000);  // each channel
  ch3 = pulseIn(ch03_rudd, HIGH, 25000);
  ch4 = pulseIn(ch04_elev, HIGH, 25000);
  ch5 = pulseIn(ch05_aeri, HIGH, 25000);
  ch5 = pulseIn(ch06_x, HIGH, 25000);

  // Serial.print("Channel 1:"); // Print the value of
  // Serial.println(ch1);        // each channel

  // Serial.print("Channel 2:"); //might be aux switch?
  // Serial.println(ch2);

  // Serial.print("Channel 3:"); //rudder left joystick L-R
  // Serial.println(ch3);

  //   Serial.print("Channel 4:"); // elev left joystick Up-Down (dead)
  // Serial.println(ch4);

  // Serial.print("Channel 5:");  // should be aeri right joystick L-R but getting 0
  // Serial.println(ch5);

  Serial.print("Channel 6:");  // should be aeri right joystick L-R but getting 0
  Serial.println(ch6);

  delay(100);  // I put this here just to make the terminal
               // window happier
}
