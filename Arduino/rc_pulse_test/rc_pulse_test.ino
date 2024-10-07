/*
 RC PulseIn Serial Read out
 By: Nick Poole
 SparkFun Electronics
 Date: 5
 License: CC-BY SA 3.0 - Creative commons share-alike 3.0
 use this code however you'd like, just keep this license and
 attribute. Let me know if you make hugely, awesome, great changes.
 */

// Here's where we'll keep our channel values

int ch1;  // throttle
int ch2;
int ch3;
int ch4;
int ch5;
int ch6;
int ch7;
int ch8;

void setup() {

  pinMode(1, INPUT);  // Set our input pins as such
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);

  Serial.begin(115200);  // Pour a bowl of Serial
}

void loop() {

  ch1 = pulseIn(1, HIGH, 25000);
  ch2 = pulseIn(2, HIGH, 25000);
  ch3 = pulseIn(3, HIGH, 25000);
  ch4 = pulseIn(4, HIGH, 25000);
  ch5 = pulseIn(5, HIGH, 25000);
  ch6 = pulseIn(6, HIGH, 25000);
  ch7 = pulseIn(7, HIGH, 25000);
  ch8 = pulseIn(8, HIGH, 25000);

  // Print the value of each channel
  Serial.print("Channel 1:");
  Serial.println(ch1);
  Serial.print("Channel 2:");
  Serial.println(ch2);
  Serial.print("Channel 3:");
  Serial.println(ch3);
  Serial.print("Channel 4:");
  Serial.println(ch4);
  Serial.print("Channel 5:");
  Serial.println(ch5);
  Serial.print("Channel 6:");
  Serial.println(ch6);
  Serial.print("Channel 7:");
  Serial.println(ch7);
  Serial.print("Channel 8:");
  Serial.println(ch8);

  delay(200);  // I put this here just to make the terminal
               // window happier
}
