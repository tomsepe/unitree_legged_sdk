//https://www.instructables.com/RC-Control-and-Arduino-A-Complete-Works/
//This will sketch will read all 8 channels of a RC reciever and input the values via serial monitor.
//Programed for the Arduino MEGA 2560!!!

// Define Variables:
const int chA=1;  //Constant variables relating to pin locations
const int chB=2;
const int chC=3;
const int chD=4;
const int chE=5;
const int chF=6;
const int chG=7;
const int chH=8;

//Varibles to store and display the values of each channel
int ch1;  // throttle (not working)
int ch2;  // Right Joy, Left-Right
int ch3;  // Right Joystick, Forward-Back
int ch4;  // Left Joystick, Left-Right (roll)
int ch5;  // Left Switch, up-down (1095-1888)
int ch6;  // 3-way top switch, 0-1-2 (1888-1495-1095)
int ch7;  // 3-way left side front, 0(down)-1(middle)-2(up) (1888-1495-1095)
int ch8;


// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  // Set input pins
  pinMode(chA, INPUT);
  pinMode(chB,INPUT);
  pinMode(chC,INPUT);
  pinMode(chD,INPUT);
  pinMode(chE,INPUT);
  pinMode(chF,INPUT);
  pinMode(chG,INPUT);
  pinMode(chH,INPUT);
}

//Main Program
void loop() {
  // read the input channels
  ch1 = pulseIn (chA,HIGH);  //Read and store channel 1
  Serial.print ("Ch1:");  //Display text string on Serial Monitor to distinguish variables
  Serial.print (ch1);     //Print in the value of channel 1
  Serial.print ("|");
 
  ch2 = pulseIn (chB,HIGH);
  Serial.print ("Ch2:");
  Serial.print (ch2);
  Serial.print ("|");
 
  ch3 = pulseIn (chC,HIGH);
  Serial.print ("Ch3:");
  Serial.print (ch3);
  Serial.print ("|");
 
  ch4 = pulseIn (chD,HIGH);
  Serial.print ("Ch4:");
  Serial.print (ch4);
  Serial.print ("|");
 
  ch5 = pulseIn (chE,HIGH);
  Serial.print ("Ch5:");
  Serial.print (ch5);
  Serial.print ("|");
 
  ch6 = pulseIn (chF,HIGH);
  Serial.print ("Ch6:");
  Serial.print (ch6);
  Serial.print ("|");
 
  ch7 = pulseIn (chG,HIGH);
  Serial.print ("Ch7:");
  Serial.print (ch7);
  Serial.print ("|");
 
  ch8 = pulseIn (chH,HIGH);
  Serial.print ("Ch8:");
  Serial.println (ch8);
}