#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include "Adafruit_VL6180X.h"
#include <Servo.h>
#define TCA9548A_ADDR 0x70
#define enA 2 //Enable1 L298 Pin enA 
#define in1 3 //Motor1 L298 Pin in1 
#define in2 4 //Motor1 L298 Pin in2 
#define in3 5 //Motor2 L298 Pin in3 
#define in4 6 //Motor2 L298 Pin in4 
#define enB 7 //Enable2 L298 Pin enB 
#define R_S A1 //ir sensor Right
#define L_S A2 //ir sensor Left
#define echo A3    //Echo pin
#define trigger A4 //Trigger pin
#define servo A5
bool systemActive = false; // Variable to track system state
bool colour_select = false; // Variable to track whether colour command activated
String robotStatus = "Idle";
Servo ServoLift;  // Create a servo object
int buzzer = 9;
int Set=20;
int distance_L, distance_F, distance_R;
int distance = 999;
int delayTime = 0; 
int counter = 0;
Servo myServo;  // Create servo object

void tcaSelect(uint8_t channel) {
    if (channel > 7) return;
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_4X);
Adafruit_VL6180X vl6180x = Adafruit_VL6180X();

String detectedColour = "Unknown"; 
String boxColour = "Unknown"; 
String colour = "None";

// Variable to track if an item is picked up
static bool itemPickedUp = false;


void setup(){ // put your setup code here, to run once
  Serial.begin(9600); // start serial communication at 9600bps
  Serial1.begin(9600);
  Wire.begin();
  Serial.println();
  // Select and initialize Sensor 1 (Channel 0)
  tcaSelect(6);
  if (tcs.begin()) {
        Serial.println("TCS34725 Sensor 1 Initialized on Channel 6");
        Serial1.println("TCS34725 Sensor 1 Initialized on Channel 6");
    } else {
        Serial.println("No TCS34725 found on Channel 6");
        Serial1.println("No TCS34725 found on Channel 6");
    }

  // Select and initialize Sensor 2 (Channel 1)
  tcaSelect(2);
  if (tcs.begin()) {
        Serial.println("TCS34725 Sensor 2 Initialized on Channel 2");
        Serial1.println("TCS34725 Sensor 2 Initialized on Channel 2");
  } else {
        Serial.println("No TCS34725 found on Channel 2");
        Serial1.println("No TCS34725 found on Channel 2");
  }

  tcaSelect(4);
  if (vl6180x.begin()) {
        Serial.println("VL6180X Sensor 1 Initialized on Channel 4");
        Serial1.println("VL6180X Sensor 1 Initialized on Channel 4");
  } else {
        Serial.println("No VL6180X found on Channel 4");
        Serial1.println("No VL6180X found on Channel 4");
  }

    tcaSelect(3);
  if (vl6180x.begin()) {
        Serial.println("VL6180X Sensor 2 Initialized on Channel 3");
        Serial1.println("VL6180X Sensor 2 Initialized on Channel 3");
  } else {
        Serial.println("No VL6180X found on Channel 3");
        Serial1.println("No VL6180X found on Channel 3");
  }
  
  delay(100);


  Serial1.println("HC-05 Bluetooth Module Ready");
  pinMode(R_S, INPUT); // declare if sensor as input  
  pinMode(L_S, INPUT); // declare ir sensor as input
  pinMode(echo, INPUT );// declare ultrasonic sensor Echo pin as input
  pinMode(trigger, OUTPUT); // declare ultrasonic sensor Trigger pin as Output  
  pinMode(enA, OUTPUT); // declare as output for L298 Pin enA 
  pinMode(in1, OUTPUT); // declare as output for L298 Pin in1 
  pinMode(in2, OUTPUT); // declare as output for L298 Pin in2 
  pinMode(in3, OUTPUT); // declare as output for L298 Pin in3   
  pinMode(in4, OUTPUT); // declare as output for L298 Pin in4 
  pinMode(enB, OUTPUT); // declare as output for L298 Pin enB 

  analogWrite(enA, 80); // Write The Duty Cycle 0 to 255 Enable Pin A for Motor1 Speed 
  analogWrite(enB, 80); // Write The Duty Cycle 0 to 255 Enable Pin B for Motor2 Speed 

  pinMode(servo, OUTPUT);
  pinMode(buzzer, OUTPUT);
  
  ServoLift.attach(8);  // Attach servo to pin 8
  counter = 0;

 for (int angle = 70; angle <= 120; angle += 5)  {
   servoPulse(servo, angle);  }
 for (int angle = 120; angle >= 20; angle -= 5)  {
   servoPulse(servo, angle);  }
 for (int angle = 20; angle <= 70; angle += 5)  {
   servoPulse(servo, angle);  }
  distance_F = Ultrasonic_read();

Serial1.println("Initialising complete.");
Serial1.println("Send '1' to start");

delay(500);
}

void loop()  {//Line Follower and Obstacle Avoiding=  
if (Serial1.available()) {
    char receivedChar = Serial1.read(); // Read the incoming character
    Serial.print("Received: ");
    Serial.println(receivedChar);
  if (receivedChar == '1') {
            systemActive = true;
            Serial1.println("System Started");
        } 
  else if (receivedChar == '0') {
            systemActive = false;
            Serial1.println("System Stopped");
            Serial1.println("System is stopped. Send '1' to start.");
            Stop();
        }
  else if (receivedChar == 'b') {
            systemActive = true;
            colour_select = true;
            colour = "Blue";
            Serial1.println("Only collecting blue items.");
  }
  else if (receivedChar == 'r') {
            systemActive = true;
            colour = "Red";
            Serial1.println("Only collecting red items.");
  }
  else if (receivedChar == 'y') {
            systemActive = true;
            colour_select = true;
            colour = "Yellow";
            Serial1.println("Only collecting yellow items.");
  }
  else if (receivedChar == 'w') {
            systemActive = true;
            colour_select = true;
            colour = "White";
            Serial1.println("Only collecting White items.");
  }
  else if (receivedChar == 'k') {
            systemActive = true;
            colour_select = true;
            colour = "Black";
            Serial1.println("Only collecting black items.");
  }
  else if (receivedChar == '2') {
            colour_select = false;
            colour = "None";
            Serial1.println("Reset for specified colour collection.");
  }
  else if (receivedChar == '3') {
            Stop();
            delay(500);
            Serial1.println("Robot stopped. Beginning variable reset.");
            distance = 999;
            delayTime = 0; 
            counter = 0;
            colour_select = false;
            colour = "None";
            detectedColour = "Unknown"; 
            boxColour = "Unknown"; 
            itemPickedUp = false;
            Serial1.println("Reset variables done.");
            systemActive = false;
            tone(buzzer, 1000);
            delay(1000);
            noTone(buzzer);
            delay(500);
            Serial1.println("HC-05 Bluetooth Module Ready.");
            Serial1.println("Send '1' to start.");
            return;
  }
}
if (systemActive == true){
  start();
}
}

void start(){
distance_F = Ultrasonic_read();

if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 0)){
  if(distance_F > Set){
  forward();
  }
  else{
    Stop();
    Serial1.print("Obstacle detected ");
    Serial1.print(distance_F);
    Serial1.println(" cm ahead.");
    Check_move();
    }  
}  
 
//if Right Sensor is Black and Left Sensor is White then it will call turn Right function
else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 0)){
  F_right();}  

//if Right Sensor is White and Left Sensor is Black then it will call turn Left function
else if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 1)){
  F_left();} 

//If black is detected by both sensors, robot will intiate pick-up mechanism
else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1)){
Stop();
delay(3000);
Serial1.println("Stop detected. Checking type...");
detectedColour = Colour_check1();
Serial1.print("Stop: ");
Serial1.println(detectedColour);
if ((itemPickedUp == false)&&(detectedColour == "Green")){
  Serial1.println("Pickup point detected.");
  Pick_up();
}
else if ((itemPickedUp == true)&&(detectedColour == boxColour)){
  Serial1.println("Drop-off point detected.");
  Drop_off();
}
else if ((itemPickedUp == true)&&(detectedColour == "Green")){
  Serial1.println("Dropping off. Ignoring pick-up.");
  forward();
  delay(400);
}
else if ((itemPickedUp == true)&&(detectedColour != boxColour)){
  Serial1.println("Wrong drop-off point. Resuming...");
  forward();
  delay(400);
}
else{
  Serial1.println("Not pick-up point. Resuming...");
  forward();
  delay(400);
}}
delay(10);
}

void servoPulse (int pin, int angle){
int pwm = (angle*11) + 500;      // Convert angle to microseconds
 digitalWrite(pin, HIGH);
 delayMicroseconds(pwm);
 digitalWrite(pin, LOW);
 delay(50); // Refresh cycle of servo
}


//Ultrasonic_read********
long Ultrasonic_read(){
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  long time = pulseIn (echo, HIGH);
  return time / 29 / 2;
}

void compareDistance(){
    if(distance_L > 20 && distance_L > distance_R){
    Serial1.println("Rerouting through left.");
    left();
    delay(600);
    Stop();
    delay(500);
    forward();
    delay(900);
    Stop();
    delay(500);
    right();
    delay(700);
    Stop();
    delay(500);
    forward();
    delay(1800);
    Stop();
    delay(500);
    right();
    delay(700);
    Stop();
    delay(500);
    forward();
    delay(900);
    Stop();
    delay(500);
    left();
    delay(600);
    Stop();
    delay(500);
    Serial1.println("Reroute complete");
    }
  
   else if(distance_R > 20 && distance_R > distance_L){
    Serial1.println("Rerouting through right.");
    right();
    delay(700);
    Stop();
    delay(500);
    forward();
    delay(900);
    Stop();
    delay(500);
    left();
    delay(600);
    Stop();
    delay(500);
    forward();
    delay(1800);
    Stop();
    delay(500);
    left();
    delay(600);
    Stop();
    delay(500);
    forward();
    delay(900);
    Stop();
    delay(500);
    right();
    delay(700);
    Stop();
    delay(500);
    Serial1.println("Reroute complete");
  }

   else if (distance_L < 20 && distance_R < 20) {
    Serial1.println("No alternative route found. Please remove obstacle.");
    tone(buzzer, 1000); // Play a 1000 Hz tone
    delay(5000); // Buzz for 5 seconds
    noTone(buzzer);
}
}

void Check_move(){
    Stop();
    delay(100);
    Serial1.println("Please remove obstacle. Waiting for removal...");
    tone(buzzer, 1000);
    delay(3000);
    noTone(buzzer);
    delay(500);
    distance_F = Ultrasonic_read();
  if (distance_F > 25) {
  Serial1.println("Obstacle removed. Resuming operation.");
  }
  else{
  for (int angle = 70; angle <= 120; angle += 5)  {
    servoPulse(servo, angle);  }
    delay(300);
    distance_R = Ultrasonic_read();
    Serial.print("D R=");Serial.println(distance_R);
    delay(100);
  for (int angle = 120; angle >= 20; angle -= 5)  {
    servoPulse(servo, angle);  }
    delay(500);
    distance_L = Ultrasonic_read();
    Serial.print("D L=");Serial.println(distance_L);
    delay(100);
 for (int angle = 20; angle <= 70; angle += 5)  {
   servoPulse(servo, angle);  }
    delay(300);
    compareDistance();
}}

void Pick_up(){
  Stop();
  delay(100);
  tof_read_L();
    if(itemPickedUp == true){
    return;
  }
  tof_read_R();
  if (counter == 5) {
    Stop();
    delay(500);
    Serial1.println("All pick-up points empty. Stopping operation.");
    systemActive = false;
    return;
  }
}


//Drop-Off Mechanism
void Drop_off(){
  Stop();
  delay(500);
  Serial1.println("Dropping off item.");
  right();
  delay(700);
  Stop();
  delay(200);
  backward();
  delay(700);
  Stop();
  delay(100);
  //ServoLift.write(180);    // Moves forklift up
  //delay(4000);
  ServoLift.write(0);    // Moves forklift down
  delay(8000);         // Run for 8 seconds
  ServoLift.write(90);
  itemPickedUp = false;
  Serial1.println("Drop-off complete.");
  forward();
  delay(700);
  Stop();
  delay(200);
  left();
  delay(600);
}

//Colour Check Mechanism (Function to detect color and store it in a variable)
String Colour_check1() {
    int r, g, b, c;
    uint8_t red, green, blue;

    // Read data from Sensor 1 (Channel 0)
    tcaSelect(6);
    tcs.getRawData(&r, &g, &b, &c);
    normalizeRGB(r, g, b, c, &red, &green, &blue);

    Serial1.print("Normalized RGB: ");
    Serial1.print((int)red); Serial1.print(", ");
    Serial1.print((int)green); Serial1.print(", ");
    Serial1.print((int)blue); Serial1.println();

    if (red >= 120 && green <= 80 && blue <= 75) {
        return "Red";
    } else if (red <= 90 && green >= 110 && blue <= 90) {
        return "Green";
    } else if (red <= 70 && green <= 110 && blue >= 110) {
        return "Blue";
    } else if (red >= 100 && green >= 90 && blue <= 80) {
        return "Yellow";
    } else if (red >= 130 && green >= 90 && blue >= 130) {
        return "White"; 
    } else if (red >= 80 && red <= 120 && green >= 80 && green <= 120 && blue >= 80 && blue <= 120) {
        return "Black";
    } else {
        return "Unknown"; // Default case
    }
}

String Colour_check2() {
    uint16_t r, g, b, c;
    float red, green, blue;

    // Read data from Sensor 1 (Channel 0)
    tcaSelect(2);
    tcs.getRawData(&r, &g, &b, &c);
    normalizeRGB_ratio(r, g, b, &red, &green, &blue);

    Serial1.print("Normalized RGB: ");
    Serial1.print(red, 4); Serial1.print(", ");
    Serial1.print(green, 4); Serial1.print(", ");
    Serial1.print(blue, 4); Serial1.println();

    if (red >= 0.5 && red <= 0.65 && green <= 0.25 && blue <= 0.24) {
        return "Red";
    } else if (red <= 0.27 && green <= 0.39 && blue >= 0.35) {
        return "Blue";
    } else if (red >= 0.43 && red <= 0.49 && green <= 0.35 && blue <= 0.22) {
        return "Yellow";
    } else if (red >= 0.29 && red <= 0.33 && green >= 0.35 && blue >= 0.31) {
        return "White";
    } else if (red >= 0.35 && red <= 0.37 && green <= 0.35 && blue <= 0.31) {
        return "Black";
    } else {
        return "Unknown"; // Default case
    }
}


void forward() {
  analogWrite(enA, 60);
  analogWrite(enB, 60);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}


void left() {
  analogWrite(enA, 120);
  analogWrite(enB, 120);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void right() {
  analogWrite(enA, 120);
  analogWrite(enB, 120);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void backward() {
  analogWrite(enA, 60);
  analogWrite(enB, 60);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void F_left() {
  analogWrite(enA, 90);
  analogWrite(enB, 90);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void F_right() {
  analogWrite(enA, 90);
  analogWrite(enB, 90);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void Stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void normalizeRGB(float r, float g, float b, float c, uint8_t* red, uint8_t* green, uint8_t* blue) {
    if (c == 0) {
        *red = 0;
        *green = 0;
        *blue = 0;
    } else {
        *red = (uint8_t)(r / c * 255);
        *green = (uint8_t)(g / c * 255);
        *blue = (uint8_t)(b / c * 255);
    }
}

void normalizeRGB_ratio(uint16_t r, uint16_t g, uint16_t b, float *red, float *green, float *blue) {
    uint16_t sum = r + g + b;
    if (sum > 0) {
        *red = (float)r / sum;
        *green = (float)g / sum;
        *blue = (float)b / sum;
    } else {
        *red = *green = *blue = 0.0;
    }
}

void tof_read_L(){
  tcaSelect(4);
   distance = vl6180x.readRange();
 if ((distance > 199)||(distance<51)){
  Serial1.println("No item on left.");
  return;
 }
  // Constrain distance to a safe range (50mm to 200mm)
  else{
    distance = constrain(vl6180x.readRange(), 50, 200);
    // Map distance to delay time (50mm → 2000ms, 200mm → 200ms)
    delayTime = map(distance, 100, 200, 700, 900);

    // Print values for debugging
    Serial1.print("Left Distance: ");
    Serial1.print(distance);
    Serial1.print(" mm, Delay: ");
    Serial1.print(delayTime);
    Serial1.println(" ms");
    Serial1.println("Item on left. Initiating pick-up.");
    right();
    delay(700);
    Stop();
    delay(200);
    backward();
    delay(delayTime);
    Stop();
    delay(1000);
    boxColour = Colour_check2();
    Serial1.print("Box Colour: ");
    Serial1.println(boxColour);
    if ((colour_select == true)&&(colour == boxColour)){
      Serial1.println("Start Pick-Up.");
      ServoLift.write(180);    // Moves forklift up
      delay(10000);
      //ServoLift.write(0);    // Moves forklift down
      //delay(4000);         // Run for 8 seconds
      ServoLift.write(90);
      itemPickedUp = true; // Mark item as picked up
      Serial1.println("Item picked up.");
    }
    else {
      Serial1.println("Wrong item colour. Resuming...");
    }
    if (colour_select == false){
      Serial1.println("Start Pick-Up.");
      ServoLift.write(180);    // Moves forklift up
      delay(10000);
      //ServoLift.write(0);    // Moves forklift down
      //delay(4000);         // Run for 8 seconds
      ServoLift.write(90);
      itemPickedUp = true; // Mark item as picked up
      Serial1.println("Item picked up.");
    }
    forward();
    delay(delayTime-50);
    Stop();
    delay(200);
    left();
    delay(600);
    forward();
    delay(100);
    return;
}}

void tof_read_R(){
  tcaSelect(3);
  distance = vl6180x.readRange();
 if ((distance > 199)||(distance < 51)){
  Serial1.println("No item on right. Resuming...");
  counter++;
  forward();
  delay(500);
  return;
 }
  // Constrain distance to a safe range (50mm to 200mm)
  else{
    counter = 0; //reset counter
    distance = constrain(vl6180x.readRange(), 50, 200);
  
    // Map distance to delay time (50mm → 2000ms, 200mm → 200ms)
    delayTime = map(distance, 100, 200, 700, 900);

    // Print values for debugging
    Serial1.print("Right Distance: ");
    Serial1.print(distance);
    Serial1.print(" mm, Delay: ");
    Serial1.print(delayTime);
    Serial1.println(" ms");
    Serial1.println("Item on right. Initiating pick-up.");
    left();
    delay(600);
    Stop();
    delay(200); 
    backward();
    delay(delayTime);
    Stop();
    delay(1000);
    boxColour = Colour_check2();
    Serial1.print("Box Colour: ");
    Serial1.println(boxColour);
    if ((colour_select == true)&&(colour == boxColour)){
      Serial1.println("Start Pick-Up.");
      delay(500);   
      ServoLift.write(180);    // Moves forklift up
      delay(10000);
      //ServoLift.write(0);  // Moves forklift down
      //delay(4000);         // Run for 8 seconds
      ServoLift.write(90);
      itemPickedUp = true; // Mark item as picked up
      Serial1.println("Item picked up.");
    }
    else {
      Serial1.println("Wrong item colour. Resuming...");
    }
    if (colour_select == false){
      Serial1.println("Start Pick-Up.");
      ServoLift.write(180);    // Moves forklift up
      delay(10000);
      //ServoLift.write(0);    // Moves forklift down
      //delay(4000);         // Run for 8 seconds
      ServoLift.write(90);
      itemPickedUp = true; // Mark item as picked up
      Serial1.println("Item picked up.");
    }
    forward();
    delay(delayTime-100);
    Stop();
    delay(200);
    right();
    delay(700);
    forward();
    delay(100);
  }
}
