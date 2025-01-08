/*
   7MRI0060 - Applied Medical Robotics Module
   October 2024
   Author: Alejandro Granados and Harry Robertshaw

   Purpose: Run a PID controller

   Tasks:
    1. Set pins to code​
    2. Compute degrees and bound them to [-360,360]​
    3. Compute elapsed time (deltaT) and control the frequency of printing​
    4. Set demand position in degrees and compute errors (compared to previous, accumulation, and differential). 
       Hint: make sure you update previous error at the end.
    5. Compute PID​
    6. Send voltage to motors​
    7. Plot demand versus current position in degrees
*/

// Define constants for pulses per revolution (PPR) and gear ratio (GR)
const float PPR = 3575.0855;
const float GR1 = 297.924;
const float GR2 = 50.0;
const float CPR = 3; //cycles per revolution

// Variables for tracking encoder positions
volatile long counter_m1 = 0;
volatile long counter_m2 = 0;
int aLastState_m1;
int aLastState_m2;

// Pins for reading encoders of motor 1 and 2
const int encoderPinA_m1 = 2;
const int encoderPinB_m1 = 10;
const int encoderPinA_m2 = 3;
const int encoderPinB_m2 = 11;

// Pins for setting the direction of motor 1 and 2
const int motorPin1_m1 = 4;
const int motorPin2_m1 = 5;
const int motorPin1_m2 = 7;
const int motorPin2_m2 = 8;

// Pins for setting the speed of rotation (Enable pin) of motors 1 and 2
const int enablePin_m1 = 6;
const int enablePin_m2 = 9;

const int ledPin = 13;

// Variables for encoder positions and desired positions
long currentPosition_m1 = 0;
long currentPosition_m2 = 0;
float demandPositionInDegrees_m1 = 0; // multiply by 30 to account for gear
float demandPositionInDegrees_m2 = 0; //always negative to be in the same direction
float currentPositionInDegrees_m1;
float currentPositionInDegrees_m2;


// Time parameters
unsigned long currentTime;
unsigned long previousTime = 0;
unsigned long deltaT;

// PID gains
float Kp_m1 = 4.5, Kd_m1 = 0.2, Ki_m1 = 0.0; //Start with P, then D, then I. Low P will be slow
float Kp_m2 = 2.2, Kd_m2 = 0.5, Ki_m2 = 0.03;

// Error values
float errorPositionInDegrees_prev_m1 = 0, errorPositionInDegrees_sum_m1 = 0;
float errorPositionInDegrees_prev_m2 = 0, errorPositionInDegrees_sum_m2 = 0;

//gendata values
int i = 0;                // i 是一个计数器，用来生成数据序列。
String matlabStr = "";    // matlabStr 存储从MATLAB接收的字符串命令，一开始是空的
bool readyToSend = false; // readyToSend 是一个标志位，表明Arduino是否已接收到完整的MATLAB指令，可以开始发送数据。

char c;                   // c 用于逐个接收MATLAB发送的字符。
float val1 = 0.0;         // input1 from matlab
float val2 = 0.0;         // input2 from matlab 从matlab接受的两个浮点数参数


void setup() {
  Serial.begin(9600);

  // Task 1: Initialize the pins using pinMode and attachInterrupt functions
  pinMode(encoderPinA_m1, INPUT_PULLUP);
  pinMode(encoderPinA_m2, INPUT_PULLUP);
  pinMode(encoderPinB_m1, INPUT_PULLUP);
  pinMode(encoderPinB_m2, INPUT_PULLUP);

  pinMode(motorPin1_m1, OUTPUT);
  pinMode(motorPin2_m1, OUTPUT);
  pinMode(enablePin_m1, OUTPUT);
  pinMode(motorPin1_m2, OUTPUT);
  pinMode(motorPin2_m2, OUTPUT);
  pinMode(enablePin_m2, OUTPUT);
  pinMode(ledPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPinA_m1), updateEncoder_m1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA_m2), updateEncoder_m2, CHANGE);

  aLastState_m1 = digitalRead(encoderPinA_m1);
  aLastState_m2 = digitalRead(encoderPinA_m2);
  
  delay(3000);
  previousTime = micros();
  digitalWrite(ledPin,LOW);
}

void loop() {

  // digitalWrite(ledPin,HIGH);
  // delay(1000);
  // digitalWrite(ledPin,LOW);
  // delay(1000);

  //gendata part
    // 当 readyToSend 为 false 时，Arduino等待接收MATLAB的数据。
  if (readyToSend == false) {
    if (Serial.available()>0)       // 检查是否有数据到达串口
    {
      c = Serial.read();            // 读取一个字符
      matlabStr = matlabStr + c;    // 将接收到的字符追加到matlabStr 字符串中
      
      if (matlabStr.indexOf(";") != -1) // 检查是否收到 ; 结束符，表明MATLAB已发送完整命令。
      {

        readyToSend = true;         // 设置标志为true，表示已接收到完整命令

        // parse incomming data, e.g. C40.0,3.5;
        int posComma1 = matlabStr.indexOf(",");                     // 查找字符串中逗号的位置
        val1 = matlabStr.substring(1, posComma1).toFloat();         // 从字符1到逗号位置提取子字符串并转换为浮点数
        int posEnd = matlabStr.indexOf(";");                        // 查找最后一个字符的位置
        val2 = matlabStr.substring(posComma1+1, posEnd).toFloat();  // 从逗号+1到结束字符前提取子字符串并转换为浮点数

        if (fabs(val1-10) < 1) {
          digitalWrite(ledPin,HIGH);
        }
        demandPositionInDegrees_m1 = val1;
        demandPositionInDegrees_m2 = val2;
      }
    }
  }

  
  // Ardiono发送数据 to matlab
  if (readyToSend)  // 当接收到完整命令后开始数据发送。
  {
    // e.g. c1,100
    Serial.print("c");                    // 构造一个数据行，以 c 开头
    Serial.print(i);                      // 计数,序列1
    Serial.print(",");                    // 分隔符
    Serial.print(val2*sin(i*val1/360.0)); // 发送计算结果，序列2
    Serial.write(13);                     // 发送回车CR
    Serial.write(10);                     // 发送换行NL，结束本行数据
    i += 1;                               // 增加计数器i

    readyToSend = false;//-----------------------------------------------------------
  }

  // Task 2: Compute the current position in degrees and bound it to [-360,360]
  currentPositionInDegrees_m1 = ((counter_m1*360)/(CPR*GR1*2));
  if (currentPositionInDegrees_m1 >= 360.0 || currentPositionInDegrees_m1 <= -360.0) {
    counter_m1 -= ((GR1*CPR*2)*((int)(currentPositionInDegrees_m1/360)));
  }
  currentPositionInDegrees_m2 = ((counter_m2*360)/(CPR*GR2*2));
  if (currentPositionInDegrees_m2 >= 360.0 || currentPositionInDegrees_m2 <= -360.0) {
    counter_m2 -= ((GR2*CPR*2)*((int)(currentPositionInDegrees_m2/360)));
  }
  // Task 3: Compute elapsed time (deltaT) and control the frequency of printing​
  currentTime = micros();
  deltaT = currentTime - previousTime;
  previousTime = currentTime;
  // Serial.println(deltaT);

  if (deltaT > 400) {
    // Task 4: Compute error (P,I,D), and ensure that the previous error is updated
    float errorPositionInDegrees_m1 = demandPositionInDegrees_m1 - currentPositionInDegrees_m1;
    float errorPositionInDegrees_diff_m1 = (errorPositionInDegrees_m1 - errorPositionInDegrees_prev_m1) / deltaT;
    errorPositionInDegrees_sum_m1 += errorPositionInDegrees_m1;
    errorPositionInDegrees_prev_m1 = errorPositionInDegrees_m1;

    float errorPositionInDegrees_m2 = demandPositionInDegrees_m2 - 
    currentPositionInDegrees_m2;
    float errorPositionInDegrees_diff_m2 = (errorPositionInDegrees_m2 - errorPositionInDegrees_prev_m2) / deltaT;
    errorPositionInDegrees_sum_m2 += errorPositionInDegrees_m2;
    errorPositionInDegrees_prev_m2 = errorPositionInDegrees_m2;

    // Task 5: Compute the PID output
    float controllerOutput_m1 = errorPositionInDegrees_m1 * Kp_m1 + errorPositionInDegrees_diff_m1 * Kd_m1 + errorPositionInDegrees_sum_m1 * Ki_m1 * deltaT;

    controllerOutput_m1 = constrain(controllerOutput_m1, -255, 255);

    float controllerOutput_m2 = errorPositionInDegrees_m2 * Kp_m2 + errorPositionInDegrees_diff_m2 * Kd_m2 + errorPositionInDegrees_sum_m2 * Ki_m2 * deltaT;

    controllerOutput_m2 = constrain(controllerOutput_m2, -255, 255);

    // Task 6: Send voltage to motors
    if (controllerOutput_m1 > 0) {
      digitalWrite(motorPin1_m1, HIGH);
      digitalWrite(motorPin2_m1, LOW);
      analogWrite(enablePin_m1, controllerOutput_m1);
    } else {
      digitalWrite(motorPin1_m1, LOW);
      digitalWrite(motorPin2_m1, HIGH);
      analogWrite(enablePin_m1, -controllerOutput_m1);
    }

      if (controllerOutput_m2 > 0) {
      digitalWrite(motorPin1_m2, HIGH);
      digitalWrite(motorPin2_m2, LOW);
      analogWrite(enablePin_m2, controllerOutput_m2);
    } else {
      digitalWrite(motorPin1_m2, LOW);
      digitalWrite(motorPin2_m2, HIGH);
      analogWrite(enablePin_m2, -controllerOutput_m2);
    }

    // Task 7: Print the current position and demand for plotting
    // Serial.print("Current Position of Motor 1: ");
    // Serial.println(currentPositionInDegrees_m1/30);

    // Serial.print("Demand Position of Motor 1: ");
    // Serial.println(demandPositionInDegrees_m1/30);

    // Serial.print("Current Position of Motor 2: ");
    // Serial.println(currentPositionInDegrees_m2/30);

    // Serial.print("Demand Position of Motor 2: ");
    // Serial.println(demandPositionInDegrees_m2/30);
  }
}

// Interrupt function for tracking the encoder positions for motor 1
void updateEncoder_m1() {
  int aState_m1 = digitalRead(encoderPinA_m1);  // Read the current state of encoder pin A
  if (aState_m1 != aLastState_m1) {  // If state changes
      counter_m1 += (digitalRead(encoderPinB_m1) != aState_m1) ? 1 : -1;  // Clockwise rotation
      aLastState_m1 = aState_m1;
  }
}

// Interrupt function for tracking the encoder positions for motor 2
void updateEncoder_m2() {
  int aState_m2 = digitalRead(encoderPinA_m2);  // Read the current state of encoder pin A
  if (aState_m2 != aLastState_m2) {  // If state changes
    if (digitalRead(encoderPinB_m2) != aState_m2) {
      counter_m2++;  // Clockwise rotation
    } else {
      counter_m2--;  // Counter-clockwise rotation
    }
    aLastState_m2 = aState_m2;  // Update last state
  }
}