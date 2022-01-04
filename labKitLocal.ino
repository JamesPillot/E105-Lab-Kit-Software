#include <TimerOne.h>

#include <Wire.h>
#include <WireKinetis.h>

//------for Motor Encoder-----
//Pin Declares  Q 
int encoder0PinA = 2;
int encoder0PinB = 3;
int motorPWM = 6;
int motorDirA = 7;
int motorDirB = 8;
//dynamics variables
double force = 0;
double duty = 0;
double duty_coeff = 7.94326331388610; //from Matlab regression of torque duty test
double torque = 0;
//PID controller terms
double P = 0;
double I = 0;
double I_CAP = 20;
double D = 0;
int motor_output = 0;
//More variables for encoder and input processing
unsigned long ticks_per_rev = 12;
volatile signed long num_revs = 0;
unsigned long lastRead = 0;
unsigned long interval = 1000;//seconds
unsigned int sec_count =0;
//Additional variables for logic surrounding controller
uint8_t state = 0; //for state machine used with controller

double pwm_freq = 25000;

volatile signed long encoder0Pos = 0;

volatile signed long tick = 0; 
volatile signed long prev_tick = 0;
volatile signed long tick_thresh = 1000;
double angle_enc = 0; //encoder angle
int output_count = 0;

int count = 0;
int count_max = 2000;
//---------End------------

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

/* Variables associated with accelerometer*/
double acc_sens =8192; //counts/g
double acc_range = 8; //+-4g
double gyro_sens =32.8; //counts/deg/s
double gyro_range = 2000; //+-1000 deg/s
double mag_range = 4800; //microstesla
double mag_sens = 0.6; //microtesla/count 
//long i = 0; //loop counter
double gz_deg_s = 0;
double gz_deg_s_prev = 0;
double gz_rad_s = 0;
double angle_gyro = 0;
double angle_gyro_degrees = 0;
double curr_time = 0;
double prev_time = 0;
double t_step = 0;
double t_step_s = 0;
double gyro_offset = -0.85366; //CHANGE THIS FROM Kit to Kit
/*
 * for James' Copy it is .51829
 * for Lab 1 Copy it is -0.85366
 */
double ax_g = 0;
double ax_ms2 = 0;
double angle_acc = 0;
double acc_x_offset = -0.01400; 
double t_val = 0;

const int CYCLES_TO_WAIT = 200; //delay any movement until sensors initialize

/* Variables for control loop */

//for serial communication
const int MESSAGE_LENGTH = 18; //based on expected number of characters being received
const int SENSOR_DATA_LENGTH = 8; //number of characters needed to transmit sensor data
char incoming_message[MESSAGE_LENGTH] = "";
const int ZERO = 48; //decimal value associated with ASCII character '0'
//for gains
float kp = 0; //proportional gain    
float ki = 0; //integral gain
float kd = 0; //derivative gain
//for target and error
double theta_target = 0; //target theta (in degrees)
double theta_error = 0; //error in theta (in degrees)
double prev_theta_error = 0; //previous error in theta (in degrees)
//for logic associated with program
int new_step = 0; //Determines if new step response should be generated. 1 if true, 0 if false
//------------------------------------

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 

void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

/*Takes in a double of the form xxx.xx and returns it in string form
  Used to send angle gyro data to PC*/
void double_to_string(char* data_out, double gyro_angle_in)
{
    //scale the gyro data
    int data_raw = gyro_angle_in * 100;
    //pull out individual digits
    int first_digit = data_raw/10000;
    int second_digit = (data_raw - (first_digit*10000))/1000;
    int third_digit = (data_raw - (first_digit*10000) - (second_digit*1000))/100;
    int fourth_digit = (data_raw - (first_digit*10000) - (second_digit*1000) - (third_digit*100))/10;
    int fifth_digit = (data_raw - (first_digit*10000) - (second_digit*1000) - (third_digit*100) - (fourth_digit*10));
    //Figure out sign to attach to data
    char sign = ' ';
    if(gyro_angle_in < 0) sign = '-';
    //Build output string of data and pass it out
    char string_out[SENSOR_DATA_LENGTH] = {sign, (char)(ZERO+abs(first_digit)), (char)(ZERO+abs(second_digit)), (char)(ZERO+abs(third_digit)) ,'.', (char)(ZERO+abs(fourth_digit)), (char)(ZERO+abs(fifth_digit)),'\0'};
    strcpy(data_out, string_out);
}

/* doEncoderA (and B respectively) take care of incrementing the encoder count when a change in position has been detected*/
void doEncoderA() {
  //Serial.println("I'm here");
  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) {
    //Serial.println("A went high");
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == HIGH) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
}

void doEncoderB() {
  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinA) == LOW) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
}

// Initial time
long int ti;
volatile bool intFlag=false;

// Initializations
void setup()
{
  // Arduino initializations
  Wire.begin();
  //------for Motor Encoder-----
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDirA, OUTPUT); 
  pinMode(motorDirB, OUTPUT);
  analogWrite(motorPWM, 0); //make sure motor is off
  analogWriteFrequency(motorPWM, pwm_freq);
  digitalWrite(motorDirA, LOW);
  digitalWrite(motorDirB, LOW);
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoderB, CHANGE);
  //------------End----------------
  Serial.begin(9600);
  
  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,29,0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,26,0x06);
  // Configure gyroscope range
  //+-1000 degrees per second
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);
  // Configure accelerometers range
  //+-4g
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);
  // Set by pass mode for the magnetometers
  //+-4800 microTesla
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  
  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS,0x0A,0x16);
  
  pinMode(13, OUTPUT);
  Timer1.initialize(10000);         // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
  
  // Store initial time
  ti=millis();  
}


// Counter
long int cpt=0;

void callback()
{ 
  intFlag=true;
  digitalWrite(13, digitalRead(13) ^ 1);
}

// Main loop, read and display data
int loop_count = 0;

void loop()
{
  /* Input processing */ 
  while (!intFlag);
  intFlag=false;
  
  curr_time = millis()-ti;

  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  int16_t gz=Buf[12]<<8 | Buf[13];
  gz_deg_s = (double)(double(gz)/gyro_sens);
  gz_deg_s -= gyro_offset;
  gz_rad_s = double(gz_deg_s * double(M_PI/180));
    
  //trapezoidal integration to get angle
  t_step = curr_time-prev_time; //milliseconds
  t_step_s = double(t_step/1000); //seconds
  angle_gyro+=double(double(0.5*t_step_s*(gz_deg_s+gz_deg_s_prev))* double(M_PI/180));  
  angle_gyro_degrees = double(angle_gyro*double(180/M_PI));
  
  //read encoder values
  int clickres = 4; //quarter ticks
  if (encoder0Pos%clickres==0 && encoder0Pos!=0) {
    int sign = encoder0Pos/clickres; 
    tick+=(signed long)(sign);
    encoder0Pos=0;
  }
  if (abs(tick-prev_tick)>=tick_thresh) {
    tick = prev_tick;
  } else {
    prev_tick = tick;
  }

  double tick_fraction = (double)encoder0Pos/clickres;
  double tick_decimal = (double)(tick+tick_fraction);
  double arc_fraction = (double)(tick_decimal/ticks_per_rev);
  angle_enc = -1*(double)(arc_fraction*360); //degrees
  
  /* Parse Serial Data */
  int str_index = 0; //go to beginning of string before receiving message
  incoming_message[str_index] = '\0'; //"reset" message string for new cycle
  while(Serial.available())
  {
    char byte_in = Serial.read(); //read in a byte
    incoming_message[str_index] = byte_in; //construct message from PC (student's laptop)
    str_index++;
  }

  if(incoming_message[0] != '\0') //only parse message if there is something to read
  {
    if(incoming_message[0] == '!')
    {
      new_step = 0; //set new step to 0 to end step response
    }
    else if(incoming_message[0] == 'z')
    {
      angle_gyro = 0;
    }
    else
    {
      char* token = strtok(incoming_message, " ");
      theta_target = (double)atoi(token);
      kp = atof(strtok(NULL, " "));
      ki = atof(strtok(NULL, " "));
      kd = atof(strtok(NULL, " "));
      new_step = 1; //set new step to 1 to generate new step response
    }
  }
 
    /* Run Controller */
    theta_error = theta_target - angle_gyro_degrees;   //compute new error
    P = kp*(theta_error); //Calculate P term of PID controller 
    I += ki * theta_error; //Calculate I term of PID controller
    D = kd * (theta_error - prev_theta_error); //Calculate D term of PID controller
    
    if(I >= I_CAP) //Put a cap on our integral term as an "anti-windup" technique
    {
      I = I_CAP;
    }
    else if(I <= -1*I_CAP)
    {
      I = -1*I_CAP;
    }

    prev_theta_error = theta_error; //update previous error term
    /* Output Generation */
    motor_output = (int)(P+I+D); //Add terms of PID controller to generate output
    if(!new_step) //if we are not generating new step, end control loop
    {
      motor_output = 0;
      //reset I and D terms here as well
      I = 0;
      D = 0;
      prev_theta_error = 0;
      
    }
    //Prevent "byte overflow" by capping to either -255 or 255
    if(motor_output >= 255) motor_output = 255;
    if(motor_output <= -255) motor_output = -255;
    //Use sign of output to determine direction motor should spin
    if(motor_output >= 0) {
      digitalWrite(motorDirA, LOW);
      digitalWrite(motorDirB,HIGH);
    } 
    if(motor_output < 0) {
      digitalWrite(motorDirA, HIGH);
      digitalWrite(motorDirB,LOW);
    }

    /* Send data to PC */  
    char* data = (char*) malloc((SENSOR_DATA_LENGTH*sizeof(char))); //allocate memory for data to be sent over
    double_to_string(data, angle_gyro_degrees); //convert the gyro degrees value to a string
    Serial.write(data); //send it over
    free(data); //free up memory for the next cycle
    
    //Do not move the motor until we are ready for controller to become active
    if(loop_count < CYCLES_TO_WAIT)
    {
      analogWrite(motorPWM, 0);
      //Flush out I and D terms while we wait
      I = 0;
      D = 0;
      prev_theta_error = 0;
    }
    else
    {
      analogWrite(motorPWM, abs(motor_output));  // output the signal
    }
  
    //set variables for next loop
    gz_deg_s_prev = gz_deg_s;
    prev_time = curr_time;
  
    loop_count++;
  
    delay(10); //run loop at 100HZ
    
} 
