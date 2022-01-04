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
double force = 0;
double duty = 0;
double duty_coeff = 7.94326331388610; //from Matlab regression of torque duty test
double torque = 0;
double output=0;
unsigned long ticks_per_rev = 12;
volatile signed long num_revs = 0;
unsigned long lastRead = 0;
unsigned long interval = 1000;//seconds
unsigned int sec_count =0;

double pwm_freq = 25000;

volatile signed long encoder0Pos = 0;

volatile signed long tick = 0; 
volatile signed long prev_tick = 0;
volatile signed long tick_thresh = 1000;
double angle_enc = 0; //encoder angle
int output_count = 0;
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

//My code:
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
double curr_time = 0;
double prev_time = 0;
double t_step = 0;
double t_step_s = 0;

double gyro_offset = 0.128104168; //CHANGE THIS

double ax_g = 0;
double ax_ms2 = 0;
double angle_acc = 0;
double acc_x_offset = -0.01400; //found manually
double t_val = 0;

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
//  analogWriteFrequency(motorPWM, pwm_freq);
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
void loop()
{
  while (!intFlag);
  intFlag=false;
  
  curr_time = millis()-ti;

  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  int16_t gz=Buf[12]<<8 | Buf[13];
  gz_deg_s = (double)(double(gz)/gyro_sens);
  gz_deg_s -= gyro_offset;
//  Serial.println(gz_deg_s,5); //leave uncommented for initial calibration
  
  //trapezoidal integration to get angle
  t_step = curr_time-prev_time; //milmliseconds
  t_step_s = double(t_step/1000); //seconds
  angle_gyro+=double(double(0.5*t_step_s*(gz_deg_s+gz_deg_s_prev))* double(M_PI/180));

  //set variables for next loop
  gz_deg_s_prev = gz_deg_s;
  prev_time = curr_time;

  //processing for encoder position
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
  
//  UNCOMMENT THIS FOR PART 2:
//    Serial.print(curr_time);
//    Serial.print("\t");
//    Serial.print(angle_enc,5);
//    Serial.print("\t");
    Serial.println(double(angle_gyro*double(180/M_PI)),5);

   //UNCOMMENT THIS FOR PART 3:
    /*if (curr_time<=5000){
      Serial.print(curr_time);
      Serial.print("\t");
      Serial.println(angle_gyro,5);
     }
      */
} 

  //----------------for Motor-------------
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
  //Serial.println (encoder0Pos, DEC);
  // use for debugging - remember to comment out
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


  //----------------End---------------------

  
