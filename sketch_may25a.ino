#include <PID_v1.h>
#include <Wire.h>
#include <TimerOne.h>
#include <math.h>

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

int16_t ax, ay, az;
int     v1, v2;
double  Setpoint;
double  Input;
double  Output;
double  Ku=0.5, Tu=0.5;
double  Kp=0.6*Ku;
double  Kd=(3*Ku*Tu)/40;
double  Ki=(1.2*Ku)/Tu;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

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
  //USED SO FAR : 3, 5, 6, 11, 13
  //Output pins for PWM generated with Timer0 and Timer 2
  pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(2,OUTPUT);//N1 from driver
  pinMode(4,OUTPUT);//N2 from driver
  pinMode(7,OUTPUT);//N3 from Driver
  pinMode(8,OUTPUT);//N4 from driver

  //Register config for PWM generation with Timer0 and Timer2
  TCCR2A = (1<<COM2A1) | (1<<COM2B1) | (1<<WGM21) | (1<<WGM20);
  TCCR2B = (1<<CS22);
  OCR2A = 128; //11
  OCR2B = 128; //3
  TCCR0A=(1<<COM0A1) | (1<<COM0B1) | (1<<WGM01) | (1<<WGM00);
  TCCR0B=(1<<CS01) | (1<<CS00);
  OCR0A=128;//5 sau 6
  OCR0B=128;//5 sau 6
  
  // Arduino initializations
  Wire.begin();
  Serial.begin(115200);//must be the same in combobox in serial monitor window
  
  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,29,0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,26,0x06);
 
  
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  
  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS,0x0A,0x16);
  
   pinMode(13, OUTPUT);
  Timer1.initialize(10000);         // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
  
  
  // Store initial time
  ti=millis();

  //PID
  Input=0;//Input=atan(ax/ay);
  Setpoint=0;
  myPID.SetMode(AUTOMATIC);
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
  //sens de rotatie al fiecarui motor
  digitalWrite(2,HIGH);//N1 from driver
  digitalWrite(4,LOW);//N2 from driver
  digitalWrite(7,HIGH);//N3 from Driver
  digitalWrite(8,LOW);//N4 from driver
  
  while (!intFlag);
  intFlag=false;
  
  // Display time
//  Serial.print (millis()-ti,DEC);
//  Serial.print ("\t");

  
  // _______________
  // ::: Counter :::
  
  // Display data counter
  //Serial.print (cpt++,DEC);
  //Serial.print ("\t"); 
 
  // ____________________________________
  // :::  accelerometer and gyroscope ::: 

  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  
  // Create 16 bits values from 8 bits data
  
  // Accelerometer
  ax=-(Buf[0]<<8 | Buf[1]);
  ay=-(Buf[2]<<8 | Buf[3]);
  az=Buf[4]<<8 | Buf[5];

  // Gyroscope
  int16_t gx=-(Buf[8]<<8 | Buf[9]);
  int16_t gy=-(Buf[10]<<8 | Buf[11]);
  int16_t gz=Buf[12]<<8 | Buf[13];
  
    // Display values
  
  // Accelerometer
  /*
  Serial.print (ax,DEC); 
  Serial.print ("\t");
  Serial.print (ay,DEC);
  Serial.print ("\t");
  Serial.print (az,DEC);  
  Serial.print ("\t");
  */
  // Gyroscope
 /*
  Serial.print (gx,DEC); 
  Serial.print ("\t");
  Serial.print (gy,DEC);
  Serial.print ("\t");
  Serial.print (gz,DEC);  
  Serial.print ("\t");
  */
  
  // _____________________
  // :::  Magnetometer ::: 

  
  // Read register Status 1 and wait for the DRDY: Data Ready
  
  uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));

  // Read magnetometer data  
  uint8_t Mag[7];  
  I2Cread(MAG_ADDRESS,0x03,7,Mag);
  

  // Create 16 bits values from 8 bits data
  
  // Magnetometer
  int16_t mx=-(Mag[3]<<8 | Mag[2]);
  int16_t my=-(Mag[1]<<8 | Mag[0]);
  int16_t mz=-(Mag[5]<<8 | Mag[4]);
  
  
  // Magnetometer
  /*
  Serial.print (mx+200,DEC); 
  Serial.print ("\t");
  Serial.print (my-70,DEC);
  Serial.print ("\t");
  Serial.print (mz-700,DEC);  
  Serial.print ("\t");
  */  
  
  // End of line
  //Serial.println("");

  Input=0;//Input=atan(ax/ay);
  myPID.Compute();
  v1=(int)128+Output;
  v2=(int)128-Output;
  OCR2A=v1;
  OCR2B=v1;
  OCR0A=v2;
  OCR0B=v2;
      
}
