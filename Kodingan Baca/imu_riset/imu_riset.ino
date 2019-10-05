#include <Wire.h>
#include <math.h>

#define IMU_ADDRESS 0x68
#define NUM_DATA_OF_IMU 5
#define FREQ_IMU 1
#define PI 3.14159265358979
#define COMP_FILTER_CONST 0.986

uint16_t accel_xout[NUM_DATA_OF_IMU];
uint16_t accel_yout[NUM_DATA_OF_IMU];
uint16_t accel_zout[NUM_DATA_OF_IMU];

uint16_t temp_out[NUM_DATA_OF_IMU];

uint64_t tnow = 0.0;
uint64_t tlast = 0.0;
uint16_t temp = 0.0;
uint16_t Sum_temp = 0.0;
uint16_t Avg_temp = 0.0;
uint16_t Att_acc_pitch = 0.0;
uint16_t Att_acc_roll = 0.0; 
uint16_t Ang_vel_x = 0.0;
uint16_t Ang_vel_y = 0.0;
int n = 0;

//struct imu 
struct imu {
  uint16_t x;
  uint16_t y;
  uint16_t z;
};

imu Accel;
imu Gyro;
imu Sum_acc;
imu Avg_acc;
imu Ang_vel_gyro;

//struct Attitude
struct att {
  uint32_t roll;
  uint32_t pitch;
  uint32_t yaw;
};

att Att;

void setup() {

  // put your setup code here, to run once:
  Wire.begin();
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  tnow = micros();
  if (tnow - tlast >= 1000/FREQ_IMU) 
  {
    //Request data
    Wire.beginTransmission(IMU_ADDRESS);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(IMU_ADDRESS,14);
    
    //Assign data ke struct
    Accel.x = Wire.read() << 8 | Wire.read(); // 0x3B (8 bit left shifted) | 0x3C
    Accel.y = Wire.read() << 8 | Wire.read();  
    Accel.z = Wire.read() << 8 | Wire.read();
    temp = Wire.read() << 8 | Wire.read();
    Gyro.x = Wire.read() << 8 | Wire.read();
    Gyro.y = Wire.read() << 8 | Wire.read();
    Gyro.z = Wire.read() << 8 | Wire.read();

    //offset gyro
    Gyro.x -= 0.56;
    Gyro.y -= 2;
    Gyro.z -= 0.79;
    
    //Assign data ke array
    accel_xout[n] = Accel.x;
    accel_yout[n] = Accel.y;
    accel_zout[n] = Accel.z;

    temp_out[n] = temp;
    n += 1;

    for (int i = 0; i < NUM_DATA_OF_IMU; i++) 
    {
      Sum_acc.x += accel_xout[i];
      Sum_acc.y += accel_yout[i];
      Sum_acc.z += accel_zout[i];
      Sum_temp += temp_out[i];
    }
    if (n = NUM_DATA_OF_IMU) 
    {
      n = 0;
    }
    //accelerometer
    Avg_acc.x = Sum_acc.x/NUM_DATA_OF_IMU;
    Avg_acc.y = Sum_acc.y/NUM_DATA_OF_IMU;
    Avg_acc.z = Sum_acc.z/NUM_DATA_OF_IMU;
    Avg_temp = Sum_temp/NUM_DATA_OF_IMU;

    //kalkulasi roll, pitch dari accelerometer
    Att_acc_pitch = atan2(Avg_acc.y, sqrt(Avg_acc.x*Avg_acc.x + Avg_acc.z*Avg_acc.z))*180.0/PI;
    Att_acc_roll = atan2(Avg_acc.x, Avg_acc.z)*180.0/PI;

    //gyroscope
    Ang_vel_x = Gyro.x/131.0; //roll
    Ang_vel_y = Gyro.y/131.0; //pitch
    Att.yaw = Gyro.z/131.0;

    //complementary filter
    Att.roll = COMP_FILTER_CONST * (Ang_vel_x * (1.0/FREQ_IMU)) + (1-COMP_FILTER_CONST) * Att_acc_roll;
    Att.pitch = COMP_FILTER_CONST * (Ang_vel_y * (1.0/FREQ_IMU)) + (1-COMP_FILTER_CONST) * Att_acc_pitch;

    /*
    Serial.print("acc_x : "); Serial.print(Avg_acc.x); Serial.print(" | ");
    Serial.print("acc_y : "); Serial.print(Avg_acc.y); Serial.print(" | ");
    Serial.print("acc_z : "); Serial.print(Avg_acc.z); Serial.print(" | ");
    Serial.print("temp : "); Serial.print(Avg_temp); Serial.print(" | ");
    Serial.print("gyro_x: "); Serial.print(Avg_gyro.x); Serial.print(" | ");
    Serial.print("gyro_y : "); Serial.print(Avg_gyro.y); Serial.print(" | ");
    Serial.print("gyro_z : "); Serial.print(Avg_gyro.z); Serial.println(" | ");
    
    //print nilai roll dan pitch
    Serial.print("Roll : "); Serial.println(Att.roll);
    Serial.print("Pitch : "); Serial.println(Att.pitch);
    */
    //Serial.print("Roll : "); Serial.println(Att.roll);
    Serial.print("Pitch : "); Serial.println(Att.pitch);
    //Serial.print("Yaw : "); Serial.println(Att.yaw);

    //reset nilai penjumlahannya untuk tiap loopnya
    Sum_acc.x = 0.0;
    Sum_acc.y = 0.0;
    Sum_acc.z = 0.0;
    Sum_temp = 0.0;

    tlast = tnow;
  }
}
