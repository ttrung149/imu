#include <Arduino.h>
#include <Wire.h>

#define MPU6050_ADDR 0x68
#define LOOP_PERIOD 4000

static int16_t acx, acy, acz, tmp, gyx, gyy, gyz;
static int16_t offset_gyx = 0, offset_gyy = 0, offset_gyz = 0;
static uint32_t _micros;

static float degrees_pitch_acc, degrees_roll_acc;
static float acc_vector;
static float degrees_pitch = 0, degrees_roll = 0;

void read_MPU6050() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(MPU6050_ADDR, 14);

    // Fetches accelerometer values
    acx = Wire.read() << 8 | Wire.read();
    acy = Wire.read() << 8 | Wire.read();
    acz = Wire.read() << 8 | Wire.read();

    // Gets temperature value
    tmp = Wire.read() << 8 | Wire.read();

    // Fetches gyro values
    gyx = Wire.read() << 8 | Wire.read();
    gyy = Wire.read() << 8 | Wire.read(); 
    gyz = Wire.read() << 8 | Wire.read(); 
}

void calibirate_MPU6050(int num_iterations) {
    Serial.println("Calibrating MPU6050");
    for (int i = 0; i < num_iterations; i++) {
        if (i % 100 == 0) Serial.print(".");
        read_MPU6050();
        offset_gyx += gyx;
        offset_gyy += gyy;
        offset_gyz += gyz;
    }

    offset_gyx /= num_iterations;
    offset_gyy /= num_iterations;
    offset_gyz /= num_iterations;
}

void setup(){
    Wire.begin();
    Wire.setClock(400000);
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);                      // PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                      // Activate the gyro.
    Wire.endTransmission(); 
   
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B);                      // GYRO_CONFIG register (1B hex)
    Wire.write(0x08);                      // Set 500dps full scale
    Wire.endTransmission();
   
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1C);                      // ACCEL_CONFIG register (1A hex)
    Wire.write(0x10);                      // (+/- 8g full scale range)
    Wire.endTransmission();
    
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1A);                      // CONFIG register (1A hex)
    Wire.write(0x03);                      // Set Digital LPF to ~43Hz
    Wire.endTransmission();   

    Serial.begin(115200);
    
    calibirate_MPU6050(3000);
    _micros = micros();
}

void loop(){
    read_MPU6050();
    gyx -= offset_gyx;
    gyy -= offset_gyy;
    gyz -= offset_gyz;

    degrees_pitch += gyy * 0.0000610687;
    degrees_roll  += gyx * 0.0000610687;
    // degrees_yaw   += gyz * 0.0000610687;

    degrees_pitch += degrees_roll * sin(gyz * 0.000001066);
    degrees_roll  -= degrees_pitch * sin(gyz * 0.000001066);
    
    acc_vector = sqrt((acx * acx) + (acy * acy) + (acz * acz));
    degrees_pitch_acc = asin((float) acy/acc_vector) * 57.2957795;
    degrees_roll_acc  = asin((float) acx/acc_vector) * -57.2957795;
    
    // degrees_yaw_acc = asin((float) acz/acc_vector) * 57.2957795;

    degrees_pitch = degrees_pitch * 0.97 + degrees_pitch_acc * 0.03;
    degrees_roll  = degrees_roll * 0.97 + degrees_roll_acc * 0.03;
    // degrees_yaw  = degrees_yaw * 0.97 + degrees_yaw_acc * 0.03;

    Serial.print("Pitch: "); Serial.print(degrees_pitch);
    Serial.print(" | Roll: "); Serial.println(degrees_roll);
    // Serial.print(" | Yaw: "); Serial.println(degrees_yaw);
    
    while(micros() < _micros + LOOP_PERIOD);
    _micros = micros();  
}
