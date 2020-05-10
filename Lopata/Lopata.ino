#include <LIS3MDL.h>
#include <LSM6.h>

#include <Wire.h>

// Create object to work with Accelerometer
LSM6 accelAndGyro;
// Create object to work with Gyroscope
LIS3MDL mag;

#define SAMPLE_FREQ  20.0f   // sample frequency in Hz
#define BETA_DEF  0.14f    // 2 * proportional gain
#define BETA 0.12f
#define BETA_START 1.f
#define epsilon 1e-6

// vars to the data from sensors - gyro, accel and mag
float gx, gy, gz, ax, ay, az, mx, my, mz;
 
// rotation angles
float yaw, pitch, roll;
 
// var for saving frequency of filter work
float fps = 20;

const double compassCalibrationBias[3] = {
  1286.414, -1975.665, 6255.944
};
const double compassCalibrationMatrix[3][3] = {
  {1.916, -0.343, -0.229},
  {0.363, 2.006, -0.127},
  {0.219, -0.02, 1.965}
};

class Madgwick {

public:
    Madgwick();
    void readQuaternions(float *q0, float *q1, float *q2, float *q3);
    void reset();
    void setKoeff(float sampleFreq, float beta);
    void calibrateMatrix(const double calibrationMatrix[3][3], const double bias[3]);
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    //void update(float gx, float gy, float gz, float ax, float ay, float az);
    void update(float &gx, float &gy, float &gz, float &ax, float &ay, float &az);
    float getPitchRad();
    float getRollRad();
    float getYawRad();
    float getPitchDeg();
    float getRollDeg();
    float getYawDeg();
    float read1k();
    float read2k();
    float read3k();
    float read4k();
    void calibrate(LIS3MDL mag);
    void readCalibrateGaussXYZ(float *x, float *y, float *z, LIS3MDL mag);
    void toEulerianAngle(float& roll, float& pitch, float& yaw);

    volatile float _q0 = 1.0f;
    volatile float _q1 = 0.0f;
    volatile float _q2 = 0.0f;
    volatile float _q3 = 0.0f;
private:
    float invSqrt(float x);
    volatile float _beta = BETA_DEF;        // algorithm gain
    volatile float _sampleFreq = SAMPLE_FREQ;
    

    float _xCalibrate;
    float _yCalibrate;
    float _zCalibrate;
    double _calibrationMatrix[3][3];
    double _bias[3];

    int _mult = 6842;
};

Madgwick::Madgwick() {

}

void Madgwick::calibrateMatrix(const double calibrationMatrix[3][3], const double bias[3]) {
    memcpy (_bias, bias, 3 * sizeof (double));
    memcpy (_calibrationMatrix, calibrationMatrix, 3 * 3 * sizeof (double));
}

void Madgwick::setKoeff(float sampleFreq, float beta) {
    _beta = beta;
    _sampleFreq = sampleFreq;
}
void Madgwick::reset() {
    _q0 = 1.0;
    _q1 = 0;
    _q2 = 0;
    _q3 = 0;
}

void Madgwick::readQuaternions(float *q0, float *q1, float *q2, float *q3) {
    *q0 = _q0;
    *q1 = _q1;
    *q2 = _q2;
    *q3 = _q3;
}

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void Madgwick::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _8bx, _8bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((fabs(mx) < epsilon) && (fabs(my) < epsilon) && (fabs(mz) < epsilon)) {
        update(gx, gy, gz, ax, ay, az);
        return;
    }

    // Rate of change of quaternion from accelAndGyroscope
    qDot1 = 0.5f * (-_q1 * gx - _q2 * gy - _q3 * gz);
    qDot2 = 0.5f * (_q0 * gx + _q2 * gz - _q3 * gy);
    qDot3 = 0.5f * (_q0 * gy - _q1 * gz + _q3 * gx);
    qDot4 = 0.5f * (_q0 * gz + _q1 * gy - _q2 * gx);

    // Compute feedback only if accelAndaccelAndGyroerometer measurement valid (avoids NaN in accelAndaccelAndGyroerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {       
        // Normalise accelAndaccelAndGyroerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;   
        
        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * _q0 * mx;
        _2q0my = 2.0f * _q0 * my;
        _2q0mz = 2.0f * _q0 * mz;
        _2q1mx = 2.0f * _q1 * mx;
        _2q0 = 2.0f * _q0;
        _2q1 = 2.0f * _q1;
        _2q2 = 2.0f * _q2;
        _2q3 = 2.0f * _q3;
        _2q0q2 = 2.0f * _q0 * _q2;
        _2q2q3 = 2.0f * _q2 * _q3;
        q0q0 = _q0 * _q0;
        q0q1 = _q0 * _q1;
        q0q2 = _q0 * _q2;
        q0q3 = _q0 * _q3;
        q1q1 = _q1 * _q1;
        q1q2 = _q1 * _q2;
        q1q3 = _q1 * _q3;
        q2q2 = _q2 * _q2;
        q2q3 = _q2 * _q3;
        q3q3 = _q3 * _q3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * _q3 + _2q0mz * _q2 + mx * q1q1 + _2q1 * my * _q2 + _2q1 * mz * _q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * _q3 + my * q0q0 - _2q0mz * _q1 + _2q1mx * _q2 - my * q1q1 + my * q2q2 + _2q2 * mz * _q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * _q2 + _2q0my * _q1 + mz * q0q0 + _2q1mx * _q3 - mz * q1q1 + _2q2 * my * _q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;
        _8bx = 2.0f * _4bx;
        _8bz = 2.0f * _4bz;
        
        // Gradient decent algorithm corrective step
        s0= -_2q2*(2*(q1q3 - q0q2) - ax) + _2q1*(2*(q0q1 + q2q3) - ay) + -_4bz*_q2*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx) + (-_4bx*_q3+_4bz*_q1)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my) + _4bx*_q2*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);

        s1= _2q3*(2*(q1q3 - q0q2) - ax) + _2q0*(2*(q0q1 + q2q3) - ay) + -4*_q1*(2*(0.5 - q1q1 - q2q2) - az) + _4bz*_q3*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx) + (_4bx*_q2+_4bz*_q0)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my) + (_4bx*_q3-_8bz*_q1)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz); 

        s2= -_2q0*(2*(q1q3 - q0q2) - ax) + _2q3*(2*(q0q1 + q2q3) - ay) + (-4*_q2)*(2*(0.5 - q1q1 - q2q2) - az) + (-_8bx*_q2-_4bz*_q0)*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)+(_4bx*_q1+_4bz*_q3)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)+(_4bx*_q0-_8bz*_q2)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);

        s3= _2q1*(2*(q1q3 - q0q2) - ax) + _2q2*(2*(q0q1 + q2q3) - ay)+(-_8bx*_q3+_4bz*_q1)*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)+(-_4bx*_q0+_4bz*_q2)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)+(_4bx*_q1)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);
        
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= _beta * s0;
        qDot2 -= _beta * s1;
        qDot3 -= _beta * s2;
        qDot4 -= _beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    _q0 += qDot1 * (1.0f / _sampleFreq);
    _q1 += qDot2 * (1.0f / _sampleFreq);
    _q2 += qDot3 * (1.0f / _sampleFreq);
    _q3 += qDot4 * (1.0f / _sampleFreq);

    // Normalise quaternion
    recipNorm = invSqrt(_q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);
    _q0 *= recipNorm;
    _q1 *= recipNorm;
    _q2 *= recipNorm;
    _q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void Madgwick::update(float &gx, float &gy, float &gz, float &ax, float &ay, float &az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from accelAndGyroscope
    qDot1 = 0.5f * (-_q1 * gx - _q2 * gy - _q3 * gz);
    qDot2 = 0.5f * (_q0 * gx + _q2 * gz - _q3 * gy);
    qDot3 = 0.5f * (_q0 * gy - _q1 * gz + _q3 * gx);
    qDot4 = 0.5f * (_q0 * gz + _q1 * gy - _q2 * gx);

    // Compute feedback only if accelAndaccelAndGyroerometer measurement valid (avoids NaN in accelAndaccelAndGyroerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelAndaccelAndGyroerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * _q0;
        _2q1 = 2.0f * _q1;
        _2q2 = 2.0f * _q2;
        _2q3 = 2.0f * _q3;
        _4q0 = 4.0f * _q0;
        _4q1 = 4.0f * _q1;
        _4q2 = 4.0f * _q2;
        _8q1 = 8.0f * _q1;
        _8q2 = 8.0f * _q2;
        q0q0 = _q0 * _q0;
        q1q1 = _q1 * _q1;
        q2q2 = _q2 * _q2;
        q3q3 = _q3 * _q3;


        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * _q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * _q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * _q3 - _2q1 * ax + 4.0f * q2q2 * _q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= _beta * s0;
        qDot2 -= _beta * s1;
        qDot3 -= _beta * s2;
        qDot4 -= _beta * s3;
    }


    // Integrate rate of change of quaternion to yield quaternion
    _q0 += qDot1 * (1.0f / _sampleFreq);
    _q1 += qDot2 * (1.0f / _sampleFreq);
    _q2 += qDot3 * (1.0f / _sampleFreq);
    _q3 += qDot4 * (1.0f / _sampleFreq);

    // Normalise quaternion
    recipNorm = invSqrt(_q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);
    _q0 *= recipNorm;
    _q1 *= recipNorm;
    _q2 *= recipNorm;
    _q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float Madgwick::invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

float Madgwick::getYawRad() {
    return atan2(2 * _q1 * _q2 - 2 * _q0 * _q3, 2 * _q0 * _q0 + 2 * _q1 * _q1 - 1);
}

float Madgwick::getPitchRad() {
    return atan2(2 * _q2 * _q3 - 2 * _q0 * _q1, 2 * _q0 * _q0 + 2 * _q3 * _q3 - 1);
}

float Madgwick::getRollRad() {
    return -1 * atan2(2.0f * (_q0 * _q2 - _q1 * _q3), 1.0f - 2.0f * (_q2 * _q2 + _q1 *_q1 ));
}

float Madgwick::getYawDeg() {
    return getYawRad() * RAD_TO_DEG;
}

float Madgwick::getPitchDeg() {
    return getPitchRad() * RAD_TO_DEG;
}

float Madgwick::getRollDeg() {
    return getRollRad() * RAD_TO_DEG;
}

float Madgwick::read1k(){
  return _q0;
}
float Madgwick::read2k(){
  return _q1;
}
float Madgwick::read3k(){
  return _q2;
}
float Madgwick::read4k(){
  return _q3;
}

void Madgwick::calibrate(LIS3MDL mag) {
    float result[3] = {0, 0, 0};
    float uncalibratedValues[3];
    mag.read();
    uncalibratedValues[0] = mag.m.x - _bias[0];
    uncalibratedValues[1] = mag.m.y - _bias[1];
    uncalibratedValues[2] = mag.m.z - _bias[2];

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
        result[i] += _calibrationMatrix[i][j] * uncalibratedValues[j];
        }
    }

    _xCalibrate = result[0];
    _yCalibrate = result[1];
    _zCalibrate = result[2];
}

void Madgwick::readCalibrateGaussXYZ(float *x, float *y, float *z, LIS3MDL mag) {
    calibrate(mag);
    *x = _xCalibrate / _mult;
    *y = _yCalibrate / _mult;
    *z = _zCalibrate / _mult;
}

void Madgwick::toEulerianAngle(float& roll, float& pitch, float& yaw)
{
  double ysqr = _q2 * _q2;

  // roll (x-axis rotation)
  double t0 = +2.0 * (_q0 * _q1 + _q2 * _q3);
  double t1 = +1.0 - 2.0 * (_q1 * _q1 + ysqr);
  roll = atan2(t0, t1) / 3.1415 * 180.0;

  // pitch (y-axis rotation)
  double t2 = +2.0 * (_q0 * _q2 - _q3 * _q1);
  t2 = ((t2 > 1.0) ? 1.0 : t2);
  t2 = ((t2 < -1.0) ? -1.0 : t2);
  pitch = asin(t2) / 3.1415 * 180.0;

  // yaw (z-axis rotation)
  double t3 = +2.0 * (_q0 * _q3 + _q1 * _q2);
  double t4 = +1.0 - 2.0 * (ysqr + _q3 * _q3);  
  yaw = atan2(t3, t4) / 3.1415 * 180.0;
}

Madgwick filter;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  //Serial.begin(9600);
//  Wire.begin();
  Serial.println("Pololu MinIMU-9");
  accelAndGyro.init();
  accelAndGyro.enableDefault();
  accelAndGyro.writeReg(LSM6::CTRL1_XL, 0x8C); // 52 Hz, 8 g full scale
  // accelAndGyro_Init() should have already called accelAndGyro.init() and enableDefault()
  accelAndGyro.writeReg(LSM6::CTRL2_G, 0x8C); // 104 Hz, 2000 dps full scale
  
  mag.init();
  mag.enableDefault();

  filter.calibrateMatrix(compassCalibrationMatrix, compassCalibrationBias);
  
  // Print message about sucessfull initialization
  Serial.println("Initialization completed");
}

unsigned long t = 0;
bool flag = 1;
int NUM_DIGITS = 5;
void loop()
{
  unsigned long startMillis = millis();
  accelAndGyro.readAcc();
  // Read data from Acceleroneter in G
  ax = accelAndGyro.a.x * 0.000061;
  ay = accelAndGyro.a.y * 0.000061;
  az = accelAndGyro.a.z * 0.000061;
  accelAndGyro.readGyro();

  gx = accelAndGyro.g.x * 0.00122173;
  gy = accelAndGyro.g.y * 0.00122173;
  gz = accelAndGyro.g.z * 0.00122173;

//  Serial.print(ax, NUM_DIGITS);
//  Serial.print("\t");
//
//  Serial.print(ay, NUM_DIGITS);
//  Serial.print("\t");
//
//  Serial.print(az, NUM_DIGITS);
//  Serial.print("\n");
  
  if (flag == 1){
    filter.setKoeff(fps, BETA_START);
  }
  else {
    filter.setKoeff(fps, BETA);
  }

  // Update input data to the filter
  filter.readCalibrateGaussXYZ(&mx, &my, &mz, mag);
  filter.update(gx, gy, gz, ax, ay, az);
  filter.toEulerianAngle(roll, pitch, yaw);

  Serial.print(int(filter.getRollDeg()));
  Serial.print("\t");

  Serial.print(int(filter.getPitchDeg()));
  Serial.print("\t");

  Serial.print(int(filter.getYawDeg()));
  Serial.print("\n");


  
//  Serial.print(filter.read1k());
//  Serial.print("\t");
//  
//  Serial.print(filter.read2k());
//  Serial.print("\t");
//  
//  Serial.print(filter.read3k());
//  Serial.print("\t");
//
//  Serial.print(filter.read4k());  
//  Serial.print("\n");
  
  // Calculate time for data processing
  unsigned long deltaMillis = millis() - startMillis;
  // Calculate filter frequency
  fps = 1000 / deltaMillis;
  if (t > 3000){
    flag = 0;
  }
  else {
      t += deltaMillis;
  }
  
  
}
