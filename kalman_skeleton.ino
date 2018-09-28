#include "motor_driver.h"
#include "imu.h"

//prescribed oscillation
#define AMPLITUDE 90    //degrees
#define FREQ 0.5        //Hz

//VALUES TO BE SET BY THE STUDENT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define CONVERT_GYRO_TO_DEG_PER_MS  0 //(SET BY STUDENT -- CONSULT TABLE 3 IN DATASHEET FOR SENSISTIVITY)
#define DEGREES_PER_TICK            0 //(SET BY STUDENT -- CONSULT WEB PAGE FOR THE MOTOR (search: "Pololu 25mm gear motor")

//PID coeffs
#define K_P   1    //(SET BY STUDENT)
#define K_I   0    //(SET BY STUDENT)
#define K_D   0    //(SET BY STUDENT)

MotorDriver motor(DIR_PIN_1, DIR_PIN_2, PWM_PIN); //pins and methods are defined in motor_driver.h -- change them if you want

void setup()
{
  SerialUSB.begin(115200);
  while (!SerialUSB) {} // Wait for Serial monitor to open

  SerialUSB.println("Initializing..."); //let us know we're talking

  //initialize the motor driver
  motor.Init(); //see motor_driver.h

  //initialize the IMU
  if (!SetupIMU()) //see imu.h for details of what this does
  {
    SerialUSB.println("Failed to communicate with LSM9DS1.");
    while (1) {}  //die gracefully
  }
  
  SerialUSB.println("Done initializing.");
}

void loop(void)
{
  //if there is new gyro data, do the PID loop
  //that is, the control loop is dependent on the rate to which you set the gyro
  if(imu.gyroAvailable())
  {
    //keep track of time for each loop -- NOTE THAT TIME IS IN MSEC
    static uint32_t prevTime = 0;
    uint32_t currTime = millis();
    uint32_t deltaT = currTime - prevTime;
    prevTime = currTime;
    
    //read accelerometer and gyro -- see the library for how these work
    imu.readGyro();
    imu.readAccel();

    //do maths on IMU
    float accelAngle = atan2(imu.ax, imu.az); //you'll need to figure out which axes you want to use!!!!!!!!!!!!!!!

    static float gyroAngle = 0;
    
    gyroAngularVelocity = imu.gy * CONVERT_GYRO_TO_DEG_PER_MS; 
    gyroAngle += gyroAngularVelocity * (float)deltaT;
    
    /////////////////////
    float kalmanAngle = 0; //this is just a place holder for kalmanAngle -- you'll need to add all the code for the KF
    /////////////////////

    //do PID
    //calc target angle
    float targetAngle = AMPLITUDE * sin(2.0 * PI * FREQ * currTime / 1000.0); //currTime is in ms
    float actualAngle = encoderCount * DEGREES_PER_TICK;
    float error = targetAngle - actualAngle;
    float effort = K_P * error;
    
    //Integral and derivative control are left as an exercise for the student

    motor.SetPower(effort); //see motor_driver.h for implementation

    SerialUSB.print(currTime);
    SerialUSB.print('\t');
    SerialUSB.print(targetAngle);
    SerialUSB.print('\t');
    SerialUSB.print(actualAngle);
    SerialUSB.print('\t');
    SerialUSB.print(accelAngle);
    SerialUSB.print('\t');
    SerialUSB.print(gyroAngle);
    SerialUSB.print('\t');
    SerialUSB.print(kalmanAngle);
    SerialUSB.print('\n');
  }
}

