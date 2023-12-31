#include "yawcontroller.h"

// Define only if yaw data of IMU emmits 0 to 180 to -180 to -0
// in a 360 rotation around the z axis.
// to avoid taking the long path to reach a goal due to the sudden
// change of sign in the error.
#define MAX_IMU_DATA_180

YawController::YawController(float rate, float Kp, float Ki, float Kd, float maxLimit, float minLimit)
{
  this->angleGoal = 0;
  this->angleActual = 0;
  this->angleStoredActual = 0;
  this->angleStoredGoal = 0;
  this->maxLimit = maxLimit;
  this->minLimit = minLimit;
  this->dt = 1/rate;
  this->previousError = 0;
  this->integral = 0;
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->goalrcvd = true;
}

void YawController::start()
{
  timer_ = new QTimer();
  timer_->start((int)(dt*1000));
  connect(timer_, SIGNAL(timeout()), this, SLOT(run()));
}

float YawController::getActual()
{
    return angleStoredActual;
}

// Called everytime the IMU passes data.
void YawController::storeActual(float actualAngle)
{
    this->angleStoredActual = actualAngle;
}

// Angle goal is relative to current angle. E.g: At angle 90, requesting a
// goal of 0 degrees would result at a goal of 90. On the other hand at angle 30,
// requesting a goal of 45 degrees would result at a goal of 75.
float YawController::getGoal()
{
    angleStoredGoal = angleStoredGoal + this->angleActual;
    return angleStoredGoal;
}

// Stores the goal in variable angleStoredGoal
// to be called only once after a goal is sent.
void YawController::storeGoal(float goalAngle)
{
    this->angleStoredGoal = goalAngle;
    this->goalrcvd = true;
    integral = 0;
}

void YawController::setOffset()
{
    integral = 0;
    differential = 0;
}

void YawController::run()
{
    //Values for PID Controller
    angleActual = getActual();
    if(goalrcvd)
    angleGoal = getGoal();

    //Setup for PID Controller
    error = angleGoal - angleActual;
    #ifdef MAX_IMU_DATA_180
    if(error >= 180)
    error -= 360;
    else if(error <= -180)
    error += 360;
    #endif

    absError = std::abs(error);
    integral += (error * dt);
    differential = (previousError - error)/dt;

    //PID Controller Terms
    PTerm = Kp * error;
    ITerm = Ki * integral;
    DTerm = Kd * differential;

    //PID Controller
    output = 1500 + PTerm + ITerm + DTerm;
    if(output > maxLimit)
    output = maxLimit;
    else if(output < minLimit)
    output = minLimit;
    previousError = error;
    goalrcvd = false;

    //PID Output
    //qDebug() << "goal :" << angleGoal << "\tactual : " << angleActual << "\tError : " << error << "\tOutput : " << output;
    Q_EMIT getPWM(output);
}

void YawController::togglePause()
{
    isRunning_ = !isRunning_;
}


