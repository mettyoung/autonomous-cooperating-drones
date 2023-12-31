#include "heightcontroller.h"

HeightController::HeightController(float rate, float Kp, float Ki, float Kd, float maxLimit, float minLimit)
{
  this->myGoal = 0;
  this->myActual = 0;
  this->myStoredActual = 0;
  this->myStoredGoal = 0;
  this->maxLimit = maxLimit;
  this->minLimit = minLimit;
  this->dt = 1/rate;
  this->previousError = 0;
  this->integral = 0;
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void HeightController::start()
{
  timer_ = new QTimer();
  timer_->start((int)(dt*1000));
  connect(timer_, SIGNAL(timeout()), this, SLOT(run()));
}

float HeightController::getActual()
{
    return myStoredActual;
}

void HeightController::storeActual(float myActual)
{
    this->myStoredActual = myActual;
}

float HeightController::getGoal()
{
    return myStoredGoal;
}

void HeightController::storeGoal(float myGoal)
{
qDebug() << "Height Controller received goal: " << myGoal;
    this->myStoredGoal = myGoal;
    integral = 0;
}

void HeightController::setOffset()
{
    integral = 0;
    differential = 0;
}

void HeightController::run()
{
    //Values for PID Controller
    myActual = getActual();
    myGoal = getGoal();

    //Setup for PID Controller
    error = myGoal - myActual;
    integral += (error * dt);
    differential = (error - previousError)/dt;

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

//    qDebug() << output << " = 1500 " << PTerm << " + " << ITerm << " + "  << DTerm;
    //PID Output
    Q_EMIT getPWM(output);
}

void HeightController::togglePause()
{
    isRunning_ = !isRunning_;
}


