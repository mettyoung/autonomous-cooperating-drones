#include "prcontroller.h"
#include <iostream>
PRController::PRController(float rate, float Kp, float Ki, float Kd, float maxLimit, float minLimit)
{
  this->myError = 0;
  this->myStoredError = 0;
  this->maxLimit = maxLimit;
  this->minLimit = minLimit;
  this->dt = 1/rate;
  this->previousError = 0;
  this->integral = 0;
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PRController::start()
{
  timer_ = new QTimer();
  timer_->start((int)(dt*1000));
  connect(timer_, SIGNAL(timeout()), this, SLOT(run()));
}

float PRController::getError()
{
    return myStoredError;
}

void PRController::storeError(float myError)
{
    this->myStoredError = myError;
    integral = 0;
}

void PRController::setOffset()
{
    integral = 0;
    differential = 0;
}

void PRController::run()
{
    //Values for PID Controller
    myError = getError();

    //Setup for PID Controller
    integral += (myError * dt);
    differential = (previousError - myError)/dt;

    //PID Controller Terms
    PTerm = Kp * myError;
    ITerm = Ki * integral;
    DTerm = Kd * differential;

    //PID Controller
    output = 1500. + PTerm + ITerm + DTerm;
//    std::cerr << "MinLimit = " << minLimit << ";MaxLimit = " << maxLimit << std::endl;
//    std::cerr << "[Output, PTerm, ITerm, DTerm] = " << output << ", " << PTerm << ", " << ITerm << ", " << DTerm << std::endl;
    if(output > maxLimit)
    output = maxLimit;
    else if(output < minLimit)
    output = minLimit;
    previousError = myError;

    float sum = output - 1500.;
    //PID Output
    //qDebug() << "goal :" << myGoal << "\tactual : " << myActual << "\tError : " << error << "\tOutput : " << output;
    Q_EMIT pTerm(PTerm);
    Q_EMIT iTerm(ITerm);
    Q_EMIT dTerm(DTerm);
    Q_EMIT sOutput(sum);
    Q_EMIT getPWM(output);
}

void PRController::togglePause()
{
    isRunning_ = !isRunning_;
}


