#ifndef YAWCONTROLLER_H
#define YAWCONTROLLER_H

#include <QObject>
#include <QThread>
#include <QTimer>
#include <QDebug>
#include <cmath>

class YawController : public QObject{
  Q_OBJECT
  public:
    YawController(float rate, float Kp, float Ki, float Kd, float maxLimit, float minLimit);
    float angleStoredActual;
    float angleStoredGoal;

  public Q_SLOTS:
    void start();
    void togglePause();

  private:
    // PID controller constants
    float Kp;
    float Ki;
    float Kd;

    // PID Variables
    float angleActual;
    float angleGoal;
    float integral;
    float differential;
    float error;
    float absError;
    float previousError;
    float PTerm;
    float ITerm;
    float DTerm;
    float maxLimit;
    float minLimit;
    float getActual();
    float getGoal();
    bool  goalrcvd;
    unsigned int output;
    unsigned int pwm;

    // Timer Related
    float dt;
    QTimer *timer_;

    // Monitors if process is running
    bool isRunning_;

  private Q_SLOTS:
    void run();
    void setOffset();
    void storeGoal(float goalAngle);
    void storeActual(float actualAngle);

  Q_SIGNALS:
    void getPWM(unsigned int pwm);
};
#endif //YAWCONTROLLER_H


