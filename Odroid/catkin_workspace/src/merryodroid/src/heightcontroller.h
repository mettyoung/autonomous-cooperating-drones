#ifndef HEIGHTCONTROLLER_H
#define HEIGHTCONTROLLER_H

#include <QObject>
#include <QThread>
#include <QTimer>
#include <QDebug>
#include <cmath>

class HeightController : public QObject{
  Q_OBJECT
  public:
    HeightController(float rate, float Kp, float Ki, float Kd, float maxLimit, float minLimit);
    float myStoredActual;
    float myStoredGoal;

  public Q_SLOTS:
    void start();
    void togglePause();

  private:
    // PID controller constants
    float Kp;
    float Ki;
    float Kd;

    // PID Variables
    float myActual;
    float myGoal;
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
    void storeGoal(float myGoal);
    void storeActual(float myActual);

  Q_SIGNALS:
    void getPWM(unsigned int pwm);
};
#endif //HEIGHTCONTROLLER_H


