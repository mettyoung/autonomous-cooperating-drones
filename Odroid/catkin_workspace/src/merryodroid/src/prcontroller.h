#ifndef PRCONTROLLER_H
#define PRCONTROLLER_H

#include <QObject>
#include <QThread>
#include <QTimer>
#include <QDebug>
#include <cmath>

class PRController : public QObject{
  Q_OBJECT
  public:
    PRController(float rate, float Kp, float Ki, float Kd, float maxLimit, float minLimit);
    float myStoredError;

  public Q_SLOTS:
    void start();
    void togglePause();

  private:
    // PID controller constants
    float Kp;
    float Ki;
    float Kd;

    // PID Variables
    float myError;
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
    float getError();
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
    void storeError(float myGoal);

  Q_SIGNALS:
    void getPWM(unsigned int pwm);
    void pTerm(float pterm);
    void iTerm(float iterm);
    void dTerm(float dterm);
    void sOutput(float output);
};
#endif //PRCONTROLLER_H


