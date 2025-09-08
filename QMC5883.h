#ifndef QMC5883_H
#define QMC5883_H

#include <Arduino.h>

class QMC5883{
  public:
    QMC5883(float min_Mx, float max_Mx, float min_My, float max_My, float min_Mz, float max_Mz);
    
    void Setup();
    void Read();
    void ReadRaw();
    float GetMx();
    float GetMy();
    float GetMz();
  private:
    float Mx;
    float My;
    float Mz;
    float min_Mx;
    float min_My;
    float min_Mz;
    float max_Mx;
    float max_My;
    float max_Mz;
};

#endif