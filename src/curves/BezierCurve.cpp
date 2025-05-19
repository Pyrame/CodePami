#include "curves/BezierCurve.h"
#include "Position.h"
#include "Integrator.h"

BezierCurve::BezierCurve(Position start, Position end, PRECISION_DATA_TYPE multiplier) : Curve(0.0f,1.0f){
      pos1 = start;
      pos2 = start.getSinCosAngle()*multiplier/3 + start;
      pos3 = end - end.getSinCosAngle()*multiplier/3;
      pos4 = end;
#ifdef DEBUG_TXT
      Serial.print("NEW BEZIER CURVE, pos1 : ");
      Serial.print(pos1);
      Serial.print(" , pos2 : ");
      Serial.print(pos2);
      Serial.print(" , pos3 : ");
      Serial.print(pos3);
      Serial.print(" , pos4 : ");
      Serial.println(pos4);
#endif
}

Position BezierCurve::getPosition(PRECISION_DATA_TYPE time) const{
  return pos1 * pow(1-time, 3) + pos2 * (3*pow(1-time, 2)*time) + pos3 * (3*(1-time)*pow(time, 2)) + pos4 * pow(time, 3);
}

Position BezierCurve::getDerivative(PRECISION_DATA_TYPE t) const {
    return pos1 *(-3)* pow(1-t, 2) + pos2 * (-6*(1-t)*t + 3 *pow(1-t, 2)) + pos3 * (3*(-pow(t, 2) + 2*t*(1-t))) + pos4 * 3 * pow(t,2);
}
