/**
    S-Curve Motion Profile Generator
    @author Thinh Lu
    @version 1.0 07/10/20
*/

#ifndef S_CURVE_H
#define S_CURVE_H

class SCurveGenerator {
  public:

  // Constructors
  SCurveGenerator(double v0, double vs, double a_s, double j_max);
  virtual ~SCurveGenerator();

  // Functions
  double s(double t);
  double v(double t);
  double a(double t);

  // Attributes	
  double v0;
  double vs;
  double a_s;
  double j_max;

  // Pre-calculated Properties
  double T, t1, v1, s1, v2, s2;
};

#endif
