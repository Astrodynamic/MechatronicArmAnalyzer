#ifndef MECHATRONICARMANALYZER_MODEL_H
#define MECHATRONICARMANALYZER_MODEL_H

#define POINT_DIMENSION 6
#define Vector1x6(T, V) T V[POINT_DIMENSION]

struct Point {
  double x;
  double y;
  double z;
};
typedef struct Point point_t;

point_t get_end_effector_position(Vector1x6(const double, theta));

#endif //MECHATRONICARMANALYZER_MODEL_H
