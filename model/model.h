#ifndef MECHATRONICARMANALYZER_MODEL_H
#define MECHATRONICARMANALYZER_MODEL_H

#define POINT_DIMENSION 6

struct Point {
  double x;
  double y;
  double z;
};
typedef struct Point point_t;

point_t get_end_effector_position(const double theta[POINT_DIMENSION]);

#endif //MECHATRONICARMANALYZER_MODEL_H
