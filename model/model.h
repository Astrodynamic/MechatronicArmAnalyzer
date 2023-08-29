#ifndef MECHATRONICARMANALYZER_MODEL_H
#define MECHATRONICARMANALYZER_MODEL_H

#define HAND_DIMENSION 6
#define MATRIX_DIMENSION 4
#define PARAMETER 4

#define MAT_4x4(V) double V[MATRIX_DIMENSION][MATRIX_DIMENSION]
#define MAT_1X6(T, V) T V[HAND_DIMENSION]

struct Point {
  double x;
  double y;
  double z;
};
typedef struct Point point_t;

struct Joint {
  double theta;
  double d;
  double alpha;
  double r;
};
typedef struct Joint joint_t;

point_t get_coord_after_transform(const MAT_1X6(double, theta));
void print_point(const point_t point);

#endif //MECHATRONICARMANALYZER_MODEL_H
