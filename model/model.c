#include "model.h"

#include <string.h>
#include <math.h>

#define MATRIX_SIZE 4
#define Matrix4x4(V) double V[MATRIX_SIZE][MATRIX_SIZE]
#define MATRIX4X4_BITS (sizeof(double) * MATRIX_SIZE * MATRIX_SIZE)

struct DHParameters {
  double theta;
  double a;
  double d;
  double alpha;
};
typedef struct DHParameters dhparameters_t;

static dhparameters_t DH_PARAMS[POINT_DIMENSION] = {
  {+0.0, +0.0,   +0.21,  +M_PI_2},
  {+0.0, -0.8,   +0.193, +0.0},
  {+0.0, -0.598, +0.16,  +0.0},
  {+0.0, +0.0,   +0.25,  +M_PI_2},
  {+0.0, +0.0,   +0.25,  -M_PI_2},
  {+0.0, +0.0,   +0.25,  +0.0}
};

static double radians(double deg) {
    return deg * M_PI / 180.0;
}

static void mat_identify(Matrix4x4(mat)) {
    memset(mat, 0, MATRIX4X4_BITS);
    for (int i = 0; i < MATRIX_SIZE; ++i) {
        mat[i][i] = 1.0;
    }
}

static void mat_mul(Matrix4x4(mat_1), Matrix4x4(mat_2), Matrix4x4(result)) {
    memset(result, 0, MATRIX4X4_BITS);
    for (int i = 0; i < MATRIX_SIZE; ++i) {
        for (int j = 0; j < MATRIX_SIZE; ++j) {
            for (int k = 0; k < MATRIX_SIZE; ++k) {
                result[i][j] += mat_1[i][k] * mat_2[k][j];
            }
        }
    }
}

static void compute_dh_transformation_matrix(const dhparameters_t* const params, Matrix4x4(result)) {
    const double cTheta = cos(radians(params->theta));
    const double sTheta = sin(radians(params->theta));
    const double cAlpha = cos(params->alpha);
    const double sAlpha = sin(params->alpha);

    result[0][0] = cTheta;
    result[0][1] = -sTheta * cAlpha;
    result[0][2] = sTheta * sAlpha;
    result[0][3] = params->a * cTheta;
    
    result[1][0] = sTheta;
    result[1][1] = cTheta * cAlpha;
    result[1][2] = -cTheta * sAlpha;
    result[1][3] = params->a * sTheta;
    
    result[2][0] = 0.0;
    result[2][1] = sAlpha;
    result[2][2] = cAlpha;
    result[2][3] = params->d;
    
    result[3][0] = 0.0;
    result[3][1] = 0.0;
    result[3][2] = 0.0;
    result[3][3] = 1.0;
}

point_t get_end_effector_position(const double theta[POINT_DIMENSION]) {
    Matrix4x4(base);
    Matrix4x4(result);
    Matrix4x4(transform);
    mat_identify(base);

    for (int stage = 0; stage < POINT_DIMENSION; ++stage) {
        DH_PARAMS[stage].theta = theta[stage];
        compute_dh_transformation_matrix(&DH_PARAMS[stage], transform);
        mat_mul(base, transform, result);
        memcpy(base, result, MATRIX4X4_BITS);
    }
    point_t point = {result[0][3], result[1][3], result[2][3]};
    return point;
}
