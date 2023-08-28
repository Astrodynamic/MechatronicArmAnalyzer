#include "model.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

const MAT_1X6(joint_t, DH_PARAMS) = {
  {+0.0, +0.0,   +0.21,  +M_PI_2},
  {+0.0, -0.8,   +0.193, +0.0},
  {+0.0, -0.598, +0.16,  +0.0},
  {+0.0, +0.0,   +0.25,  +M_PI_2},
  {+0.0, +0.0,   +0.25,  -M_PI_2},
  {+0.0, +0.0,   +0.25,  +0.0}
};

void mat_identify(MAT_4x4(mat)) {
    memset(mat, 0, sizeof(double) * MATRIX_DIMENSION * MATRIX_DIMENSION);
    for (int i = 0; i < MATRIX_DIMENSION; ++i) {
        mat[i][i] = 1.0;
    }
}

void mat_mul(const MAT_4x4(mat_1), const MAT_4x4(mat_2), MAT_4x4(result)) {
    memset(result, 0, sizeof(double) * MATRIX_DIMENSION * MATRIX_DIMENSION);
    for (int i = 0; i < MATRIX_DIMENSION; ++i) {
        for (int j = 0; j < MATRIX_DIMENSION; ++j) {
            for (int k = 0; k < MATRIX_DIMENSION; ++k) {
                result[i][j] += mat_1[i][k] * mat_2[k][j];
            }
        }
    }
}

void set_dh_params(const joint_t params, MAT_4x4(result)) {
    const double cTheta = cos(params.theta);
    const double sTheta = sin(params.theta);
    const double cAlpha = cos(params.alpha);
    const double sAlpha = sin(params.alpha);

    result[0][0] = cTheta;
    result[0][1] = -sTheta * cAlpha;
    result[0][2] = sTheta * sAlpha;
    result[0][3] = params.r * cTheta;
    
    result[1][0] = sTheta;
    result[1][1] = cTheta * cAlpha;
    result[1][2] = -cTheta * sAlpha;
    result[1][3] = params.r * sTheta;
    
    result[2][0] = 0.0;
    result[2][1] = sAlpha;
    result[2][2] = cAlpha;
    result[2][3] = params.d;
    
    result[3][0] = 0.0;
    result[3][1] = 0.0;
    result[3][2] = 0.0;
    result[3][3] = 1.0;
}

point_t get_coord_after_transform(const MAT_1X6(double, theta)) {
    MAT_4x4(base);
    MAT_4x4(result);
    MAT_4x4(transform);
    mat_identify(base);

    for (int stage = 0; stage < HAND_DIMENSION; ++stage) {
        set_dh_params(DH_PARAMS[stage], transform);
        mat_mul(base, transform, result);
        memcpy(base, result, sizeof(double) * MATRIX_DIMENSION * MATRIX_DIMENSION);
    }
    point_t point = {result[0][3], result[1][3], result[2][3]};
    return point;
}

void print_point(const point_t point) {
    printf("End Effector Position: (%lf, %lf, %lf)\n", point.x, point.y, point.z);
}

int main(int argc, char** argv) {
    MAT_1X6(double, theta) = {10.0, -50.0, -60.0, 90.0, 50.0, 0.0};
    print_point(get_coord_after_transform(theta));
    return 0;
}
