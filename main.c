#include <stdio.h>

#include "model.h"

int main(int argc, char** argv) {
    Vector1x6(double, theta) = {10.0, -50.0, -60.0, 90.0, 50.0, 0.0};

    point_t position = get_end_effector_position(theta);
    printf("End Effector Position: (%lf, %lf, %lf)\n", position.x, position.y, position.z);
    return 0;
}
