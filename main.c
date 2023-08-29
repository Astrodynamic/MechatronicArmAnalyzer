#include "model.h"

int main(int argc, char** argv) {
    MAT_1X6(double, theta) = {10.0, -50.0, -60.0, 90.0, 50.0, 0.0};
    print_point(get_coord_after_transform(theta));
    return 0;
}
