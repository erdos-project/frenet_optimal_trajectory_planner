#include "FrenetOptimalTrajectory.h"
#include "FrenetPath.h"
#include "py_cpp_struct.h"

#include <iostream>

using namespace std;

int main() {
    double wx [25] = {132.67, 128.67, 124.67, 120.67, 116.67, 112.67, 108.67,
                   104.67, 101.43,  97.77,  94.84,  92.89,  92.4 ,  92.4 ,
                   92.4 ,  92.4 ,  92.4 ,  92.4 ,  92.4 ,  92.39,  92.39,
                   92.39,  92.39,  92.39,  92.39};
    double wy [25] = {195.14, 195.14, 195.14, 195.14, 195.14, 195.14, 195.14,
                   195.14, 195.14, 195.03, 193.88, 191.75, 188.72, 185.32,
                   181.32, 177.32, 173.32, 169.32, 165.32, 161.32, 157.32,
                   153.32, 149.32, 145.32, 141.84};
    double o_llx[1] = {92.89};
    double o_lly[1] = {191.75};
    double o_urx[1] = {92.89};
    double o_ury[1] = {191.75};

    // set up experiment
    FrenetInitialConditions fot_ic = {
        34.6,
        7.10964962,
        -1.35277168,
        -1.86,
        0.0,
        10,
        wx,
        wy,
        25,
        o_llx,
        o_lly,
        o_urx,
        o_ury,
        1
    };
    FrenetHyperparameters fot_hp = {
        25.0,
        6.0,
        10.0,
        5.0,
        1.0,
        0.25,
        0.25,
        6.0,
        2.0,
        0.25,
        2.0,
        3.0,
        1.0,
        0.1,
        0.1,
        0.1,
        0.1,
        1.0,
        1.0,
        1.0
    };

    // run experiment
    FrenetOptimalTrajectory fot = FrenetOptimalTrajectory(&fot_ic, &fot_hp);
    FrenetPath* best_frenet_path = fot.getBestPath();
    if (best_frenet_path) {
        cout << "Success\n";
        return 1;
    }
    cout << "Failure\n";
    return 0;
}
