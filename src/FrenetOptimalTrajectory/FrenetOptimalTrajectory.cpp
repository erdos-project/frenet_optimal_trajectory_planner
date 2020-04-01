#include "FrenetOptimalTrajectory.h"
#include "QuarticPolynomial.h"
#include "QuinticPolynomial.h"
#include "utils.h"

#include <cmath>
#include <iostream>

using namespace std;

// Compute the frenet optimal trajectory
FrenetOptimalTrajectory::FrenetOptimalTrajectory(
        FrenetInitialConditions *fot_ic_, FrenetHyperparameters *fot_hp_) {
    // parse the waypoints and obstacles
    fot_ic = fot_ic_;
    fot_hp = fot_hp_;
    x.assign(fot_ic->wx, fot_ic->wx + fot_ic->nw);
    y.assign(fot_ic->wy, fot_ic->wy + fot_ic->nw);
    vector<double> ox (fot_ic->ox, fot_ic->ox + fot_ic->no);
    vector<double> oy (fot_ic->oy, fot_ic->oy + fot_ic->no);
    for (int i = 0; i < ox.size(); i++) {
        tuple<double, double> ob (ox[i], oy[i]);
        obstacles.push_back(ob);
    }

    // make sure best_frenet_path is initialized
    best_frenet_path = nullptr;

    // exit if not enough waypoints
    if (x.size() <= 2) {
        return;
    }

    // construct spline path
    csp = new CubicSpline2D(x, y);

    // calculate the trajectories
    calc_frenet_paths();

    // select the best path
    double mincost = INFINITY;
    for (FrenetPath* fp : frenet_paths) {
        if (fp->cf <= mincost) {
            mincost = fp->cf;
            best_frenet_path = fp;
        }
    }
}

FrenetOptimalTrajectory::~FrenetOptimalTrajectory() {
    delete csp;
    for (FrenetPath* fp : frenet_paths) {
        delete fp;
    }
}

// Return the best path
FrenetPath* FrenetOptimalTrajectory::getBestPath() {
    return best_frenet_path;
}

// Calculate frenet paths
void FrenetOptimalTrajectory::calc_frenet_paths() {
    double t, ti, tv, jp, js, ds;
    FrenetPath* fp, *tfp;

    double di = -fot_hp->max_road_width_l;
    // generate path to each offset goal
    do {
        ti = fot_hp->mint;
        // lateral motion planning
        do {
            ti += fot_hp->dt;
            fp = new FrenetPath(fot_hp);
            QuinticPolynomial lat_qp = QuinticPolynomial(
                fot_ic->c_d, fot_ic->c_d_d, fot_ic->c_d_dd, di, 0.0, 0.0, ti
            );
            t = 0;
            // construct frenet path
            do {
                fp->t.push_back(t);
                fp->d.push_back(lat_qp.calc_point(t));
                fp->d_d.push_back(lat_qp.calc_first_derivative(t));
                fp->d_dd.push_back(lat_qp.calc_second_derivative(t));
                fp->d_ddd.push_back(lat_qp.calc_third_derivative(t));
                t += fot_hp->dt;
            } while (t < ti);

            // velocity keeping
            tv = fot_ic->target_speed - fot_hp->d_t_s * fot_hp->n_s_sample;
            do {
                jp = 0;
                js = 0;

                // copy frenet path
                tfp = new FrenetPath(fot_hp);
                for (double tt : fp->t) {
                    tfp->t.push_back(tt);
                    tfp->d.push_back(lat_qp.calc_point(tt));
                    tfp->d_d.push_back(lat_qp.calc_first_derivative(tt));
                    tfp->d_dd.push_back(lat_qp.calc_second_derivative(tt));
                    tfp->d_ddd.push_back(lat_qp.calc_third_derivative(tt));
                    jp += pow(lat_qp.calc_third_derivative(tt), 2);
                    // square jerk
                }
                QuarticPolynomial lon_qp = QuarticPolynomial(
                    fot_ic->s0, fot_ic->c_speed, 0.0, tv, 0.0, ti
                );

                // longitudinal motion
                for (double tp : tfp->t) {
                    tfp->s.push_back(lon_qp.calc_point(tp));
                    tfp->s_d.push_back(lon_qp.calc_first_derivative(tp));
                    tfp->s_dd.push_back(lon_qp.calc_second_derivative(tp));
                    tfp->s_ddd.push_back(lon_qp.calc_third_derivative(tp));
                    js += pow(lon_qp.calc_third_derivative(tp), 2);
                    // square jerk
                }

                // calculate costs
                bool success = tfp->to_global_path(csp);

                // append
                if (!success || !tfp->is_valid_path(obstacles)) {
                    // deallocate memory and continue
                    delete tfp;
                    tv += fot_hp->d_t_s;
                    continue;
                }
                ds = pow(fot_ic->target_speed - tfp->s_d.back(), 2);
                tfp->cd = fot_hp->kj * jp + fot_hp->kt * ti +
                          fot_hp->kd * pow(tfp->d.back(), 2);
                tfp->cv = fot_hp->kj * js + fot_hp->kt * ti +
                          fot_hp->kd * ds;
                tfp->cf = fot_hp->klat * tfp->cd + fot_hp->klon * tfp->cv + tfp->co;

                frenet_paths.push_back(tfp);
                tv += fot_hp->d_t_s;
            } while (tv < fot_ic->target_speed + fot_hp->d_t_s * fot_hp->n_s_sample);
            // make sure to deallocate
            delete fp;
        } while(ti < fot_hp->maxt);
        di += fot_hp->d_road_w;
    } while (di < fot_hp->max_road_width_r);
}

