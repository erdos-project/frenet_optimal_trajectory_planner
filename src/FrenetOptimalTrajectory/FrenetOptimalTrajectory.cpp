#include "FrenetOptimalTrajectory.h"
#include "QuarticPolynomial.h"
#include "QuinticPolynomial.h"
#include "utils.h"

#include <cmath>

using namespace std;

// Compute the frenet optimal trajectory
FrenetOptimalTrajectory::FrenetOptimalTrajectory(
        FrenetInitialConditions *fot_ic_, FrenetHyperparameters *fot_hp_) {
    // parse the waypoints and obstacles
    fot_ic = fot_ic_;
    fot_hp = fot_hp_;
    x.assign(fot_ic->wx, fot_ic->wx + fot_ic->nw);
    y.assign(fot_ic->wy, fot_ic->wy + fot_ic->nw);
    for (int i = 0; i < fot_ic->no; i++) {
        tuple<double, double> ob (fot_ic->ox[i], fot_ic->oy[i]);
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
    double t, ti, tv, jd, jp, js, ds;
    FrenetPath* fp, *tfp;

    double di = -fot_hp->max_road_width_l;
    // generate path to each offset goal
    while (di <= fot_hp->max_road_width_r) {
        ti = fot_hp->mint;
        // lateral motion planning
        while (ti <= fot_hp->maxt) {
            jp = 0;
            jd = 0;

            fp = new FrenetPath(fot_hp);
            QuinticPolynomial lat_qp = QuinticPolynomial(
                fot_ic->c_d, fot_ic->c_d_d, fot_ic->c_d_dd, di, 0.0, 0.0, ti
            );

            // construct frenet path
            t = 0;
            while (t <= ti) {
                fp->t.push_back(t);
                fp->d.push_back(lat_qp.calc_point(t));
                fp->d_d.push_back(lat_qp.calc_first_derivative(t));
                fp->d_dd.push_back(lat_qp.calc_second_derivative(t));
                fp->d_ddd.push_back(lat_qp.calc_third_derivative(t));
                jp += pow(lat_qp.calc_third_derivative(t), 2);
                jd += pow(max(lat_qp.calc_point(t), 1.0), 4);
                t += fot_hp->dt;
            }

            // velocity keeping
            tv = fot_ic->target_speed - fot_hp->d_t_s * fot_hp->n_s_sample;
            while (tv <= fot_ic->target_speed + fot_hp->d_t_s * fot_hp->n_s_sample) {
                js = 0;

                // copy frenet path
                tfp = new FrenetPath(fot_hp);
                tfp->t.assign(fp->t.begin(), fp->t.end());
                tfp->d.assign(fp->d.begin(), fp->d.end());
                tfp->d_d.assign(fp->d_d.begin(), fp->d_d.end());
                tfp->d_dd.assign(fp->d_dd.begin(), fp->d_dd.end());
                tfp->d_ddd.assign(fp->d_ddd.begin(), fp->d_ddd.end());
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

                // delete if failure or invalid path
                bool success = tfp->to_global_path(csp);
                if (!success || !tfp->is_valid_path(obstacles)) {
                    // deallocate memory and continue
                    delete tfp;
                    tv += fot_hp->d_t_s;
                    continue;
                }

                // total lateral cost
                tfp->cd = fot_hp->kj * jp + // accumulated lateral jerk
                          fot_hp->kt * ti + // time taken cost
                          fot_hp->kd * jd;  // accumulated deviation from lane center
                // total longitudinal cost
                ds = pow(fot_ic->target_speed - tfp->s_d.back(), 2);
                tfp->cv = fot_hp->kj * js + // accumulated longitudinal jerk
                          fot_hp->kt * ti + // time taken cost
                          fot_hp->kd * ds;  // deviation from longitudinal goal
                // final cost
                tfp->cf = fot_hp->klat * tfp->cd + fot_hp->klon * tfp->cv;

                frenet_paths.push_back(tfp);
                tv += fot_hp->d_t_s;
            }
            ti += fot_hp->dt;
            // make sure to deallocate
            delete fp;
        }
        di += fot_hp->d_road_w;
    }
}

