/*
 * Készítette: Riskutia Balázs, 2020
 * BME MOGI Tanszék
 */

#include "mechanics.h"
#include "uMogi2.h"

mechanics e; // end-effector mechanics
mechanics b; // ball mechanics
mechanics o; // opponent mechanics
spinner_mechanics s;

innerWall walls_labirinth[18];
innerWall walls_airhockey[6];

// wall parameters
float wl, wt, wr, wb; // variables denoting wall boundaries
float kwe, kwb, bwe, bse; // parameters used for collision and spinner
// Karnopp model parameters - when moving along the wall
float Dp, Dn, Cp, Cn, bp, bn, delta_v;

// pantograph dimensions
float a1, a2, a3, a4, a5;

// auxiliary variables for forward kinematics
float theta1, theta5, x2, y2, x4, y4, x3, y3;
float P2P4, P2Ph, Ph_x, Ph_y, P3Ph;
float ex_prev, ey_prev, evx_raw, evy_raw;
float e_alfa, e_alfa_prev, e_omega, e_omega_prev, e_omega_raw, e_r, e_r_hiba, e_r_hiba_prev;
float o_x_hiba, o_x_hiba_prev, o_y_hiba, o_y_hiba_prev;

// auxiliary variables for the Jacobian
float del1_y4, del1_x4, del5_y2, del5_x2, d, b_j, h;
float del1_x2, del1_y2, del5_x4, del5_y4, del1_d, del1_b, del1_h;
float del1_yh, del1_xh, del1_y3, del1_x3, del5_y3, del5_x3;
float del5_d, del5_b, del5_h, del5_yh, del5_xh;
float torque_left, torque_right;

// auxiliary variables for end-effector calculations
float ew1, ew2, ewx, ewy, ewx_abs, ewy_abs, ewx_abs_min, ewy_abs_min, ewr_abs;

// auxiliary variables for ball calculations
float bw1, bw2, bwx, bwy, bwx_abs, bwy_abs, bwr_abs, b_v_abs;
float ebx, eby, d, feb_abs;
float ball_force_scaling_factor;
float spinner_torque_scaling_factor;
float spinner_force_tangent, spinner_force_radial;

float delta_t; // time

BOOL isEffectorMoving;
float targetX, targetY;
float e_x_hiba_prev, e_y_hiba_prev, e_x_hiba, e_y_hiba;
float P, D;
float F_max, F_abs, v_abs;

// function for initializing variables

void init_mechanics() {

    // initialize pantograph dimensions
    a1 = 0.180;
    a2 = 0.240;
    a3 = 0.240;
    a4 = 0.180;
    a5 = 0.120;

    // initial values of end-effector properties
    e.k = 0.0f; // won't be used
    e.m = 0.0f; //won't be used
    e.r = 0.01f; // d=20 mm
    e.b = 0.0f; // won't be used
    e.fx = 0.0f;
    e.fy = 0.0f;
    e.ax = 0.0f;
    e.ay = 0.0f;
    e.vx = 0.0f;
    e.vy = 0.0f;

    e.x = 0.0f;
    e.y = 0.0f;

    e_alfa = 0.0f;
    e_alfa_prev = 0.0f;
    e_omega = 0.0f;
    e_omega_prev = 0.0f;
    e_r_hiba = 0.01f;

    // initial values of ball properties
    b.k = 3000.0f; // 2000 N/m
    b.m = 0.6f; // 0.6 kg
    b.r = 0.02f; // d=40 mm
    b.b = 0.4f; // damping
    ball_force_scaling_factor = 0.7f;

    // initial values of spinner properties
    s.k = 0.0f; //won't be used
    s.b = 0.016f; //damping
    s.m = 8.0f; //kg
    s.r = 0.1; //d=20cm
    spinner_torque_scaling_factor = 0.9f;

    // initial values of opponent properties
    o.k = 3000.0f; // 3000 N/m
    o.m = 0.6f; // 0.6 kg
    o.r = 0.01f; // d=40 mm
    o.b = 0.4f; // damping

    reset_simulation();

    // wall boundaries (absolute values)
    wl = 0.125f;
    wr = 0.125f;
    wt = 0.080f;
    wb = 0.080f;
    kwe = 3000.0f; // stiffnes between wall and end-effector
    kwb = 20000.0f; // stiffnes between wall and ball
    bwe = 0.4f; // damping of the wall for the end-effector
    bse = 2.0f; // damping of the spinner for the end-effector

    // initializing the Karnopp model
    // aluminium on brass
    /*Dp = 2.1302f;
    Dn = -1.8623f;
    Cp = 1.7714f;
    Cn = 1.729f;
    bp = 0.457f;
    bn = 0.6196f;
    delta_v = 0.007;*/

    Dp = 1.4f;
    Dn = -1.4f;
    Cp = 1.0f;
    Cn = 1.0f;
    bp = 0.4f;
    bn = 0.4f;
    delta_v = 0.007;

    delta_t = 0.005; // 5 ms

    init_InnerWalls();
    isEffectorMoving = 0;
    e_x_hiba = 0.0f;
    e_y_hiba = 0.0f;
    P = 10.0f;
    D = 200.0f;
    F_max = 0.5f;
}

/*
 * functions for collision detection (with force calculations)
 */

void collision_detection() {
    if (SimSelect == 1) { /*Billiard*/
        effector_wall_cd();
        ball_wall_cd();
        effector_ball_cd();
    } else if (SimSelect == 2) { /*Air Hockey*/
        effector_wall_cd();
        ball_innerWalls_cd();
        effector_ball_cd();
        opponent_wall_cd();
        opponent_ball_cd();
        opponent_logic();
    } else if (SimSelect == 3) { /*Spiner*/
        effector_spinner_cd();
    } else if (SimSelect == 4) { /*Labyrinth*/
        effector_wall_cd();
    } else if (SimSelect == 5) { /*Coffee*/
        effector_spinner_cd();
    }
}

void effector_wall_cd() {
    e.fx = 0.0f;
    e.fy = 0.0f;

    ew1 = e.x - wl;
    ew2 = e.x - wr;
    ewx = fabsf(ew1) < fabsf(ew2) ? ew1 : ew2;

    ew1 = e.y - wb;
    ew2 = e.y - wt;
    ewy = fabsf(ew1) < fabsf(ew2) ? ew1 : ew2;

    ewx_abs = fabsf(ewx);
    ewy_abs = fabsf(ewy);
    ewx_abs_min = ewx_abs;
    ewy_abs_min = ewy_abs;

    if (ewx_abs < e.r) {
        e.fx = kwe * (e.r - ewx_abs);
        e.fx *= ewx > 0 ? 1 : -1;

        // add damping to force when moving towards the wall
        if (ewx > 0 && e.vx < 0) e.fx += bwe * fabsf(e.vx);
        if (ewx < 0 && e.vx > 0) e.fx += bwe * fabsf(e.vx);
    }

    if (ewy_abs < e.r) {
        e.fy = kwe * (e.r - ewy_abs);
        e.fy *= ewy > 0 ? 1 : -1;

        // add damping to force when moving towards the wall
        if (ewy > 0 && e.vy < 0) e.fy += bwe * fabsf(e.vy);
        if (ewy < 0 && e.vy > 0) e.fy += bwe * fabsf(e.vy);
    }

    if (SimSelect == 4) {
        effector_innerWalls_cd();
    }

    apply_karnopp_model();
}

void effector_innerWalls_cd() {
    int i;
    for (i = 0; i < 18; i++) {
        ew1 = e.x - walls_labirinth[i].wl;
        ew2 = e.x - walls_labirinth[i].wr;
        ewx = fabsf(ew1) < fabsf(ew2) ? ew1 : ew2;

        ew1 = e.y - walls_labirinth[i].wt;
        ew2 = e.y - walls_labirinth[i].wb;
        ewy = fabsf(ew1) < fabsf(ew2) ? ew1 : ew2;

        ewx_abs = fabsf(ewx);
        ewy_abs = fabsf(ewy);


        if (ewx_abs < e.r && ewy_abs < e.r) { /*Corner*/
            if (walls_labirinth[i].wr > e.x && e.x > walls_labirinth[i].wl) {
                e.fy = ewy > 0 ? 1 * kwe * (e.r - ewy_abs) : -1 * kwe * (e.r - ewy_abs);
                // add damping to force when moving towards the wall
                if (ewy > 0 && e.vy < 0) e.fy += bwe * fabsf(e.vy);
                if (ewy < 0 && e.vy > 0) e.fy += bwe * fabsf(e.vy);
                //For Karnopp model
                if (ewy_abs < ewy_abs_min) ewy_abs_min = ewy_abs;
            } else if (walls_labirinth[i].wt > e.y && e.y > walls_labirinth[i].wb) {
                e.fx = ewx > 0 ? 1 * kwe * (e.r - ewx_abs) : -1 * kwe * (e.r - ewx_abs);
                // add damping to force when moving towards the wall
                if (ewx > 0 && e.vx < 0) e.fx += bwe * fabsf(e.vx);
                if (ewx < 0 && e.vx > 0) e.fx += bwe * fabsf(e.vx);
                //For Karnopp model
                if (ewx_abs < ewx_abs_min) ewx_abs_min = ewx_abs;
            } else {
                ewr_abs = sqrt(ewx_abs * ewx_abs + ewy_abs * ewy_abs);
                if (ewr_abs < e.r) {
                    e.fy = ewy > 0 ? ewy_abs / ewr_abs * kwe * (e.r - ewr_abs) : -1 * ewy_abs / ewr_abs * kwe * (e.r - ewr_abs);
                    // add damping to force when moving towards the wall
                    if (ewy > 0 && e.vy < 0) e.fy += bwe * fabsf(e.vy);
                    if (ewy < 0 && e.vy > 0) e.fy += bwe * fabsf(e.vy);
                    //For Karnopp model
                    if (ewy_abs < ewy_abs_min) ewy_abs_min = ewy_abs;

                    e.fx = ewx > 0 ? ewx_abs / ewr_abs * kwe * (e.r - ewr_abs) : -1 * ewx_abs / ewr_abs * kwe * (e.r - ewr_abs);
                    // add damping to force when moving towards the wall
                    if (ewx > 0 && e.vx < 0) e.fx += bwe * fabsf(e.vx);
                    if (ewx < 0 && e.vx > 0) e.fx += bwe * fabsf(e.vx);
                    //For Karnopp model
                    if (ewx_abs < ewx_abs_min) ewx_abs_min = ewx_abs;
                }
            }
        } else {
            if (ewx_abs < e.r) {
                if (walls_labirinth[i].wt > e.y && e.y > walls_labirinth[i].wb) {
                    e.fx = ewx > 0 ? 1 * kwe * (e.r - ewx_abs) : -1 * kwe * (e.r - ewx_abs);
                    // add damping to force when moving towards the wall
                    if (ewx > 0 && e.vx < 0) e.fx += bwe * fabsf(e.vx);
                    if (ewx < 0 && e.vx > 0) e.fx += bwe * fabsf(e.vx);
                    //For Karnopp model
                    if (ewx_abs < ewx_abs_min) ewx_abs_min = ewx_abs;
                }

            }
            if (ewy_abs < e.r) {
                if (walls_labirinth[i].wr > e.x && e.x > walls_labirinth[i].wl) {
                    e.fy = ewy > 0 ? 1 * kwe * (e.r - ewy_abs) : -1 * kwe * (e.r - ewy_abs);
                    // add damping to force when moving towards the wall
                    if (ewy > 0 && e.vy < 0) e.fy += bwe * fabsf(e.vy);
                    if (ewy < 0 && e.vy > 0) e.fy += bwe * fabsf(e.vy);
                    //For Karnopp model
                    if (ewy_abs < ewy_abs_min) ewy_abs_min = ewy_abs;
                }
            }
        }
    }
}

void effector_ball_cd() {
    ebx = e.x - b.x;
    eby = e.y - b.y;
    d = sqrt(ebx * ebx + eby * eby);

    if (d < e.r + b.r) {
        feb_abs = b.k * (e.r + b.r - d);
        e.fx += feb_abs * ebx / (e.r + b.r - d);
        e.fy += feb_abs * eby / (e.r + b.r - d);
        b.fx += -1 * e.fx * ball_force_scaling_factor;
        b.fy += -1 * e.fy * ball_force_scaling_factor;
    }
}

void effector_spinner_cd() {
    e_r_hiba_prev = e_r_hiba;
    e_r = sqrt(e.x * e.x + e.y * e.y);
    e_r_hiba = 0.06f - e_r;
    if (fabsf(e_r_hiba) < 0.001f) {
        e_r_hiba = 0.0f;
    }

    if (fabsf(e_omega) < 0.5f) {
        spinner_force_tangent = 0.0f;
    } else {
        spinner_force_tangent = bse * (e_omega - s.omega) * e_r;
    }

    spinner_force_radial = 5.0f * e_r_hiba + 50.0f * (e_r_hiba - e_r_hiba_prev); //PD controller

    e.fx = spinner_force_tangent * sin(e_alfa) + spinner_force_radial * cos(e_alfa);
    e.fy = spinner_force_tangent * -1.0f * cos(e_alfa) + spinner_force_radial * sin(e_alfa);
    s.M = spinner_force_tangent * e_r * spinner_torque_scaling_factor;
}

void ball_wall_cd() {
    b.fx = 0.0f;
    b.fy = 0.0f;

    bw1 = b.x - wl;
    bw2 = b.x - wr;
    bwx = fabsf(bw1) < fabsf(bw2) ? bw1 : bw2;

    bw1 = b.y - wb;
    bw2 = b.y - wt;
    bwy = fabsf(bw1) < fabsf(bw2) ? bw1 : bw2;

    bwx_abs = fabsf(bwx);
    bwy_abs = fabsf(bwy);

    if (bwx_abs < b.r) {
        b.fx = kwb * (b.r - bwx_abs);
        b.fx *= bwx > 0 ? 1 : -1;
    }

    if (bwy_abs < b.r) {
        b.fy = kwb * (b.r - bwy_abs);
        b.fy *= bwy > 0 ? 1 : -1;
    }
}

void ball_innerWalls_cd() {
    b.fx = 0.0f;
    b.fy = 0.0f;
    int i;
    for (i = 0; i < 6; i++) {
        bw1 = b.x - walls_airhockey[i].wl;
        bw2 = b.x - walls_airhockey[i].wr;
        bwx = fabsf(bw1) < fabsf(bw2) ? bw1 : bw2;

        bw1 = b.y - walls_airhockey[i].wt;
        bw2 = b.y - walls_airhockey[i].wb;
        bwy = fabsf(bw1) < fabsf(bw2) ? bw1 : bw2;

        bwx_abs = fabsf(bwx);
        bwy_abs = fabsf(bwy);
        
        if (bwx_abs < b.r && bwy_abs < b.r) { /*Corner*/
            if (walls_airhockey[i].wr > b.x && b.x > walls_airhockey[i].wl) {
                b.fy = bwy > 0 ? 1 * kwb * (b.r - bwy_abs) : -1 * kwb * (b.r - bwy_abs);
            } else if (walls_airhockey[i].wt > b.y && b.y > walls_airhockey[i].wb) {
                b.fx = bwx > 0 ? 1 * kwb * (b.r - bwx_abs) : -1 * kwb * (b.r - bwx_abs);
            } else {
                bwr_abs = sqrt(bwx_abs * bwx_abs + bwy_abs * bwy_abs);
                if (bwr_abs < b.r) {
                    b.fy = bwy > 0 ? bwy_abs / bwr_abs * kwb * (b.r - bwr_abs) : -1 * bwy_abs / bwr_abs * kwb * (b.r - bwr_abs);
                    b.fx = bwx > 0 ? bwx_abs / bwr_abs * kwb * (b.r - bwr_abs) : -1 * bwx_abs / bwr_abs * kwb * (b.r - bwr_abs);
                }
            }
        } else {
            if (bwx_abs < b.r) {
                if (walls_airhockey[i].wt > b.y && b.y > walls_airhockey[i].wb) {
                    b.fx = bwx > 0 ? 1 * kwb * (b.r - bwx_abs) : -1 * kwb * (b.r - bwx_abs);
                }
            }
            if (bwy_abs < b.r) {
                if (walls_airhockey[i].wr > b.x && b.x > walls_airhockey[i].wl) {
                    b.fy = bwy > 0 ? 1 * kwb * (b.r - bwy_abs) : -1 * kwb * (b.r - bwy_abs);
                }
            }
        }
    }
}

void opponent_wall_cd() {
    o.fx = 0.0f;
    o.fy = 0.0f;

    bw1 = o.x - wl;
    bw2 = o.x - wr;
    bwx = fabsf(bw1) < fabsf(bw2) ? bw1 : bw2;

    bw1 = o.y - wb;
    bw2 = o.y - wt;
    bwy = fabsf(bw1) < fabsf(bw2) ? bw1 : bw2;

    bwx_abs = fabsf(bwx);
    bwy_abs = fabsf(bwy);

    if (bwx_abs < o.r) {
        o.fx = kwe * (o.r - bwx_abs);
        o.fx *= bwx > 0 ? 1 : -1;
    }

    if (bwy_abs < o.r) {
        o.fy = kwe * (o.r - bwy_abs);
        o.fy *= bwy > 0 ? 1 : -1;
    }
}

void opponent_ball_cd() {
    ebx = o.x - b.x;
    eby = o.y - b.y;
    d = sqrt(ebx * ebx + eby * eby);
    if (d < o.r + b.r) {
        feb_abs = b.k * (o.r + b.r - d);
        o.fx += feb_abs * ebx / (o.r + b.r - d)*0.05f;
        o.fy += feb_abs * eby / (o.r + b.r - d)*0.05f;
        b.fx += -1 * feb_abs * ebx / (o.r + b.r - d) * ball_force_scaling_factor;
        b.fy += -1 * feb_abs * eby / (o.r + b.r - d) * ball_force_scaling_factor;
    }
}

void opponent_logic(){
    o_x_hiba_prev = o_x_hiba;
    o_y_hiba_prev = o_y_hiba;
    float d_bkapu= sqrt(b.x*b.x+(b.y+0.077f)*(b.y+0.077f));
    o_x_hiba = (b.x +b.x/d_bkapu*(o.r+b.r) - o.x);
    b_v_abs = sqrt(b.vx * b.vx + b.vy * b.vy);
    if (b.y > 0.1f && b_v_abs < 0.1f) {
        o_y_hiba = (b.y - o.y);
    } else {
        o_y_hiba = (0.3f - o.y);
    }
    o.fx += 10.0f * o_x_hiba + 300.f * (o_x_hiba - o_x_hiba_prev);
    o.fy += 10.0f * o_y_hiba + 300.f * (o_y_hiba - o_y_hiba_prev);
}

/*
 * functions for applying forces on objects
 */

void apply_forces() {
    //apply_passivity_controller();
    if (SimSelect == 1) { /*Billiard*/
        apply_forces_on_effector();
        apply_forces_on_ball();
    } else if (SimSelect == 2) { /*Air Hockey*/
        apply_forces_on_effector();
        apply_forces_on_ball();
        apply_forces_on_opponent();
    } else if (SimSelect == 3) { /*Spiner*/
        apply_forces_on_effector();
        apply_torque_on_spinner();
    } else if (SimSelect == 4) { /*Labyrinth*/
        apply_forces_on_effector();
    } else if (SimSelect == 5) { /*Coffee*/
        apply_forces_on_effector();
        apply_torque_on_spinner();
    }
}

void apply_forces_on_effector() {
    compute_jacobian();
    compute_torques();
}

void apply_forces_on_ball() {
    b.ax = (b.fx - b.b * b.vx) / b.m;
    b.ay = (b.fy - b.b * b.vy) / b.m;
}

void apply_torque_on_spinner() {
    s.beta_prev = s.beta;
    s.beta = (s.M - s.b * s.omega) / (s.m * s.r * s.r / 2);
}

void apply_forces_on_opponent() {
    o.ax = (o.fx - o.b * o.vx) / o.m;
    o.ay = (o.fy - o.b * o.vy) / o.m;
}

/*
 * functions for simulating motion of virtual objects
 */

void simulate_motion() {
    if (SimSelect == 1) { /*Billiard*/
        move_ball();
    } else if (SimSelect == 2) { /*Air Hockey*/
        move_ball();
        move_opponent();
    } else if (SimSelect == 3) { /*Spinner*/
        rotate_spinner();
    } else if (SimSelect == 4) { /*Labyrinth*/

    } else if (SimSelect == 5) { /*Coffee*/
        rotate_spinner();
    }
}

void rotate_spinner() {
    // calculate velocity from acceleartion and previous velocity
    s.omega_prev = s.omega;
    s.omega = s.beta * delta_t + s.omega;

    // calculate position from velocity and previous position
    s.alfa = s.omega * delta_t + s.alfa;
}

void move_ball() {
    // calculate velocity from acceleartion and previous velocity
    b.vx_prev = b.vx;
    b.vy_prev = b.vy;
    b.vx = b.ax * delta_t + b.vx;
    b.vy = b.ay * delta_t + b.vy;

    // calculate position from velocity and previous position
    b.x = b.vx * delta_t + b.x;
    b.y = b.vy * delta_t + b.y;

    // if the ball is out of the box
    if (b.x < (wl - b.r) || b.x > (wr + b.r) || b.y < (wb - b.r) || b.y > (wt + b.r)) {
        if (SimSelect == 2) {
            if (b.y < (wb - b.r) && b.x < (walls_airhockey[4].wl - b.r) && b.x > (walls_airhockey[5].wr + b.r)) {
                PointV++;
                sprintf(lcd + lcd_cpl, "%i:%i             ", PointH, PointV);
                lcd_update();
            }
            if (b.y > (wt + b.r) && b.x < (walls_airhockey[2].wl - b.r) && b.x > (walls_airhockey[1].wr + b.r)) {
                PointH++;
                sprintf(lcd + lcd_cpl, "%i:%i             ", PointH, PointV);
                lcd_update();
            }
        }
        reset_simulation();
    }
}

void move_opponent() {
    o.vx_prev = o.vx;
    o.vy_prev = o.vy;
    o.vx = o.ax * delta_t + o.vx;
    o.vy = o.ay * delta_t + o.vy;

    // calculate position from velocity and previous position
    o.x = o.vx * delta_t + o.x;
    o.y = o.vy * delta_t + o.y;
}

void reset_simulation() {

    b.fx = 0.0f;
    b.fy = 0.0f;
    b.ax = 0.0f;
    b.ay = 0.0f;
    b.vx = 0.0f;
    b.vy = 0.0f;
    b.vx_prev = 0.0f;
    b.vy_prev = 0.0f;
    b.x = 0.065f;
    b.y = 0.02f;

    s.M = 0.0f;
    s.alfa = 0.0f;
    s.omega = 0.0f;
    s.omega_prev = 0.0f;
    s.beta = 0.0f;
    s.beta_prev = 0.0f;

    o.fx = 0.0f;
    o.fy = 0.0f;
    o.ax = 0.0f;
    o.ay = 0.0f;
    o.vx = 0.0f;
    o.vy = 0.0f;
    o.vx_prev = 0.0f;
    o.vy_prev = 0.0f;
    o.x = 0.0f;
    o.y = 0.3f;
}

/*
 * advanced haptics techniques
 */

void apply_karnopp_model() {

    if (ewx_abs_min < e.r) {
        // applying the Karnopp model - when moving along the wall
        if (e.vy < -1 * delta_v) {
            e.fy -= Cn * sgn(e.vy) + bn * e.vy;
        } else if (e.vy >= -1 * delta_v && e.vy < 0) {
            e.fy -= max(Dn, e.fy);
        } else if (e.vy >= 0 && e.vy < delta_v) {
            e.fy -= min(Dp, e.fy);
        } else if (e.vy >= delta_v) {
            e.fy -= Cp * sgn(e.vy) + bp * e.vy;
        }
    }

    if (ewy_abs_min < e.r) {
        // applying the Karnopp model - when moving along the wall
        if (e.vx < -1 * delta_v) {
            e.fx -= Cn * sgn(e.vx) + bn * e.vx;
        } else if (e.vx >= -1 * delta_v && e.vx < 0) {
            e.fx -= max(Dn, e.fx);
        } else if (e.vx >= 0 && e.vx < delta_v) {
            e.fx -= min(Dp, e.fx);
        } else if (e.vx >= delta_v) {
            e.fx -= Cp * sgn(e.vx) + bp * e.vx;
        }
    }
}

void apply_passivity_controller() {
    // compute for ball

    b.Eobv_x = b.Eobv_x + (b.fx * b.vx + b.alpha_x * b.vx_prev * b.vx_prev) * delta_t;
    b.alpha_x = b.Eobv_x < 0 ? (-1 * b.Eobv_x / (delta_t * b.vx * b.vx)) : 0;
    b.fx = b.fx + b.alpha_x * b.vx;

    b.Eobv_y = b.Eobv_y + (b.fy * b.vy + b.alpha_y * b.vy_prev * b.vy_prev) * delta_t;
    b.alpha_y = b.Eobv_y < 0 ? (-1 * b.Eobv_y / (delta_t * b.vy * b.vy)) : 0;
    b.fy = b.fy + b.alpha_y * b.vy;

    // compute for end-effector
    e.Eobv_x = e.Eobv_x + (e.fx * e.vx + e.alpha_x * e.vx_prev * e.vx_prev) * delta_t;
    e.alpha_x = e.Eobv_x < 0 ? (-1 * e.Eobv_x / (delta_t * e.vx * e.vx)) : 0;
    e.fx = e.fx + e.alpha_x * e.vx;

    e.Eobv_y = e.Eobv_y + (e.fy * e.vy + e.alpha_y * e.vy_prev * e.vy_prev) * delta_t;
    e.alpha_y = e.Eobv_y < 0 ? (-1 * e.Eobv_y / (delta_t * e.vy * e.vy)) : 0;
    e.fy = e.fy + e.alpha_y * e.vy;
}

/*
 * functions for calculation of the pantograph mechanism
 */

void fwd_kinematics(float theta1_measured, float theta5_measured) {

    theta1 = theta1_measured + 0.3636f; // theta1_measured + theta1_ref
    theta5 = theta5_measured + 2.6061f; // theta5_measured + theta5_ref

    /*sprintf(lcd, "%.4f;%.4f", theta1, theta5);
    lcd_update();*/
    // Compute position of P2
    x2 = a1 * cos(theta1);
    y2 = a1 * sin(theta1);

    // Compute position of P4
    x4 = a4 * cos(theta5) - a5;
    y4 = a4 * sin(theta5);

    // Get distance between P2 and P4
    P2P4 = norm(x4, y4, x2, y2);

    // Compute position of handle
    P2Ph = (a2 * a2 - a3 * a3 + P2P4 * P2P4) / (2.0 * P2P4);
    Ph_x = x2 + (P2Ph / P2P4) * (x4 - x2);
    Ph_y = y2 + (P2Ph / P2P4) * (y4 - y2);
    P3Ph = sqrt(a2 * a2 - P2Ph * P2Ph);
    x3 = Ph_x + (P3Ph / norm(x2, y2, x4, y4)) * (y4 - y2);
    y3 = Ph_y - (P3Ph / norm(x2, y2, x4, y4)) * (x4 - x2);

    ex_prev = e.x;
    ey_prev = e.y;
    e.x = x3 + 0.06f;
    e.y = y3 - 0.27f;

    // Compute velocity of handle
    evx_raw = (e.x - ex_prev) / delta_t;
    evy_raw = (e.y - ey_prev) / delta_t;

    // Filter velocity (low pass filter)
    e.vx_prev = e.vx;
    e.vy_prev = e.vy;
    e.vx = 0.9f * evx_raw + (1 - 0.9f) * e.vx;
    e.vy = 0.9f * evy_raw + (1 - 0.9f) * e.vy;

    e_alfa_prev = e_alfa;
    e_alfa = atan2(e.y, e.x);
    if (e_alfa < 0 && e_alfa_prev > 0 && e.x < 0) {
        e_omega_raw = (e_alfa + 2 * 3.1415f - e_alfa_prev) / delta_t;
    } else if (e_alfa > 0 && e_alfa_prev < 0 && e.x < 0) {
        e_omega_raw = (e_alfa - 2 * 3.1415f - e_alfa_prev) / delta_t;
    } else {
        e_omega_raw = (e_alfa - e_alfa_prev) / delta_t;
    }

    e_omega_prev = e_omega;
    e_omega = 0.8f * e_omega_raw + (1 - 0.8f) * e_omega;
}

void compute_jacobian() {

    del1_y4 = 0.0f;
    del1_x4 = 0.0f;
    del5_y2 = 0.0f;
    del5_x2 = 0.0f;

    d = norm(x2, y2, x4, y4);
    b_j = norm(x2, y2, Ph_x, Ph_y);
    h = norm(x3, y3, Ph_x, Ph_y);

    del1_x2 = -a1 * sin(theta1); //NOTE: THE AUTHOR FORGOT NEGATIVE SIGN IN THE PAPER
    del1_y2 = a1 * cos(theta1);
    del5_x4 = -a4 * sin(theta5); //NOTE: THE AUTHOR FORGOT NEGATIVE SIGN IN THE PAPER
    del5_y4 = a4 * cos(theta5);

    //joint 1
    del1_d = ((x4 - x2)*(del1_x4 - del1_x2) + (y4 - y2)*(del1_y4 - del1_y2)) / d;
    del1_b = del1_d - (del1_d * (a2 * a2 - a3 * a3 + d * d)) / (2.0 * d * d);
    del1_h = -b_j * del1_b / h;

    del1_yh = del1_y2 + (del1_b * d - del1_d * b_j) / (d * d) * (y4 - y2) + b_j / d * (del1_y4 - del1_y2);
    del1_xh = del1_x2 + (del1_b * d - del1_d * b_j) / (d * d) * (x4 - x2) + b_j / d * (del1_x4 - del1_x2);

    del1_y3 = del1_yh - h / d * (del1_x4 - del1_x2) - (del1_h * d - del1_d * h) / (d * d) *(x4 - x2);
    del1_x3 = del1_xh + h / d * (del1_y4 - del1_y2) + (del1_h * d - del1_d * h) / (d * d) *(y4 - y2);

    //joint 2
    del5_d = ((x4 - x2)*(del5_x4 - del5_x2)+(y4 - y2)*(del5_y4 - del5_y2)) / d;
    del5_b = del5_d - (del5_d * (a2 * a2 - a3 * a3 + d * d)) / (2.0 * d * d);
    del5_h = -b_j * del5_b / h;

    del5_yh = del5_y2 + (del5_b * d - del5_d * b_j) / (d * d) * (y4 - y2) + b_j / d * (del5_y4 - del5_y2);
    del5_xh = del5_x2 + (del5_b * d - del5_d * b_j) / (d * d) * (x4 - x2) + b_j / d * (del5_x4 - del5_x2);

    del5_y3 = del5_yh - h / d * (del5_x4 - del5_x2) - (del5_h * d - del5_d * h) / (d * d) * (x4 - x2);
    del5_x3 = del5_xh + h / d * (del5_y4 - del5_y2) + (del5_h * d - del5_d * h) / (d * d) * (y4 - y2);
}

void compute_torques() {

    torque_right = del1_x3 * e.fx + del1_y3 * e.fy;
    torque_left = del5_x3 * e.fx + del5_y3 * e.fy;
}

void init_InnerWalls() {

    walls_labirinth[0].wl = -0.143f;
    walls_labirinth[0].wt = 0.049f;
    walls_labirinth[0].wr = -0.079f;
    walls_labirinth[0].wb = 0.047f;

    walls_labirinth[1].wl = -0.049f;
    walls_labirinth[1].wt = 0.079f;
    walls_labirinth[1].wr = -0.047f;
    walls_labirinth[1].wb = 0.049f;

    walls_labirinth[2].wl = -0.049f;
    walls_labirinth[2].wt = 0.049f;
    walls_labirinth[2].wr = 0.017f;
    walls_labirinth[2].wb = 0.047f;

    walls_labirinth[3].wl = 0.047f;
    walls_labirinth[3].wt = 0.049f;
    walls_labirinth[3].wr = 0.113f;
    walls_labirinth[3].wb = 0.047f;

    walls_labirinth[4].wl = -0.113f;
    walls_labirinth[4].wt = 0.017f;
    walls_labirinth[4].wr = -0.047f;
    walls_labirinth[4].wb = 0.015f;

    walls_labirinth[5].wl = -0.017f;
    walls_labirinth[5].wt = 0.047f;
    walls_labirinth[5].wr = -0.015f;
    walls_labirinth[5].wb = -0.017f;

    walls_labirinth[6].wl = 0.047f;
    walls_labirinth[6].wt = 0.047f;
    walls_labirinth[6].wr = 0.049f;
    walls_labirinth[6].wb = 0.017f;

    walls_labirinth[7].wl = -0.143f;
    walls_labirinth[7].wt = -0.015f;
    walls_labirinth[7].wr = -0.111f;
    walls_labirinth[7].wb = -0.017f;

    walls_labirinth[8].wl = -0.081f;
    walls_labirinth[8].wt = 0.015f;
    walls_labirinth[8].wr = -0.079f;
    walls_labirinth[8].wb = -0.049f;

    walls_labirinth[9].wl = -0.049f;
    walls_labirinth[9].wt = -0.015f;
    walls_labirinth[9].wr = -0.017f;
    walls_labirinth[9].wb = -0.017f;

    walls_labirinth[10].wl = 0.015f;
    walls_labirinth[10].wt = 0.017f;
    walls_labirinth[10].wr = 0.113f;
    walls_labirinth[10].wb = 0.015f;

    walls_labirinth[11].wl = -0.113f;
    walls_labirinth[11].wt = -0.047f;
    walls_labirinth[11].wr = -0.081f;
    walls_labirinth[11].wb = -0.049f;

    walls_labirinth[12].wl = -0.049f;
    walls_labirinth[12].wt = -0.047f;
    walls_labirinth[12].wr = 0.015f;
    walls_labirinth[12].wb = -0.049f;

    walls_labirinth[13].wl = 0.015f;
    walls_labirinth[13].wt = 0.015f;
    walls_labirinth[13].wr = 0.017f;
    walls_labirinth[13].wb = -0.079f;

    walls_labirinth[14].wl = 0.047f;
    walls_labirinth[14].wt = -0.015f;
    walls_labirinth[14].wr = 0.049f;
    walls_labirinth[14].wb = -0.049f;

    walls_labirinth[15].wl = 0.049f;
    walls_labirinth[15].wt = -0.015f;
    walls_labirinth[15].wr = 0.143f;
    walls_labirinth[15].wb = -0.017f;

    walls_labirinth[16].wl = 0.079f;
    walls_labirinth[16].wt = -0.047f;
    walls_labirinth[16].wr = 0.113f;
    walls_labirinth[16].wb = -0.049f;

    walls_labirinth[17].wl = 0.111f;
    walls_labirinth[17].wt = -0.049f;
    walls_labirinth[17].wr = 0.113f;
    walls_labirinth[17].wb = -0.079f;

    walls_airhockey[0].wl = -0.150;
    walls_airhockey[0].wt = 0.377f;
    walls_airhockey[0].wr = -0.120f;
    walls_airhockey[0].wb = -0.077f;

    walls_airhockey[1].wl = -0.120;
    walls_airhockey[1].wt = 0.407f;
    walls_airhockey[1].wr = -0.040f;
    walls_airhockey[1].wb = 0.377f;

    walls_airhockey[2].wl = 0.040;
    walls_airhockey[2].wt = 0.407f;
    walls_airhockey[2].wr = 0.120f;
    walls_airhockey[2].wb = 0.377f;

    walls_airhockey[3].wl = 0.120;
    walls_airhockey[3].wt = 0.377f;
    walls_airhockey[3].wr = 0.150f;
    walls_airhockey[3].wb = -0.077f;

    walls_airhockey[4].wl = 0.040;
    walls_airhockey[4].wt = -0.077f;
    walls_airhockey[4].wr = 0.120f;
    walls_airhockey[4].wb = -0.107f;

    walls_airhockey[5].wl = -0.120;
    walls_airhockey[5].wt = -0.077f;
    walls_airhockey[5].wr = -0.040f;
    walls_airhockey[5].wb = -0.107f;

}

void setTargetPoint(float x, float y) {
    targetX = x;
    targetY = y;
    isEffectorMoving = 1;
}

void calculateForces() {
    e_x_hiba_prev = e_x_hiba;
    e_y_hiba_prev = e_y_hiba;
    e_x_hiba = (targetX - e.x);
    e_y_hiba = (targetY - e.y);
    e_r_hiba = sqrt(e_x_hiba * e_x_hiba + e_y_hiba * e_y_hiba);
    v_abs = sqrt(e.vx * e.vx + e.vy * e.vy);
    if (e_r_hiba < 0.002f && v_abs < 0.05f) {
        isEffectorMoving = 0;
        e.fx = 0;
        e.fy = 0;
    } else {
        e.fx = P * e_x_hiba + D * (e_x_hiba - e_x_hiba_prev);
        e.fy = P * e_y_hiba + D * (e_y_hiba - e_y_hiba_prev);
        F_abs = sqrt(e.fx * e.fx + e.fy * e.fy);
        if (F_abs > F_max) {
            e.fx = F_max * e.fx / F_abs;
            e.fy = F_max * e.fy / F_abs;
        }
    }
}

/*
 * helper functions
 */

float norm(float v1_x, float v1_y, float v2_x, float v2_y) {
    return (float) sqrt((v1_x - v2_x)*(v1_x - v2_x) + (v1_y - v2_y)*(v1_y - v2_y));
}

int sgn(float val) {

    return (0 < val) - (val < 0);
}

float max(float a, float b) {
    if (a > b) return a;

    return b;
}

float min(float a, float b) {
    if (a < b) return a;
    return b;
}