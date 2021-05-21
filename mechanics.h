/*
 * Készítette: Riskutia Balázs, 2020
 * BME MOGI Tanszék
 */

#ifndef MECHANICS_H
#define	MECHANICS_H

#include <math.h>

#define BYTE unsigned char
#define BOOL unsigned char
#define FALSE 0
#define TRUE 1

typedef struct {
    float k, m, r, b; // material properties
    float fx, fy; // forces
    float ax, ay; // accelerations
    float vx, vy, vx_prev, vy_prev; // velocities
    float x, y; // positions
    float Eobv_x, alpha_x, Eobv_y, alpha_y; // passivity observer
} mechanics;

typedef struct {
    float k, m, r, b; // material properties
    float M; // torque
    float beta, beta_prev; // accelerations
    float omega, omega_prev; // velocities
    float alfa; // positions
} spinner_mechanics;

extern mechanics e; // end-effector mechanics
extern mechanics b; // ball mechanics
extern mechanics o; // opponent mechanics

extern spinner_mechanics s; // ball mechanics

typedef struct {
    float wl, wt, wr, wb; /*Wall left, top, right, bottom*/
} innerWall;

// wall parameters
extern float wl, wt, wr, wb; // variables denoting wall boundaries
extern float kwe, kwb, bwe, bse; // parameters used for collision
// Karnopp model parameters - when moving along the wall
extern float Dp, Dn, Cp, Cn, bp, bn, delta_v;

// pantograph dimensions
extern float a1, a2, a3, a4, a5;

// auxiliary variables for forward kinematics
extern float theta1, theta5, x2, y2, x4, y4, x3, y3;
extern float P2P4, P2Ph, Ph_x, Ph_y, P3Ph;
extern float ex_prev, ey_prev, evx_raw, evy_raw;
extern float e_alfa, e_alfa_prev, e_omega, e_omega_prev, e_omega_raw, e_r, e_r_hiba, e_r_hiba_prev;

// auxiliary variables for the Jacobian
extern float del1_y4, del1_x4, del5_y2, del5_x2, d, b_j, h;
extern float del1_x2, del1_y2, del5_x4, del5_y4, del1_d, del1_b, del1_h;
extern float del1_yh, del1_xh, del1_y3, del1_x3, del5_y3, del5_x3;
extern float del5_d, del5_b, del5_h, del5_yh, del5_xh;
extern float torque_left, torque_right;

// auxiliary variables for end-effector calculations
extern float ew1, ew2, ewx, ewy, ewx_abs, ewy_abs, ewx_abs_min, ewy_abs_min, ewr_abs;

// auxiliary variables for ball calculations
extern float bw1, bw2, bwx, bwy, bwx_abs, bwy_abs, b_v_abs;
extern float ebx, eby, d, feb_abs;
extern float ball_force_scaling_factor;
extern float spinner_torque_scaling_factor;
extern float spinner_force_tangent, spinner_force_radial;

extern float delta_t; // time

extern BOOL isEffectorMoving;
extern float targetX, targetY;
extern float e_x_hiba_prev, e_y_hiba_prev, e_x_hiba, e_y_hiba;
extern float P,D;
extern float F_max, F_abs, v_abs;

// function for initializing variables
void init_mechanics();
void init_InnerWalls();

// functions for collision detection (with force calculations)
void collision_detection();
void effector_wall_cd();
void effector_innerWalls_cd();
void effector_ball_cd();
void effector_spinner_cd();
void ball_wall_cd();
void ball_innerWalls_cd();
void opponent_wall_cd();
void opponent_ball_cd();

void opponent_logic();

// functions for applying forces on objects
void apply_forces();
void apply_forces_on_effector();
void apply_forces_on_ball();
void apply_torque_on_spinner();
void apply_forces_on_opponent();

// functions for simulating motion of virtual objects
void simulate_motion();
void move_ball();
void move_opponent();
void rotate_spinner();
void reset_simulation();

// advanced haptics techniques
void apply_karnopp_model();
void apply_passivity_controller();

// functions for calculation of the pantograph mechanism
void fwd_kinematics(float theta1_measured, float theta5_measured);
void compute_jacobian();
void compute_torques();

void setTargetPoint(float x, float y);
void calculateForces();

// helper functions
float norm(float v1_x, float v1_y, float v2_x, float v2_y);
int sgn(float val);
float max(float a, float b);
float min(float a, float b);

#endif

