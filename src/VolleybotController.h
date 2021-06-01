#ifndef VOLLEYBOT_CONTROLLER_H_
#define VOLLEYBOT_CONTROLLER_H_

#include "Sai2Model.h"
#include "Sai2Primitives.h"
#include <Eigen/Dense>
#include <string>
#include <chrono>
#include <random>

class VolleybotController
{

enum State
{
    IDLE,
    BALL_TRACKING,
    HIT,
};

public:

    VolleybotController(int robot_number, Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* ball);

    void execute(unsigned long long k_iter_ctrl, Eigen::VectorXd& output_torques);

    int _robot_number;

    Sai2Model::Sai2Model* _robot;
    Sai2Model::Sai2Model* _ball;

    Sai2Primitives::JointTask* _base_task;
    Sai2Primitives::PosOriTask* _ee_posori_task;
    Sai2Primitives::JointTask* _posture_regularization_task;

    State _state;

    MatrixXd _Jv_foot_left;
    MatrixXd _Jv_foot_right;

    Matrix3d _Kp_leg;
    Matrix3d _Kv_leg;
    double _Kr_feet;
    double _Komega_feet;

    MatrixXd _N_no_external;
    MatrixXd _N_base_trans;
    MatrixXd _N_posture_ee;
    MatrixXd _N_posture_reg;

    VectorXd _desired_position;

    double _hit_height;
    Vector3d _pos_ee_desired_hip;
    Matrix3d _R_ee_desired;
    Vector3d _vel_hit;

    VectorXd _ddq;

    double _dz_hip_foot;

    std::default_random_engine _generator;
    std::uniform_real_distribution<double> _uniform_dist;

private:

    void legControl(Eigen::VectorXd& leg_torques, Vector3d base_accel);

    void plan(unsigned long long k_iter_ctrl);

    void control(unsigned long long k_iter_ctrl, Eigen::VectorXd& output_torques);

    void get_ball_state_robot_frame(Vector3d& pos, Vector3d& vel);

    double ball_time_of_flight_z_intersection(Vector3d& pos, Vector3d& vel, double z);

    void forward_prediction(Vector3d pos, Vector3d vel, Vector3d& pos_pred, Vector3d& vel_pred, double dt);

    Matrix3d compute_des_rotation(Vector3d vel_incident, Vector3d pos_incident, Vector3d pos_des, Matrix3d R_init);

    bool ball_incoming(Vector3d pos_pred, Vector3d pos_ee_cur);

};

/* VOLLEYBOT_CONTROLLER_H_ */
#endif
