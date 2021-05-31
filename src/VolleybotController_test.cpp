#include "VolleybotController.h"

VolleybotController::VolleybotController(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* ball,int idx):
    _uniform_dist(0.0, 1.0)
{
    _robot = robot;
    int dof = _robot->_dof;

    _ball = ball;

    _idx = idx;

    /*
     * Initialize the base control joint task
     */
    _base_task = new Sai2Primitives::JointTask(_robot);
    _base_task->_use_velocity_saturation_flag = true;
    _base_task->_use_interpolation_flag = false;
    _base_task->_saturation_velocity(0) = 4.0;
    _base_task->_saturation_velocity(1) = 4.0;
    _base_task->_saturation_velocity(3) = 2.0;

    VectorXd kp_base(dof);
    kp_base << 250.0 * _robot->_M(0,0),
               250.0 * _robot->_M(1,1),
               250.0 * _robot->_M(2,2),
               250.0 * _robot->_M(3,3),
               VectorXd::Ones(dof - 4);

    VectorXd kv_base(dof);
    kv_base << 15.0 * _robot->_M(0,0),
               15.0 * _robot->_M(1,1),
               15.0 * _robot->_M(2,2),
               15.0 * _robot->_M(3,3),
               VectorXd::Ones(dof - 4);

    _base_task->setNonIsotropicGains(kp_base, kv_base, VectorXd::Zero(dof));
    _base_task->setDynamicDecouplingNone();

    /*
     * Leg control gains
     */
    _Kp_leg = Matrix3d::Zero();
    _Kp_leg(0,0) = 500.0;
    _Kp_leg(1,1) = 250.0;
    _Kp_leg(2,2) = 250.0;

    _Kv_leg = Matrix3d::Zero();
    _Kv_leg(0,0) = 15.0;
    _Kv_leg(1,1) = 15.0;
    _Kv_leg(2,2) = 15.0;

    _Kr_feet = 20.0;
    _Komega_feet = 3.0;

    _N_no_external = MatrixXd::Identity(dof, dof);
    _N_no_external(0,0) = 0.0;
    _N_no_external(1,1) = 0.0;
    _N_no_external(2,2) = 0.0;

    /*
     * Base nullspace
     */
    _N_base_trans = MatrixXd::Zero(dof, dof);
    _N_base_trans(0,0) = 1.0;
    _N_base_trans(1,1) = 1.0;
    _N_base_trans(2,2) = 1.0;
    _N_base_trans(3,3) = 1.0;

    /*
     * Arm nullspace
     */
    _N_posture_ee = MatrixXd::Identity(dof, dof);
    _N_posture_ee(0,0) = 0.0;
    _N_posture_ee(1,1) = 0.0;
    _N_posture_ee(2,2) = 0.0;
    _N_posture_ee(3,3) = 0.0;
    _N_posture_ee(11,11) = 0.0;
    _N_posture_ee(12,12) = 0.0;
    _N_posture_ee(13,13) = 0.0;
    _N_posture_ee(14,14) = 0.0;
    _N_posture_ee(15,15) = 0.0;
    _N_posture_ee(16,16) = 0.0;

    /*
     * Regularization nullspace (to be filled in later)
     */
    _N_posture_reg = MatrixXd::Zero(dof, dof);

    /*
     * Initialize posture regularization
     */
    _posture_regularization_task = new Sai2Primitives::JointTask(_robot);
    _posture_regularization_task->_use_velocity_saturation_flag = true;
    _posture_regularization_task->_use_interpolation_flag = false;
    _posture_regularization_task->_kp = 250.0;
    _posture_regularization_task->_kv = 15.0;
    _posture_regularization_task->setDynamicDecouplingNone();

    _dz_hip_foot = 0.3;

    _ddq = VectorXd::Zero(dof);

    /*
     * End effector task
     */
    const Vector3d control_point = Vector3d(0,0,0.01);
    _ee_posori_task = new Sai2Primitives::PosOriTask(_robot, "link7", control_point);
    _ee_posori_task->_kp_pos = 200.0;
    _ee_posori_task->_kv_pos = 20.0;
    _ee_posori_task->_kp_ori = 200.0;
    _ee_posori_task->_kv_ori = 20.0;
    _ee_posori_task->_use_interpolation_flag = false;
    _ee_posori_task->reInitializeTask();

    _hit_height = 0.7;
    _pos_ee_desired_hip = Vector3d::Zero();
    _R_ee_desired = Matrix3d::Identity();

    _state = IDLE;
}

void VolleybotController::execute(unsigned long long k_iter_ctrl, Eigen::VectorXd& output_torques)
{
    plan(k_iter_ctrl);
    control(k_iter_ctrl, output_torques);
}

void VolleybotController::plan(unsigned long long k_iter_ctrl)
{
    Vector3d pos, vel;
    get_ball_state_robot_frame(pos, vel);

    double tof = ball_time_of_flight_z_intersection(pos, vel, _hit_height);

    Vector3d pos_pred, vel_pred;
    forward_prediction(pos, vel, pos_pred, vel_pred, tof);

    Vector3d pos_ee_cur, pos_land_des;
    _robot->position(pos_ee_cur, "link7");
    Matrix3d R_ee;
    _robot->rotation(R_ee, "link7");


    if(_state == IDLE)
    {
        Vector3d vel_world;
        _ball->linearVelocityInWorld(vel_world, "link6", Vector3d::Zero());
        //if((pos_pred - pos_ee_cur).norm() <= 8.0 && vel_world(1)*pos_ee_cur(1)>0)
        if(vel_world(1)*(2*_idx-1)>0)
        {
            _state = BALL_TRACKING;
            _hit_height = 0.6 + 0.3 * _uniform_dist(_generator);
            cout << "setting hit height to " << _hit_height << endl;
            cout << "Robot" << _idx << ": State transition to BALL_TRACKING" << endl;
        }
    }
    else if(_state == BALL_TRACKING)
    {
        _desired_position.head(3) = _robot->_q.head(3) + (pos_pred - pos_ee_cur);
        //_R_ee_desired = compute_des_rotation(pos_pred, vel_pred, -vel_pred, R_ee);
        if (pos_pred(2)>0){
            pos_land_des << 0,-4.5,0;//_hit_height;
        }else{
            pos_land_des << 0,4.5,0;//_hit_height;
        }
        _R_ee_desired = compute_des_rotation(pos_pred, vel_pred, pos_land_des, R_ee);

        Vector3d v_ball_ee = R_ee.transpose() * vel;
        Vector3d vel_world;
        _ball->linearVelocityInWorld(vel_world, "link6", Vector3d::Zero());
        if(vel_world(1)*(2*_idx-1)<0)
        {
            _state = IDLE;
            _R_ee_desired = Matrix3d::Identity();
            cout << "Robot" << _idx << ": State transition to IDLE" << endl;
        }
    }

    if (k_iter_ctrl % 100 == 0)
    {
        //std::cout << "pos ball:\n" << pos << std::endl;
        //std::cout << "vel ball:\n" << vel << std::endl;
        //std::cout << "pos pred:\n" << pos_pred << std::endl;
        //std::cout << "vel pred:\n" << vel_pred << std::endl;
        //std::cout << "vel pred n:\n" << vel_pred.normalized() << std::endl;
        //std::cout << "R_ee:\n" << R_ee << std::endl;
        //std::cout << "R_des:\n" << _R_ee_desired << std::endl;
        //std::cout << "q:\n" << _robot->_q.head(3) << std::endl;
        //std::cout << "eepos:\n" << pos_ee_cur << std::endl;
        //std::cout << "dpos:\n" << _robot->_q.head(3) + (pos_pred - pos_ee_cur) << std::endl;
    }

    //if (k_iter_ctrl % 100 == 0)
    //{
    //    std::cout << "pos ball:\n" << pos << std::endl;
    //    std::cout << "vel ball:\n" << vel << std::endl;
    //
    //    Vector3d pos_ee;
    //    _robot->position(pos_ee, "link7");
    //    std::cout << "pos base real:\n" << pos_ee << std::endl;

    //    double tof = ball_time_of_flight_z_intersection(pos, vel, _hit_height);
    //    std::cout << "tof:\n" << tof << std::endl;
    //    //std::cout << "base accel:\n" << base_accel << std::endl;
    //    //std::cout << "base accel old:\n" << base_accel_old << std::endl;
    //    //std::cout << "g reject arm:\n" << g_arm_reject_accel << std::endl;
    //    //std::cout << "kp:\n" << _base_task->_kp_mat << std::endl;
    //    //std::cout << "kv:\n" << _base_task->_kv_mat << std::endl;
    //    //std::cout << "ki:\n" << _base_task->_ki_mat << std::endl;
    //    //std::cout << "M:\n" << _base_task->_M_modified << std::endl;
    //    //std::cout << "M1:\n" << _base_task->_M_modified*_N_base_trans << std::endl;
    //    //std::cout << "M2:\n" << _N_base_trans*_base_task->_M_modified*_N_base_trans << std::endl;
    //    //std::cout << "tf:\n" << _base_task->_task_force << std::endl;
    //}
}

void VolleybotController::control(unsigned long long k_iter_ctrl, Eigen::VectorXd& output_torques)
{
    int dof = _robot->_dof;

    /*
     * Gravity compensation
     */
    VectorXd g(dof);
    _robot->gravityVector(g);

    /*
     * Base control
     */
    _base_task->updateTaskModel(_N_base_trans);
    VectorXd base_torques(dof);
    _base_task->_desired_position = _desired_position;
    _base_task->computeTorques(base_torques);

    /*
     * Leg control
     */
    Vector3d base_accel = _ddq.head(3);
    Vector3d hip_pos;
    _robot->position(hip_pos, "base_link");
    _dz_hip_foot = hip_pos(2) + 1.0;
    VectorXd leg_torques(dof);
    legControl(leg_torques, base_accel);

    /*
     * Arm control
     */
    Affine3d T_hip;
    _robot->transform(T_hip, "base_link");
    Vector3d ee_pos_des = T_hip * _pos_ee_desired_hip;
    //Vector3d hip_vel;
    //_robot->linearVelocity(hip_vel, "base_link");
    //_ee_posori_task->_desired_velocity = hip_vel;

    _ee_posori_task->updateTaskModel(_N_posture_ee);
    _ee_posori_task->_desired_position = ee_pos_des;
    _ee_posori_task->_desired_orientation = _R_ee_desired;
    VectorXd ee_posori_torques(dof);
    _ee_posori_task->computeTorques(ee_posori_torques);

    _N_posture_reg = _ee_posori_task->_N;
    _posture_regularization_task->updateTaskModel(_N_posture_reg);
    VectorXd regularization_torques(dof);
    _posture_regularization_task->_desired_position = _desired_position;
    _posture_regularization_task->computeTorques(regularization_torques);

    VectorXd g_arm_reject_accel(dof);
    _robot->gravityVector(g_arm_reject_accel, -base_accel);
    g_arm_reject_accel.head(4) = VectorXd::Zero(4);
    g_arm_reject_accel.tail(6) = VectorXd::Zero(6);

    VectorXd arm_torques(dof);
    arm_torques = ee_posori_torques + regularization_torques + g_arm_reject_accel;

    //if (k_iter_ctrl % 100 == 0)
    //{
    //    Vector3d pos_ee;
    //    _robot->position(pos_ee, "link7");
    //    std::cout << "pos input:\n" << _pos_ee_desired_hip << std::endl;
    //    std::cout << "pos base:\n" << ee_pos_des << std::endl;
    //    std::cout << "pos base real:\n" << pos_ee << std::endl;
    //    //std::cout << "base accel:\n" << base_accel << std::endl;
    //    //std::cout << "base accel old:\n" << base_accel_old << std::endl;
    //    //std::cout << "g reject arm:\n" << g_arm_reject_accel << std::endl;
    //    //std::cout << "kp:\n" << _base_task->_kp_mat << std::endl;
    //    //std::cout << "kv:\n" << _base_task->_kv_mat << std::endl;
    //    //std::cout << "ki:\n" << _base_task->_ki_mat << std::endl;
    //    //std::cout << "M:\n" << _base_task->_M_modified << std::endl;
    //    //std::cout << "M1:\n" << _base_task->_M_modified*_N_base_trans << std::endl;
    //    //std::cout << "M2:\n" << _N_base_trans*_base_task->_M_modified*_N_base_trans << std::endl;
    //    //std::cout << "tf:\n" << _base_task->_task_force << std::endl;
    //}

    output_torques =  g + base_torques + arm_torques + leg_torques;
}

void VolleybotController::legControl(Eigen::VectorXd& leg_torques, Vector3d base_accel)
{
    int dof = _robot->_dof;

    _robot->JvLocalFrame(_Jv_foot_left, "LL_foot", Vector3d::Zero());
    _Jv_foot_left = _Jv_foot_left * _N_no_external;

    MatrixXd Jv_foot_right;
    _robot->JvLocalFrame(_Jv_foot_right, "RL_foot", Vector3d::Zero());
    _Jv_foot_right = _Jv_foot_right * _N_no_external;

    Vector3d hip_pos;
    _robot->position(hip_pos, "base_link", Vector3d::Zero());

    Vector3d hip_vel;
    _robot->linearVelocity(hip_vel, "base_link", Vector3d::Zero());

    Matrix3d R_hip;
    _robot->rotation(R_hip, "base_link");

    Vector3d foot_left_pos;
    _robot->position(foot_left_pos, "LL_foot", Vector3d::Zero());

    Vector3d foot_left_vel;
    _robot->linearVelocity(foot_left_vel, "LL_foot", Vector3d::Zero());

    Vector3d foot_right_pos;
    _robot->position(foot_right_pos, "RL_foot", Vector3d::Zero());

    Vector3d foot_right_vel;
    _robot->linearVelocity(foot_right_vel, "RL_foot", Vector3d::Zero());

    Vector3d dr_foot_left = R_hip.transpose()*(hip_pos - foot_left_pos);
    Vector3d dr_foot_right = R_hip.transpose()*(hip_pos - foot_right_pos);
    Vector3d dv_foot_left = R_hip.transpose()*(hip_vel - foot_left_vel);
    Vector3d dv_foot_right = R_hip.transpose()*(hip_vel - foot_right_vel);

    Matrix3d G_control_axes = -Matrix3d::Identity();
    G_control_axes(1,1) = 0.0;

    Vector3d dr_des = Vector3d(0.0, 0.0, _dz_hip_foot);
    Vector3d dv_des = Vector3d::Zero();

    VectorXd g_legs_reject_accel(dof);
    _robot->gravityVector(g_legs_reject_accel, -base_accel);
    g_legs_reject_accel.head(11) = VectorXd::Zero(dof - 6);

    Vector3d f_foot_left = -_Kp_leg*G_control_axes*(dr_foot_left - dr_des) -
        _Kv_leg*G_control_axes*(dv_foot_left - dv_des);
    Vector3d f_foot_right = -_Kp_leg*G_control_axes*(dr_foot_right - dr_des) -
        _Kv_leg*G_control_axes*(dv_foot_right - dv_des);

    VectorXd tau_balance(dof);
    MatrixXd M_feet = MatrixXd::Identity(dof, dof);
    M_feet.block(11, 11, 6, 6) = _robot->_M.block(11, 11, 6, 6);
    tau_balance = M_feet * (_Jv_foot_left.transpose() * f_foot_left +
            _Jv_foot_right.transpose() * f_foot_right) + g_legs_reject_accel;

    VectorXd m_feet = VectorXd::Zero(dof);

    Matrix3d R_foot_left;
    _robot->rotation(R_foot_left, "LL_foot");
    Vector3d omega_foot_left;
    _robot->angularVelocity(omega_foot_left, "LL_foot");
    m_feet(13) = (_Kr_feet*R_hip.transpose()*R_foot_left.col(0).cross(R_hip.col(0)) -
            _Komega_feet*R_hip.transpose()*omega_foot_left)(1);

    Matrix3d R_foot_right;
    _robot->rotation(R_foot_right, "RL_foot");
    Vector3d omega_foot_right;
    _robot->angularVelocity(omega_foot_right, "RL_foot");
    m_feet(16) = (_Kr_feet*R_hip.transpose()*R_foot_right.col(0).cross(R_hip.col(0)) -
            _Komega_feet*R_hip.transpose()*omega_foot_right)(1);

    leg_torques = tau_balance + m_feet;

}

void VolleybotController::get_ball_state_robot_frame(Vector3d& pos, Vector3d& vel)
{
    Affine3d T_world2base;
    _robot->transformInWorld(T_world2base, "ground_link");

    Vector3d pos_world;
    _ball->positionInWorld(pos_world, "link6", Vector3d::Zero());
    pos = T_world2base.inverse() * pos_world;

    Vector3d vel_world;
    _ball->linearVelocityInWorld(vel_world, "link6", Vector3d::Zero());
    vel = T_world2base.inverse().rotation() * vel_world;
}


double VolleybotController::ball_time_of_flight_z_intersection(Vector3d& pos, Vector3d& vel, double z)
{
    double time_flight = (-vel(2) - sqrt(pow(vel(2),2.0) - 4.0*(-9.81/2.0)*(pos(2) - z)))/(2*(-9.81/2)); // quadratic equation
    return time_flight;
}

void VolleybotController::forward_prediction(Vector3d pos, Vector3d vel, Vector3d& pos_pred, Vector3d& vel_pred, double dt)
{
    pos_pred << vel(0)*(dt) + pos(0),
                vel(1)*(dt) + pos(1),
                pos(2) + vel(2)*(dt) - (9.81/2.0)*(dt)*(dt);

    vel_pred << vel(0),
                vel(1),
                vel(2) - 9.81*dt;
}

// Matrix3d VolleybotController::compute_des_rotation(Vector3d pos_incident, Vector3d vel_incident, Vector3d vel_des, Matrix3d R_init){
//   Vector3d z_des = .5*(-vel_incident.normalized() + vel_des.normalized());
//   Matrix3d R_des = R_init;
//   R_des.col(1) = z_des.cross(R_des.col(0)).normalized();
//   R_des.col(2) = z_des.normalized();
//   R_des.col(0) = R_des.col(1).cross(R_des.col(2));
//   return R_des;
// }

Matrix3d VolleybotController::compute_des_rotation(Vector3d pos_incident, Vector3d vel_incident, Vector3d pos_des, Matrix3d R_init){
  double tf = -2*vel_incident(2)/9.81;
  Vector3d a;
  a << 0,0,-9.81;
  Vector3d vel_des = 1/tf*(pos_des-pos_incident-.5*a*pow(tf,2.0));
  Vector3d z_des = .5*(-vel_incident.normalized() + vel_des.normalized());
  Matrix3d R_des = R_init;
  R_des.col(1) = z_des.cross(R_des.col(0)).normalized();
  R_des.col(2) = z_des.normalized();
  R_des.col(0) = R_des.col(1).cross(R_des.col(2));
  return R_des;
}
