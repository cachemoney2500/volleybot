#include "VolleybotController.h"

VolleybotController::VolleybotController(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* ball)
{
	_robot = robot;
	int dof = _robot->_dof;

	_ball = ball;

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
    _N_posture = MatrixXd::Identity(dof, dof);
    _N_posture(0,0) = 0.0;
    _N_posture(1,1) = 0.0;
    _N_posture(2,2) = 0.0;
    _N_posture(3,3) = 0.0;
    _N_posture(11,11) = 0.0;
    _N_posture(12,12) = 0.0;
    _N_posture(13,13) = 0.0;
    _N_posture(14,14) = 0.0;
    _N_posture(15,15) = 0.0;
    _N_posture(16,16) = 0.0;

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

}

void VolleybotController::execute(unsigned long long k_iter_ctrl, Eigen::VectorXd& output_torques)
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
    Vector3d base_accel_old = Vector3d(base_torques(0)/_robot->_M(0,0), 
            base_torques(1)/_robot->_M(0,0), 0.0);
    Vector3d base_accel = _ddq.head(3);
    VectorXd leg_torques(dof);
    legControl(leg_torques, base_accel);

    /*
     * Arm control
     */
    _posture_regularization_task->updateTaskModel(_N_posture);
    VectorXd posture_torques(dof);
    _posture_regularization_task->_desired_position = _desired_position;
    _posture_regularization_task->computeTorques(posture_torques);

    VectorXd g_arm_reject_accel(dof);
    _robot->gravityVector(g_arm_reject_accel, -base_accel);
    g_arm_reject_accel.head(4) = VectorXd::Zero(4);
    g_arm_reject_accel.tail(6) = VectorXd::Zero(6);

    posture_torques = posture_torques + g_arm_reject_accel;


    if (k_iter_ctrl % 100 == 0)
    {
        std::cout << "base accel:\n" << base_accel << std::endl;
        std::cout << "base accel old:\n" << base_accel_old << std::endl;
        std::cout << "g reject arm:\n" << g_arm_reject_accel << std::endl;
        //std::cout << "kp:\n" << _base_task->_kp_mat << std::endl;
        //std::cout << "kv:\n" << _base_task->_kv_mat << std::endl;
        //std::cout << "ki:\n" << _base_task->_ki_mat << std::endl;
        //std::cout << "M:\n" << _base_task->_M_modified << std::endl;
        //std::cout << "M1:\n" << _base_task->_M_modified*_N_base_trans << std::endl;
        //std::cout << "M2:\n" << _N_base_trans*_base_task->_M_modified*_N_base_trans << std::endl;
        //std::cout << "tf:\n" << _base_task->_task_force << std::endl;
    }

    output_torques =  g + base_torques + posture_torques + leg_torques;
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
