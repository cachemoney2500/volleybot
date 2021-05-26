#ifndef VOLLEYBOT_CONTROLLER_H_
#define VOLLEYBOT_CONTROLLER_H_

#include "Sai2Model.h"
#include "Sai2Primitives.h"
#include <Eigen/Dense>
#include <string>
#include <chrono>

class VolleybotController
{

public:

	VolleybotController(Sai2Model::Sai2Model* robot);

	void computeTorques(Eigen::VectorXd& output_torques);

	Sai2Model::Sai2Model* _robot;

    Sai2Primitives::JointTask* _base_task;

    MatrixXd _Jv_foot_left;
    MatrixXd _Jv_foot_right;

    Matrix3d _Kp_leg;
    Matrix3d _Kv_leg;
    double _Kr_feet;
    double _Komega_feet;

    MatrixXd _N_no_external;
    MatrixXd _N_base_no_legs;

    VectorXd _desired_position;

    double _dz_hip_foot;

private:

	void legControl(Eigen::VectorXd& leg_torques, Vector3d base_accel);


};

/* VOLLEYBOT_CONTROLLER_H_ */
#endif
