// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }
bool fSimulationLoopDone = false;
bool fControllerLoopDone = false;

// function for converting string to bool
bool string_to_bool(const std::string& x);

// function for converting bool to string
inline const char * const bool_to_string(bool b);

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/legged_panda.urdf";

const std::string JOINT_ANGLES_KEY  = "cs225a::volleybot::robot1::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "cs225a::volleybot::robot1::sensors::dq";
const std::string BALL_POS_KEY  = "cs225a::volleybot::ball::sensors::q";
const std::string BALL_VEL_KEY = "cs225a::volleybot::ball::sensors::dq";
const std::string JOINT_TORQUES_COMMANDED_KEY  = "cs225a::volleybot::robot1::actuators::fgc";
const std::string CUSTOM_JOINT_ANGLES_KEY  = "cs225a::volleybot::robot1::input::q_custom";
const std::string BOUNCE_DEMO_CONFIGS  = "cs225a::volleybot::demo::bounce_configs";
const std::string CUSTOM_HIP_GOAL_KEY  = "cs225a::volleybot::robot1::input::hip_goal";

const std::string CONTROLLER_START_FLAG  = "cs225a::simulation::controller_start_flag";
const std::string SIMULATION_LOOP_DONE_KEY = "cs225a::simulation::done";
const std::string CONTROLLER_LOOP_DONE_KEY = "cs225a::controller::done";
const std::string CONTROLLER_TIME = "cs225a::controller::time";


unsigned long long controller_counter = 0;

const bool inertia_regularization = true;

int main() {

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_use_velocity_saturation_flag = true;

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 250.0;
	joint_task->_kv = 15.0;
	joint_task->_saturation_velocity(0) = 12.0;
	joint_task->_saturation_velocity(1) = 12.0;
	//joint_task->_ki = 30.0;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

    redis_client.set(CONTROLLER_TIME, std::to_string(controller_counter*0.001));

	while (runloop) {
		// read simulation state
        fSimulationLoopDone = string_to_bool(redis_client.get(SIMULATION_LOOP_DONE_KEY));

		// run controller loop when simulation loop is done
		if (fSimulationLoopDone) {
            double time = timer.elapsedTime() - start_time;

            // read robot state from redis
            robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
            robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

            // update model
            robot->updateModel();

            VectorXd g(dof);
            robot->gravityVector(g);


            MatrixXd N_no_external = MatrixXd::Identity(dof, dof);
            N_no_external(0,0) = 0.0;
            N_no_external(1,1) = 0.0;
            N_no_external(2,2) = 0.0;

            MatrixXd Jv_foot_left;
            robot->JvLocalFrame(Jv_foot_left, "LL_foot", Vector3d::Zero());
            MatrixXd Jv_foot_left_proj = Jv_foot_left * N_no_external;

            MatrixXd Jv_foot_right;
            robot->JvLocalFrame(Jv_foot_right, "RL_foot", Vector3d::Zero());
            MatrixXd Jv_foot_right_proj = Jv_foot_right * N_no_external;

            VectorXd bounce_demo_configs = redis_client.getEigenMatrixJSON(BOUNCE_DEMO_CONFIGS);
            double dz_hip_foot = bounce_demo_configs(0);

            Vector3d hip_pos;
            robot->position(hip_pos, "base_link", Vector3d::Zero());

            Vector3d hip_vel;
            robot->linearVelocity(hip_vel, "base_link", Vector3d::Zero());

            Matrix3d R_hip;
            robot->rotation(R_hip, "base_link");

            Vector3d foot_left_pos;
            robot->position(foot_left_pos, "LL_foot", Vector3d::Zero());

            Vector3d foot_left_vel;
            robot->linearVelocity(foot_left_vel, "LL_foot", Vector3d::Zero());

            Vector3d foot_right_pos;
            robot->position(foot_right_pos, "RL_foot", Vector3d::Zero());

            Vector3d foot_right_vel;
            robot->linearVelocity(foot_right_vel, "RL_foot", Vector3d::Zero());

            Vector3d dr_foot_left = R_hip.transpose()*(hip_pos - foot_left_pos);
            Vector3d dr_foot_right = R_hip.transpose()*(hip_pos - foot_right_pos);
            Vector3d dv_foot_left = R_hip.transpose()*(hip_vel - foot_left_vel);
            Vector3d dv_foot_right = R_hip.transpose()*(hip_vel - foot_right_vel);

            Matrix3d G_control_axes = -Matrix3d::Identity();
            G_control_axes(1,1) = 0.0;

            Matrix3d Kp_leg = Matrix3d::Zero();
            Kp_leg(0,0) = 100.0;
            Kp_leg(1,1) = 100.0;
            Kp_leg(2,2) = 20.0;

            Matrix3d Kv_leg = Matrix3d::Zero();
            Kv_leg(0,0) = 10.0;
            Kv_leg(1,1) = 10.0;
            Kv_leg(2,2) = 5.0;

            Vector3d dr_des = Vector3d(0.0, 0.0, dz_hip_foot);
            Vector3d dv_des = Vector3d::Zero();

            //Vector3d f_foot_gravity_feedforward = Vector3d(0.0, 0.0, -g(2)*0.05/2.0);
            Vector3d f_foot_gravity_feedforward = Vector3d::Zero();

            Vector3d f_foot_left = -Kp_leg*G_control_axes*(dr_foot_left - dr_des) - Kv_leg*G_control_axes*(dv_foot_left - dv_des) + f_foot_gravity_feedforward;
            Vector3d f_foot_right = -Kp_leg*G_control_axes*(dr_foot_right - dr_des) - Kv_leg*G_control_axes*(dv_foot_right - dv_des) + f_foot_gravity_feedforward;

            if (controller_counter % 100 == 0)
            {
                //cout << "dr l:\n" << dr_foot_left << endl;
                //cout << "dr r:\n" << dr_foot_right << endl;
                //cout << "dv l:\n" << dv_foot_left << endl;
                //cout << "dv r:\n" << dv_foot_right << endl;
            }

            VectorXd tau_balance(dof);
            tau_balance = Jv_foot_left_proj.transpose() * f_foot_left + Jv_foot_right_proj.transpose() * f_foot_right;
            //cout << "tau balance:\n" << tau_balance << endl;

            //cout << "Jv:\n" << Jv_foot_left << endl;
            //cout << "Jv proj:\n" << Jv_foot_left_proj << endl;
            //exit(1);
            //MatrixXd J_balance_full(6, dof);
            //J_balance_full << Jv_foot_left_proj, Jv_foot_right_proj;
            MatrixXd N_balance = MatrixXd::Identity(dof, dof);
            //N_balance(2, 2) = 0.0;
            N_balance(11, 11) = 0.0;
            N_balance(12, 12) = 0.0;
            N_balance(13, 13) = 0.0;
            N_balance(14, 14) = 0.0;
            N_balance(15, 15) = 0.0;
            N_balance(16, 16) = 0.0;
            //robot->nullspaceMatrix(N_balance, J_balance_full);
            joint_task->updateTaskModel(N_balance);
            //cout << "N balance:\n" << N_balance << endl;
            //exit(1);

            VectorXd q_specified = redis_client.getEigenMatrixJSON(CUSTOM_JOINT_ANGLES_KEY);
            VectorXd q_init_desired = q_specified;
            joint_task->_desired_position = q_init_desired;
            joint_task->computeTorques(joint_task_torques);

            VectorXd m_feet = VectorXd::Zero(dof);

            double Kr = 20.0;
            double Komega = 3.0;
            Matrix3d R_foot_left;
            robot->rotation(R_foot_left, "LL_foot");
            Vector3d omega_foot_left;
            robot->angularVelocity(omega_foot_left, "LL_foot");
            m_feet(13) = (Kr*R_hip.transpose()*R_foot_left.col(0).cross(R_hip.col(0)) - Komega*R_hip.transpose()*omega_foot_left)(1);

            Matrix3d R_foot_right;
            robot->rotation(R_foot_right, "RL_foot");
            Vector3d omega_foot_right;
            robot->angularVelocity(omega_foot_right, "RL_foot");
            m_feet(16) = (Kr*R_hip.transpose()*R_foot_right.col(0).cross(R_hip.col(0)) - Komega*R_hip.transpose()*omega_foot_right)(1);

            //g(2) *= 0.95;
            //joint_task_torques(2) *= 0.0;

            if (controller_counter % 100 == 0)
            {
                //cout << "dq:\n" << robot->_dq << endl;
                //cout << "joint torque:\n" << joint_task_torques << endl;
                //cout << "M:\n" << robot->_M << endl;
                //cout << "q fl:" << robot->_q(13) << endl;
                //cout << "R hip:\n" << R_hip << endl;
                //cout << "R foot:\n" << R_foot_left << endl;
                //cout << "R err:\n" << R_hip.transpose()*R_foot_left.col(0).cross(R_hip.col(0)) << endl;
                //cout << "om err:\n" << R_hip.transpose()*omega_foot_left << endl;
                //cout << "m fl vec:\n" << (Kr*R_hip.transpose()*R_foot_left.col(0).cross(R_hip.col(0)) - Komega*R_hip.transpose()*omega_foot_left) << endl;
                //cout << "m fl:" << m_feet(13) << endl;
            }


            command_torques =  g + tau_balance + joint_task_torques + m_feet;
            command_torques(0) = 30.0;
            //command_torques =  N_no_external*g + tau_balance;
            //cout << "cmd trq:\n" << command_torques << endl;

            // send to redis
            redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
            redis_client.set(CONTROLLER_TIME, std::to_string(controller_counter*0.001));

            // ask for next simulation loop
            fSimulationLoopDone = false;
            redis_client.set(SIMULATION_LOOP_DONE_KEY, bool_to_string(fSimulationLoopDone));
		
            controller_counter++;
        }

		// controller loop is done
        fControllerLoopDone = true;
        redis_client.set(CONTROLLER_LOOP_DONE_KEY, bool_to_string(fControllerLoopDone));
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    //std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    //std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
    std::cout << "Control Loop updates   : " << controller_counter << "\n";

	return 0;
}

//------------------------------------------------------------------------------

bool string_to_bool(const std::string& x) {
  assert(x == "false" || x == "true");
  return x == "true";
}

//------------------------------------------------------------------------------

inline const char * const bool_to_string(bool b)
{
  return b ? "true" : "false";
}
