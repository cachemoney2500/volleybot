// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "VolleybotController.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

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
const std::string SIMULATION_LOOP_ITERATION = "cs225a::simulation::k_iter";
const std::string CONTROLLER_LOOP_ITERATION = "cs225a::controller::k_iter";

bool controller_start_flag = false;
unsigned long long k_iter_ctrl = 0;
unsigned long long k_iter_sim = 0;

//const string obj_link_name = "link6";

//Vector3d forwardTracking (double time_forward, double time_air) {
//    Vector3d ball_launch_pos;
//    object->positionInWorld(ball_launch_pos, obj_link_name, Vector3d::Zero());
//    Vector3d ball_launch_vel;
//    object->linearVelocityInWorld(ball_vel, obj_link_name, Vector3d::Zero());
//    double x_f = ball_launch_vel(0)*(time_forward + time_air) + ball_launch_pos(0);
//    double y_f = ball_launch_vel(1)*(time_forward + time_air) + ball_launch_pos(1);
//    double z_f = ball_launch_pos(2) + ball_launch_vel(2)*(time_forward + time_air) - (9.81/2.0)*(time_forward + time_air)*(time_forward + time_air);
//    return predictedLanding = Vector3d(x_f, y_f, z_f);
//}
//
//Vector3d backwardTracking (double time_forward, double time_air) {
//    Vector3d ball_launch_pos;
//    object->positionInWorld(ball_launch_pos, obj_link_name, Vector3d::Zero());
//    Vector3d ball_launch_vel;
//    object->linearVelocityInWorld(ball_vel, obj_link_name, Vector3d::Zero());
//    double x_f = -ball_launch_vel(0)*(time_forward + time_air) + ball_launch_pos(0);
//    double y_f = -ball_launch_vel(1)*(time_forward + time_air) + ball_launch_pos(1);
//    double z_f = ball_launch_pos(2) + -ball_launch_vel(2)*(time_forward + time_air) + (9.81/2.0)*(time_forward + time_air)*(time_forward + time_air);
//    return predictedLanding = Vector3d(x_f, y_f, z_f);
//}
//
//Matrix3d compute_des_rotation(Vector3d vel_incident, Vector3d pos_incident, Vector3d pos_des, Matrix3d R_init){
//  double tf = -2*vel_incident(2)/9.81;
//  Vector3d a;
//  a << 0,0,-9.81
//  Vector3d vel_des = 1/tf*(pos_des-pos_incident-.5*a*pow(tf,2.0));
//  Vector3d z_des = (.5*(vel_incident+vel_des)).normalized();
//  Matrix3d R_des = R_init;
//  R_des.col(1) = R_des.col(0).cross(z_des);
//  R_des.col(2) = z_des;
//  return R_des;
//}

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
	int dof = robot->_dof;
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

    VectorXd command_torques(dof);
    auto controller = new VolleybotController(robot);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

    k_iter_ctrl = 0;
    redis_client.set(SIMULATION_LOOP_ITERATION, std::to_string(0));
    redis_client.set(CONTROLLER_LOOP_ITERATION, std::to_string(0));

	while (runloop) {
		// run controller loop when simulation loop is done (sim has caught up)
        k_iter_sim = std::stoull(redis_client.get(SIMULATION_LOOP_ITERATION));
        controller_start_flag = redis_client.get(CONTROLLER_START_FLAG) == "true";
		if (controller_start_flag && k_iter_ctrl == k_iter_sim) {
            double time = timer.elapsedTime() - start_time;

            // read robot state from redis
            robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
            robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

            // update model
            robot->updateModel();

            // control!
            VectorXd q_specified = redis_client.getEigenMatrixJSON(CUSTOM_JOINT_ANGLES_KEY);
            controller->_desired_position = q_specified;
            controller->computeTorques(command_torques);

            // send to redis
            redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

            k_iter_ctrl++;

            // tell sim we have completed control
            redis_client.set(CONTROLLER_LOOP_ITERATION, std::to_string(k_iter_ctrl));
        }
	}

	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Control Loop updates   : " << k_iter_ctrl << "\n";

	return 0;
}
