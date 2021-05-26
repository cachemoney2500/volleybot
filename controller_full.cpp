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
	joint_task->_saturation_velocity(0) = 6.0;
	joint_task->_saturation_velocity(1) = 6.0;
	//joint_task->_ki = 30.0;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

    double heading = 0.0;

	while (runloop) {
        //cout << "START CONTROL LOOP " << controller_counter << endl;
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

		VectorXd bounce_demo_configs = redis_client.getEigenMatrixJSON(BOUNCE_DEMO_CONFIGS);
        double paddle_tilt_angle = bounce_demo_configs(0);
        double paddle_tilt_freq = bounce_demo_configs(1);
        double heading_slew_rate = bounce_demo_configs(2);

		// update model
		robot->updateModel();
	
		VectorXd q_specified = redis_client.getEigenMatrixJSON(CUSTOM_JOINT_ANGLES_KEY);
        VectorXd q_init_desired = q_specified;
        //q_init_desired(0) = -0.8;
        //q_init_desired(2) = 0.8;
        //q_init_desired(3) = 45*M_PI/180;
        //q_init_desired(4) = -90*M_PI/180;
        //q_init_desired(5) = 45*M_PI/180;
        N_prec.setIdentity();
        joint_task->updateTaskModel(N_prec);

		Vector3d q_ball = redis_client.getEigenMatrixJSON(BALL_POS_KEY);
		Vector3d dq_ball = redis_client.getEigenMatrixJSON(BALL_VEL_KEY);
        q_init_desired(0) = q_ball(0);
        q_init_desired(1) = q_ball(1);
        q_init_desired(3) = heading;
        heading += heading_slew_rate * 0.001;
        q_init_desired(10) = q_specified(10) + paddle_tilt_angle * sin(2.0*M_PI * paddle_tilt_freq * time);
        joint_task->_desired_position = q_init_desired;
        
        //cout << "base torques:\n" << base_torques << endl;
        joint_task->computeTorques(joint_task_torques);

        Vector3d paddle_position;
        robot->position(paddle_position, "racquetlink", Vector3d(0.0, 0.0, 0.0));

        Vector3d paddle_velocity;
        robot->linearVelocity(paddle_velocity, "racquetlink", Vector3d(0.0, 0.0, 0.0));


        VectorXd g(dof);
		robot->gravityVector(g);

        command_torques = joint_task_torques  + g;
        //cout << "cmd trq:\n" << command_torques << endl;

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

        //cout << "END CONTROL LOOP " << controller_counter << endl;
		controller_counter++;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}
