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
const string obj_file = "./resources/volleyBall.urdf";

const std::string JOINT_ANGLES_KEY  = "cs225a::volleybot::robot1::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "cs225a::volleybot::robot1::sensors::dq";
const std::string JOINT_ACCEL_KEY = "cs225a::volleybot::robot1::sensors::ddq";
const std::string BALL_POS_KEY  = "cs225a::volleybot::ball::sensors::q";
const std::string BALL_VEL_KEY = "cs225a::volleybot::ball::sensors::dq";
const std::string JOINT_TORQUES_COMMANDED_KEY  = "cs225a::volleybot::robot1::actuators::fgc";
const std::string CUSTOM_JOINT_ANGLES_KEY  = "cs225a::volleybot::robot1::input::q_custom";
const std::string CUSTOM_HIT_POSITION  = "cs225a::volleybot::robot1::input::hit_pos";

const std::string CONTROLLER_START_FLAG  = "cs225a::simulation::controller_start_flag";
const std::string SIMULATION_LOOP_ITERATION = "cs225a::simulation::k_iter";
const std::string CONTROLLER_LOOP_ITERATION = "cs225a::controller::k_iter";

bool controller_start_flag = false;
unsigned long long k_iter_ctrl = 0;
unsigned long long k_iter_sim = 0;

int main() {

    // start redis client
    auto redis_client = RedisClient();
    redis_client.connect();

    // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // load robots
    Affine3d T_world_robot = Affine3d::Identity();
    T_world_robot.translation() << 0.0, -5.0, 0.5;
    auto robot = new Sai2Model::Sai2Model(robot_file, false, T_world_robot);
    int dof = robot->_dof;
    robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
    VectorXd initial_q = robot->_q;
    robot->updateModel();

    Affine3d T_world_ball = Affine3d::Identity();
    T_world_ball.translation() << 0.0, 5.0, 3.0;
    auto ball = new Sai2Model::Sai2Model(obj_file, false, T_world_ball);

    VectorXd command_torques(dof);
    auto controller = new VolleybotController(robot, ball);

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
            ball->_q = redis_client.getEigenMatrixJSON(BALL_POS_KEY);
            ball->_dq = redis_client.getEigenMatrixJSON(BALL_VEL_KEY);

            // update model
            robot->updateModel();
            ball->updateModel();

            Vector3d hit_pos = redis_client.getEigenMatrixJSON(CUSTOM_HIT_POSITION);
            VectorXd q_specified = redis_client.getEigenMatrixJSON(CUSTOM_JOINT_ANGLES_KEY);

            // control!
            controller->_desired_position = q_specified;
            controller->_pos_ee_desired_hip = hit_pos;
            controller->_ddq = redis_client.getEigenMatrixJSON(JOINT_ACCEL_KEY);
            controller->execute(k_iter_ctrl, command_torques);

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
