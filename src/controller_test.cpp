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

const int n_robots = 2;

const vector<string> robot_files = {
    "./resources/legged_panda.urdf",
    "./resources/legged_panda.urdf"
};
const string obj_file = "./resources/volleyBall.urdf";

const vector<string> JOINT_ANGLES_KEYS = {
    "cs225a::volleybot::robot1::sensors::q",
    "cs225a::volleybot::robot2::sensors::q"
};
const vector<string> JOINT_VELOCITIES_KEYS = {
    "cs225a::volleybot::robot1::sensors::dq",
    "cs225a::volleybot::robot2::sensors::dq"
};
const vector<string> JOINT_ACCEL_KEYS = {
    "cs225a::volleybot::robot1::sensors::ddq",
    "cs225a::volleybot::robot2::sensors::ddq"
};
const std::string BALL_POS_KEY  = "cs225a::volleybot::ball::sensors::q";
const std::string BALL_VEL_KEY = "cs225a::volleybot::ball::sensors::dq";
const vector<string> JOINT_TORQUES_COMMANDED_KEYS = {
    "cs225a::volleybot::robot1::actuators::fgc",
    "cs225a::volleybot::robot2::actuators::fgc"
};
const vector<string> CUSTOM_JOINT_ANGLES_KEYS = {
    "cs225a::volleybot::robot1::input::q_custom",
    "cs225a::volleybot::robot2::input::q_custom"
};

const vector<string> CUSTOM_HIT_POSITION = {
    "cs225a::volleybot::robot1::input::hit_pos",
    "cs225a::volleybot::robot2::input::hit_pos"
};

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
    vector<Vector3d> robot_offsets = {
        Vector3d(0.0, -5.0, 0.5),
        Vector3d(0.0, 4.0, 0.5)
    };
    Affine3d T_world_robot = Affine3d::Identity();
    vector<Sai2Model::Sai2Model*> robots;
    for(int i = 0; i<n_robots; i++) {
        T_world_robot.translation() = robot_offsets[i];
        robots.push_back(new Sai2Model::Sai2Model(robot_files[i], false, T_world_robot));
    }
    int dof = robots[0]->_dof;
    vector<VectorXd> initial_qs;
    for(int i = 0; i<n_robots; i++) {
        robots[i]->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[i]);
        initial_qs.push_back(robots[i]->_q);
        robots[i]->updateModel();
    }


    Affine3d T_world_ball = Affine3d::Identity();
    T_world_ball.translation() << 0.0, 5.0, 3.0;
    auto ball = new Sai2Model::Sai2Model(obj_file, false, T_world_ball);

    vector<VectorXd> command_torques;
    vector<VolleybotController *> controllers;
    for(int i=0;i<n_robots; i++){
        command_torques.push_back(VectorXd(dof));
        controllers.push_back(new VolleybotController(i, robots[i], ball));
    }

    vector<Vector3d> vel_hit = {
        Vector3d(0.0, 0.0, 0.5),
        Vector3d(0.0, -1.0, 0.5)
    };

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
            for(int i=0;i<n_robots; i++){
                robots[i]->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEYS[i]);
                robots[i]->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEYS[i]);
            }
            ball->_q = redis_client.getEigenMatrixJSON(BALL_POS_KEY);
            ball->_dq = redis_client.getEigenMatrixJSON(BALL_VEL_KEY);


            // update model
            for(int i=0;i<n_robots; i++){
                robots[i]->updateModel();
            }
            ball->updateModel();


            for(int i=0;i<n_robots;i++){
                Vector3d hit_pos = redis_client.getEigenMatrixJSON(CUSTOM_HIT_POSITION[i]);
                VectorXd q_specified = redis_client.getEigenMatrixJSON(CUSTOM_JOINT_ANGLES_KEYS[i]);

                // control!
                controllers[i]->_desired_position = q_specified;
                controllers[i]->_pos_ee_desired_hip = hit_pos;
                controllers[i]->_vel_hit = vel_hit[i];
                controllers[i]->_ddq = redis_client.getEigenMatrixJSON(JOINT_ACCEL_KEYS[i]);
                controllers[i]->execute(k_iter_ctrl, command_torques[i]);

                // send to redis
                redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[i], command_torques[i]);
            }


            k_iter_ctrl++;

            // tell sim we have completed control
            redis_client.set(CONTROLLER_LOOP_ITERATION, std::to_string(k_iter_ctrl));
        }
    }

    for(int i=0;i<n_robots;i++){
        command_torques[i].setZero();
        redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEYS[i], command_torques[i]);
    }

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Control Loop updates   : " << k_iter_ctrl << "\n";

    return 0;
}
