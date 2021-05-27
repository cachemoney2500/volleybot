#include <GL/glew.h>
#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew
#include "uiforce/UIForceWidget.h"

#include <iostream>
#include <string>
#include <random>

#include <signal.h>

bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;
using namespace Eigen;

// specify urdf and robots
const string world_file = "./resources/world.urdf";
const string robot_file = "./resources/legged_panda.urdf";
const string obj_file = "./resources/volleyBall.urdf";

const string robot_name = "mmp_panda";
const string obj_name = "ball";
const string camera_name = "camera_fixed";
const string ee_link_name = "link7";
const string obj_link_name = "link6";

RedisClient redis_client;

// redis keys:
// - write:
const std::string JOINT_ANGLES_KEY  = "cs225a::volleybot::robot1::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "cs225a::volleybot::robot1::sensors::dq";
const std::string JOINT_ACCEL_KEY = "cs225a::volleybot::robot1::sensors::ddq";
const std::string BALL_POS_KEY  = "cs225a::volleybot::ball::sensors::q";
const std::string BALL_VEL_KEY = "cs225a::volleybot::ball::sensors::dq";

// - read:
const std::string JOINT_TORQUES_COMMANDED_KEY  = "cs225a::volleybot::robot1::actuators::fgc";
const std::string CUSTOM_JOINT_ANGLES_KEY  = "cs225a::volleybot::robot1::input::q_custom";
const std::string BALL_TOSS_POS_KEY  = "cs225a::volleybot::ball::toss_pos";
const std::string BALL_TOSS_VEL_KEY = "cs225a::volleybot::ball::toss_vel";

const std::string CONTROLLER_START_FLAG  = "cs225a::simulation::controller_start_flag";
const std::string SIMULATION_LOOP_ITERATION = "cs225a::simulation::k_iter";
const std::string CONTROLLER_LOOP_ITERATION = "cs225a::controller::k_iter";

// simulation thread
void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* object, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback to print glew errors
bool glewInitialize();

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;
bool fRobotLinkSelect = false;
bool fToss = false;

Vector3d ball_toss_pos = Vector3d::Zero();
Vector3d ball_toss_vel = Vector3d::Zero();

bool controller_start_flag = false;
unsigned long long k_iter_ctrl = 0;
unsigned long long k_iter_sim = 0;

int main(int argc, char* argv[]) {
    cout << "Loading URDF world model file: " << world_file << endl;

    // start redis client
    redis_client = RedisClient();
    redis_client.connect();

    // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // load graphics scene
    auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
    Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
    graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
    graphics->getCamera(camera_name)->setClippingPlanes(0.01, 30.0);

    // load robots
    auto robot = new Sai2Model::Sai2Model(robot_file, false);
    robot->_q(0) = -0.8;
    robot->_q(2) = 0.8;
    robot->_q(3) = 45*M_PI/180;
    robot->_q(4) = -90*M_PI/180;
    robot->_q(5) = 45*M_PI/180;
    robot->updateModel();

    // load robot objects
    auto object = new Sai2Model::Sai2Model(obj_file, false);
    // object->_q(0) = 0.60;
    // object->_q(1) = -0.35;
     object->_q(3) = 5.0;
    object->updateModel();

    // load simulation world
    auto sim = new Simulation::Sai2Simulation(world_file, false);
    sim->setJointPositions(robot_name, robot->_q);
    sim->setJointVelocities(robot_name, robot->_dq);
    sim->setJointPositions(obj_name, object->_q);

    // set co-efficient of restition to zero for force control
    // see issue: https://github.com/manips-sai/sai2-simulation/issues/1
    sim->setCollisionRestitution(1.0);

    // set co-efficient of friction also to zero for now as this causes jitter
    sim->setCoeffFrictionStatic(0.0);
    sim->setCoeffFrictionDynamic(0.0);

    /*------- Set up visualization -------*/
    // set up error callback
    glfwSetErrorCallback(glfwError);

    // initialize GLFW
    glfwInit();

    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);

    // information about computer screen and GLUT display window
    int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - PandaApplications", NULL, NULL);
    glfwSetWindowPos(window, windowPosX, windowPosY);
    glfwShowWindow(window);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // set callbacks
    glfwSetKeyCallback(window, keySelect);
    glfwSetMouseButtonCallback(window, mouseClick);

    // init click force widget
    auto ui_force_widget = new UIForceWidget(robot_name, robot, graphics);
    ui_force_widget->setEnable(false);

    // cache variables
    double last_cursorx, last_cursory;

    // init redis client values
    redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q);
    redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq);
    redis_client.setEigenMatrixJSON(JOINT_ACCEL_KEY, VectorXd::Zero(robot->_dof));
    redis_client.setEigenMatrixJSON(BALL_POS_KEY, object->_q);
    redis_client.setEigenMatrixJSON(BALL_VEL_KEY, object->_dq);

    k_iter_sim = 0;
    redis_client.set(SIMULATION_LOOP_ITERATION, std::to_string(0));
    redis_client.set(CONTROLLER_LOOP_ITERATION, std::to_string(0));

    // setup white noise generator
    const double mean = 0.0;
    const double stddev = 1.0;  // tune based on your system 
    std::default_random_engine generator;
    std::normal_distribution<double> normal_dist(mean, stddev);

    // start simulation thread
    thread sim_thread(simulation, robot, object, sim, ui_force_widget);

    // initialize glew
    //glewInitialize();

    // while window is open:
    while (!glfwWindowShouldClose(window))// && fSimulationRunning)
    {
        // update graphics. this automatically waits for the correct amount of time
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        graphics->updateGraphics(robot_name, robot);
        graphics->updateGraphics(obj_name, object);
        graphics->render(camera_name, width, height);

        // swap buffers
        glfwSwapBuffers(window);

        // wait until all GL commands are completed
        glFinish();

        // check for any OpenGL errors
        GLenum err;
        err = glGetError();
        assert(err == GL_NO_ERROR);

        // poll for events
        glfwPollEvents();

        // move scene camera as required
        // graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
        Eigen::Vector3d cam_depth_axis;
        cam_depth_axis = camera_lookat - camera_pos;
        cam_depth_axis.normalize();
        Eigen::Vector3d cam_up_axis;
        // cam_up_axis = camera_vertical;
        // cam_up_axis.normalize();
        cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
        Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
        cam_roll_axis.normalize();
        Eigen::Vector3d cam_lookat_axis = camera_lookat;
        cam_lookat_axis.normalize();

        if (fTransXp) {
            camera_pos = camera_pos + 0.05*cam_roll_axis;
            camera_lookat = camera_lookat + 0.05*cam_roll_axis;
        }
        if (fTransXn) {
            camera_pos = camera_pos - 0.05*cam_roll_axis;
            camera_lookat = camera_lookat - 0.05*cam_roll_axis;
        }
        if (fTransYp) {
            // camera_pos = camera_pos + 0.05*cam_lookat_axis;
            camera_pos = camera_pos + 0.05*cam_up_axis;
            camera_lookat = camera_lookat + 0.05*cam_up_axis;
        }
        if (fTransYn) {
            // camera_pos = camera_pos - 0.05*cam_lookat_axis;
            camera_pos = camera_pos - 0.05*cam_up_axis;
            camera_lookat = camera_lookat - 0.05*cam_up_axis;
        }
        if (fTransZp) {
            camera_pos = camera_pos + 0.1*cam_depth_axis;
            camera_lookat = camera_lookat + 0.1*cam_depth_axis;
        }
        if (fTransZn) {
            camera_pos = camera_pos - 0.1*cam_depth_axis;
            camera_lookat = camera_lookat - 0.1*cam_depth_axis;
        }
        if (fRotPanTilt) {
            // get current cursor position
            double cursorx, cursory;
            glfwGetCursorPos(window, &cursorx, &cursory);
            //TODO: might need to re-scale from screen units to physical units
            double compass = 0.006*(cursorx - last_cursorx);
            double azimuth = 0.006*(cursory - last_cursory);
            double radius = (camera_pos - camera_lookat).norm();
            Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
            camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
            Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
            camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
        }
        if (fTransXp || fTransXn || fTransYp || fTransYn || fTransZp || fTransZn || fRotPanTilt)
        {
            cout << "Updated camera position:\n" << camera_pos << endl;
            cout << "Updated camera lookat:\n" << camera_lookat << endl;
        }
        graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
        glfwGetCursorPos(window, &last_cursorx, &last_cursory);

        ui_force_widget->setEnable(fRobotLinkSelect);
        if (fRobotLinkSelect)
        {
            double cursorx, cursory;
            int wwidth_scr, wheight_scr;
            int wwidth_pix, wheight_pix;
            std::string ret_link_name;
            Eigen::Vector3d ret_pos;

            // get current cursor position
            glfwGetCursorPos(window, &cursorx, &cursory);

            glfwGetWindowSize(window, &wwidth_scr, &wheight_scr);
            glfwGetFramebufferSize(window, &wwidth_pix, &wheight_pix);

            int viewx = floor(cursorx / wwidth_scr * wwidth_pix);
            int viewy = floor(cursory / wheight_scr * wheight_pix);

            if (cursorx > 0 && cursory > 0)
            {
                ui_force_widget->setInteractionParams(camera_name, viewx, wheight_pix - viewy, wwidth_pix, wheight_pix);
                //TODO: this behavior might be wrong. this will allow the user to click elsewhere in the screen
                // then drag the mouse over a link to start applying a force to it.
            }
        }
        if(fToss) // retoss a ball
        {
            object->_q(0) = ball_toss_pos(0);
            object->_q(1) = ball_toss_pos(1);
            object->_q(2) = ball_toss_pos(2);
            object->_dq(0) = ball_toss_vel(0) + 1.0*normal_dist(generator);//-.5+0.01*(rand()%150);
            object->_dq(1) = ball_toss_vel(1) + 0.5*normal_dist(generator);//-9.0+0.02*(rand()%100); // 7.9 is a good value
            object->_dq(2) = ball_toss_vel(2) + 0.5*normal_dist(generator);
            object->_dq(3) = 0.0; // x spin
            object->_dq(4) = 0.0; // x spin
            object->_dq(5) = 0.0; // x spin

            object->updateModel();

            cout << "Tossing ball, \nposition:\n" << object->_q.head(3) << "\nvel:\n" << object->_dq.head(3) << endl;

            sim->setJointPositions(obj_name, object->_q);
            sim->setJointVelocities(obj_name, object->_dq);
        }
    }

    // wait for simulation to finish
    fSimulationRunning = false;
    sim_thread.join();

    // destroy context
    glfwSetWindowShouldClose(window,GL_TRUE);
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

    return 0;
}

//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* object, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget)
{
    redis_client.set(CONTROLLER_START_FLAG, "false");
    // prepare simulation
    int dof = robot->dof();
    VectorXd command_torques = VectorXd::Zero(dof);
    redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

    // create a timer
    LoopTimer timer;
    timer.initializeTimer();
    timer.setLoopFrequency(1000);
    double time_slowdown_factor = 1.0;  // adjust to higher value (i.e. 2) to slow down simulation by this factor relative to real time (for slower machines)
    bool fTimerDidSleep = true;
    double start_time = timer.elapsedTime() / time_slowdown_factor; // secs
    double last_time = start_time;

    // manual object offset since the offset in world.urdf file since positionInWorld() doesn't account for this
    Vector3d obj_offset;
    obj_offset << 0, -0.35, 0.544;
    Vector3d robot_offset;
    robot_offset << 0.0, 0.3, 0.0;
    double kvj = 10;  // velocity damping for ui force drag

    // setup redis client data container for pipeset (batch write)
    std::vector<std::pair<std::string, std::string>> redis_data(6);  // set with the number of keys to write

    fSimulationRunning = true;
    while (fSimulationRunning) {
        // run sim until catches up with control loop
        k_iter_ctrl = std::stoull(redis_client.get(CONTROLLER_LOOP_ITERATION));
        controller_start_flag = redis_client.get(CONTROLLER_START_FLAG) == "true";
        if(!controller_start_flag)
        {
            VectorXd custom_q = redis_client.getEigenMatrixJSON(CUSTOM_JOINT_ANGLES_KEY);
            sim->setJointPositions(robot_name, custom_q);
            robot->_q = custom_q;
            robot->_dq = VectorXd::Zero(dof);
            robot->updateModel();

            ball_toss_pos = redis_client.getEigenMatrixJSON(BALL_TOSS_POS_KEY);
            ball_toss_vel = redis_client.getEigenMatrixJSON(BALL_TOSS_VEL_KEY);

            VectorXd ball_pos = redis_client.getEigenMatrixJSON(BALL_POS_KEY);
            object->_q = ball_pos;
            object->_dq = Vector3d::Zero();
            sim->setJointPositions(obj_name, object->_q);
            sim->setJointVelocities(obj_name, object->_dq);
            object->updateModel();
            
            redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q);
            redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq);
            redis_client.setEigenMatrixJSON(BALL_POS_KEY, object->_q);
            redis_client.setEigenMatrixJSON(BALL_VEL_KEY, object->_dq);

        }
        else 
        {
            while(k_iter_sim < k_iter_ctrl) {
                command_torques = redis_client.getEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY);
                ball_toss_pos = redis_client.getEigenMatrixJSON(BALL_TOSS_POS_KEY);
                ball_toss_vel = redis_client.getEigenMatrixJSON(BALL_TOSS_VEL_KEY);


                // z axis prismatic joint is virtual, don't allow commanded actuation
                VectorXd sim_torques = command_torques;
                sim->setJointTorques(robot_name, sim_torques);

                sim->integrate(0.001);

                // read joint positions, velocities, update model
                sim->getJointPositions(robot_name, robot->_q);
                sim->getJointVelocities(robot_name, robot->_dq);
                robot->updateModel();

                // get accel
                VectorXd ddq(dof);
                sim->getJointAccelerations(robot_name, ddq);

                sim->getJointPositions(obj_name, object->_q);
                sim->getJointVelocities(obj_name, object->_dq);
                object->updateModel();

                k_iter_sim++;

                // publish all redis keys at once to reduce multiple redis calls that slow down simulation
                // shown explicitly here, but you can define a helper function to publish data
                redis_data.at(0) = std::pair<string, string>(JOINT_ANGLES_KEY, redis_client.encodeEigenMatrixJSON(robot->_q));
                redis_data.at(1) = std::pair<string, string>(JOINT_VELOCITIES_KEY, redis_client.encodeEigenMatrixJSON(robot->_dq));
                redis_data.at(2) = std::pair<string, string>(JOINT_ACCEL_KEY, redis_client.encodeEigenMatrixJSON(ddq));
                redis_data.at(3) = std::pair<string, string>(BALL_POS_KEY, redis_client.encodeEigenMatrixJSON(object->_q));
                redis_data.at(4) = std::pair<string, string>(BALL_VEL_KEY, redis_client.encodeEigenMatrixJSON(object->_dq));
                redis_data.at(5) = std::pair<string, string>(SIMULATION_LOOP_ITERATION, std::to_string(k_iter_sim)); // tell controller sim loop is done

                redis_client.pipeset(redis_data);

                //if (object->_environmental_contacts.size() != 0) {
                //        Vector3d ball_launch_pos;
                //        object->positionInWorld(ball_launch_pos, obj_link_name, Vector3d::Zero());
                //        Vector3d ball_launch_vel;
                //        object->linearVelocityInWorld(ball_launch_vel, obj_link_name, Vector3d::Zero());
                //        double time_flight = (-ball_launch_vel(2) + sqrt(pow(ball_launch_vel(2),2.0) - 4.0*(-9.81/2.0)*ball_launch_pos(2)))/(2*(-9.81/2)); // quadratic equation
                //        double x_f = ball_launch_vel(0)*time_flight + ball_launch_pos(0);
                //        double y_f = ball_launch_vel(1)*time_flight + ball_launch_pos(1);
                //        predictedLanding = Vector3d(x_f, y_f, 0);
                //    }
        
            }
        }
    }

    double end_time = timer.elapsedTime() / time_slowdown_factor;
    std::cout << "\n";
    std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
    std::cout << "Simulation Loop updates   : " << k_iter_sim << "\n";

}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
    cerr << "GLFW Error: " << description << endl;
    exit(1);
}

//------------------------------------------------------------------------------

bool glewInitialize() {
    bool ret = false;
    #ifdef GLEW_VERSION
    if (glewInit() != GLEW_OK) {
        cout << "Failed to initialize GLEW library" << endl;
        cout << glewGetErrorString(ret) << endl;
        glfwTerminate();
    } else {
        ret = true;
    }
    #endif
    return ret;
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    bool set = (action != GLFW_RELEASE);
    switch(key) {
        case GLFW_KEY_ESCAPE:
            // exit application
            fSimulationRunning = false;
            glfwSetWindowShouldClose(window, GL_TRUE);
            break;
        case GLFW_KEY_RIGHT:
            fTransXp = set;
            break;
        case GLFW_KEY_LEFT:
            fTransXn = set;
            break;
        case GLFW_KEY_UP:
            fTransYp = set;
            break;
        case GLFW_KEY_DOWN:
            fTransYn = set;
            break;
        case GLFW_KEY_A:
            fTransZp = set;
            break;
        case GLFW_KEY_Z:
            fTransZn = set;
            break;
        case GLFW_KEY_T: // re-toss a ball
            fToss = set;
            break;
        default:
            break;
    }
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
    bool set = (action != GLFW_RELEASE);
    //TODO: mouse interaction with robot
    switch (button) {
        // left click pans and tilts
        case GLFW_MOUSE_BUTTON_LEFT:
            fRotPanTilt = set;
            // NOTE: the code below is recommended but doesn't work well
            // if (fRotPanTilt) {
            //  // lock cursor
            //  glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            // } else {
            //  glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            // }
            break;
        // if right click: don't handle. this is for menu selection
        case GLFW_MOUSE_BUTTON_RIGHT:
            fRobotLinkSelect = set;
            break;
        // if middle click: don't handle. doesn't work well on laptops
        case GLFW_MOUSE_BUTTON_MIDDLE:
            break;
        default:
            break;
    }
}
