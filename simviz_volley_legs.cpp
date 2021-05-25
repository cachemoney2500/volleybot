#include <GL/glew.h>
#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include <GLFW/glfw3.h>  // must be loaded after loading opengl/glew
#include "uiforce/UIForceWidget.h"  // used for right-click drag interaction in window
#include <random>  // used for white-noise generation
#include "force_sensor/ForceSensorSim.h"  // references src folder in sai2-common directory
#include "force_sensor/ForceSensorDisplay.h"
#include <signal.h>

bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}
bool fSimulationLoopDone = false;
bool fControllerLoopDone = true; // initialize as true for first loop

// function for converting string to bool
bool string_to_bool(const std::string& x);

// function for converting bool to string
inline const char * const bool_to_string(bool b);


using namespace std;
using namespace Eigen;

// specify urdf and robots
const string world_file_full = "./resources/world_legged.urdf";
const string robot_file_full = "./resources/legged_panda.urdf";
const string world_file_simple = "./resources/world_simple_legged.urdf";
const string robot_file_simple = "./resources/simple_legged_panda.urdf";
const string obj_file = "./resources/tennisBall.urdf";
const string robot_name = "mmp_panda";
const string obj_name = "ball";
const string camera_name = "camera_fixed";
const string ee_link_name = "link7";

RedisClient redis_client;

// redis keys:
// - write:
const std::string JOINT_ANGLES_KEY  = "cs225a::volleybot::robot1::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "cs225a::volleybot::robot1::sensors::dq";
const std::string BALL_POS_KEY  = "cs225a::volleybot::ball::sensors::q";
const std::string BALL_VEL_KEY = "cs225a::volleybot::ball::sensors::dq";
const std::string CAMERA_POS_KEY = "cs225a::camera::pos";
const std::string CAMERA_ORI_KEY = "cs225a::camera::ori";
const std::string CAMERA_DETECT_KEY = "cs225a::camera::detect";
const std::string CAMERA_OBJ_POS_KEY = "cs225a::camera::obj_pos";
const std::string EE_FORCE_KEY = "cs225a::sensor::force";
const std::string EE_MOMENT_KEY = "cs225a::sensor::moment";
// - read:
const std::string JOINT_TORQUES_COMMANDED_KEY  = "cs225a::volleybot::robot1::actuators::fgc";
const std::string CUSTOM_JOINT_ANGLES_KEY  = "cs225a::volleybot::robot1::input::q_custom";

const std::string CONTROLLER_START_FLAG  = "cs225a::simulation::controller_start_flag";
const std::string SIMULATION_LOOP_ITERATION = "cs225a::simulation::k_iter";
const std::string CONTROLLER_LOOP_ITERATION = "cs225a::controller::k_iter";

// force sensor
ForceSensorSim* force_sensor;

// display widget for forces at end effector
ForceSensorDisplay* force_display;

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

// callback boolean check for objects in camera FOV
bool cameraFOV(Vector3d object_pos, Vector3d camera_pos, Matrix3d camera_ori, double radius, double fov_angle);

// helper function for cameraFOV
bool compareSigns(double a, double b);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;
bool fRobotLinkSelect = false;

bool controller_start_flag = false;
unsigned long long k_iter_ctrl = 0;
unsigned long long k_iter_sim = 0;

int main(int argc, char* argv[]) {
    if(argc < 2)
    {
        cerr << "Error: need to specify \"simple\" or \"full\" argument" << endl;
        exit(1);
    }

    std::string world_file;
    std::string robot_file;
    if(std::strcmp(argv[1], "simple") == 0)
    {
        world_file = world_file_simple;
        robot_file = robot_file_simple;
    }
    else if(std::strcmp(argv[1], "full") == 0)
    {
        world_file = world_file_full;
        robot_file = robot_file_full;
    }
    else
    {
        cerr << "Unknown argument input" << endl;
        exit(1);
    }

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
	//graphics->showLinkFrame(true, robot_name, ee_link_name, 0.15);  // can add frames for different links
    graphics->getCamera(camera_name)->setClippingPlanes(0.01, 30.0);

	//graphics->showLinkFrame(true, robot_name, "base_heading", 0.15);  // can add frames for different links
	//graphics->showLinkFrame(true, robot_name, "LL_KOSY_L56", 0.15);  // can add frames for different links
	//graphics->showLinkFrame(true, robot_name, "LL_KOSY_L4", 0.15);  // can add frames for different links
	//graphics->showLinkFrame(true, robot_name, "LL_KOSY_L1", 0.15);  // can add frames for different links
	//graphics->showLinkFrame(true, robot_name, "LL_foot", 0.15);  // can add frames for different links

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
    //sim->setCoeffFrictionStatic(0.5);
    //sim->setCoeffFrictionDynamic(0.5);

	// initialize force sensor: needs Sai2Simulation sim interface type
	force_sensor = new ForceSensorSim(robot_name, ee_link_name, Eigen::Affine3d::Identity(), robot);
	force_display = new ForceSensorDisplay(force_sensor, graphics);

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
	GLFWwindow* window = glfwCreateWindow(windowW, windowH, "cs225a - collision-demo", NULL, NULL);
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
	redis_client.setEigenMatrixJSON(BALL_POS_KEY, object->_q);
	redis_client.setEigenMatrixJSON(BALL_VEL_KEY, object->_dq);

    k_iter_sim = 0;
    redis_client.set(SIMULATION_LOOP_ITERATION, std::to_string(0));
    redis_client.set(CONTROLLER_LOOP_ITERATION, std::to_string(0));

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
		force_display->update();
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

	}

	// wait for simulation to finish
	fSimulationRunning = false;
	sim_thread.join();

	// destroy context
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

	// init control variables
	VectorXd g(dof);
	Eigen::Vector3d ui_force;
	ui_force.setZero();
	Eigen::VectorXd ui_force_command_torques;
	ui_force_command_torques.setZero();

	// sensed forces and moments from sensor
	Eigen::Vector3d sensed_force;
    Eigen::Vector3d sensed_moment;

	// manual object offset since the offset in world.urdf file since positionInWorld() doesn't account for this
	Vector3d obj_offset;
	obj_offset << 0, -0.35, 0.544;
	Vector3d robot_offset;
	robot_offset << 0.0, 0.3, 0.0;
	double kvj = 10;  // velocity damping for ui force drag

	// init camera detection variables
	Vector3d camera_pos, obj_pos;
	Matrix3d camera_ori;
	bool detect;
	const std::string true_message = "Detected";
	const std::string false_message = "Not Detected";

	// setup redis client data container for pipeset (batch write)
	std::vector<std::pair<std::string, std::string>> redis_data(13);  // set with the number of keys to write

	// setup white noise generator
    const double mean = 0.0;
    const double stddev = 0.001;  // tune based on your system
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);

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

            VectorXd ball_pos = redis_client.getEigenMatrixJSON(BALL_POS_KEY);
            sim->setJointPositions(obj_name, ball_pos);
            object->_q = ball_pos;
            object->_dq = Vector3d::Zero();
            object->updateModel();
            
            redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q);
            redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq);
            redis_client.setEigenMatrixJSON(BALL_POS_KEY, object->_q);
            redis_client.setEigenMatrixJSON(BALL_VEL_KEY, object->_dq);
        }
        else 
        {
            while(k_iter_sim < k_iter_ctrl) {
                //cout << "starting simulation" << endl;
                // get gravity torques
                robot->gravityVector(g);
                //g(2) = 0.0;

                //int q_ctrl_index = 3;
                //double Kq = 3.0;
                //double q_des = 60*M_PI/180 * sin(2*M_PI*0.3*last_time);
                //VectorXd q_des_vec = VectorXd::Zero(dof);
                //q_des_vec(q_ctrl_index) = q_des;
                //q_des_vec(0) = -0.8;
                //q_des_vec(2) = 0.8;

                // read arm torques from redis and apply to simulated robot
                command_torques = redis_client.getEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY);

                // z axis prismatic joint is virtual, don't allow commanded actuation
                VectorXd sim_torques = command_torques;
                //for(int i = 11; i  <= 16; i++)
                //{
                //    sim_torques(i) *= 0.0;
                //}

                // get forces from interactive screen
                ui_force_widget->getUIForce(ui_force);
                ui_force_widget->getUIJointTorques(ui_force_command_torques);

                sim->setJointTorques(robot_name, sim_torques);

                sim->integrate(0.001);

                //int q_ctrl_index = (int) redis_client.getEigenMatrixJSON("test::q_ctrl_index")(0);
                //cout << "q ctrl: " << q_ctrl_index << endl;
                //double q_des = 60*M_PI/180 * sin(2*M_PI*0.3*last_time);
                //robot->_q.setZero();
                //robot->_q(q_ctrl_index) = q_des;
                //sim->setJointPositions(robot_name, robot->_q);

                // read joint positions, velocities, update model
                sim->getJointPositions(robot_name, robot->_q);
                sim->getJointVelocities(robot_name, robot->_dq);
                robot->updateModel();

                sim->getJointPositions(obj_name, object->_q);
                sim->getJointVelocities(obj_name, object->_dq);
                object->updateModel();

                // update force sensor readings
                force_sensor->update(sim);
                force_sensor->getForceLocalFrame(sensed_force);  // refer to ForceSensorSim.h in sai2-common/src/force_sensor (can also get wrt global frame)
                force_sensor->getMomentLocalFrame(sensed_moment);

                if (k_iter_sim % 100 == 0)
                {
                    sim->showContactInfo();
                    //cout << "dq:\n" << robot->_dq << endl;
                    //cout << "dr l:\n" << dr_foot_left << endl;
                    //cout << "dr r:\n" << dr_foot_right << endl;
                    //cout << "dv l:\n" << dv_foot_left << endl;
                    //cout << "dv r:\n" << dv_foot_right << endl;
                }

                // std::cout << "Sensed Force: " << sensed_force.transpose() << "Sensed Moment: " << sensed_moment.transpose() << std::endl;

                // query object position and ee pos/ori for camera detection
                //object->positionInWorld(obj_pos, "link6");
                //robot->positionInWorld(camera_pos, "link7");
                //robot->rotationInWorld(camera_ori, "link7");  // local to world frame

                // add position offset in world.urdf file since positionInWorld() doesn't account for this
                obj_pos += obj_offset;
                camera_pos += robot_offset;  // camera position/orientation is set to the panda's last link

                // object camera detect
                detect = cameraFOV(obj_pos, camera_pos, camera_ori, 1.0, M_PI/6);
                if (detect == true) {
                    obj_pos(0) += dist(generator);  // add white noise
                    obj_pos(1) += dist(generator);
                    obj_pos(2) += dist(generator);
                    redis_data.at(0) = std::pair<string, string>(CAMERA_DETECT_KEY, true_message);
                    redis_data.at(1) = std::pair<string, string>(CAMERA_OBJ_POS_KEY, redis_client.encodeEigenMatrixJSON(obj_pos));
                }
                else {
                    redis_data.at(0) = std::pair<string, string>(CAMERA_DETECT_KEY, false_message);
                    redis_data.at(1) = std::pair<string, string>(CAMERA_OBJ_POS_KEY, redis_client.encodeEigenMatrixJSON(Vector3d::Zero()));
                }

                k_iter_sim++;
                //cout << "simulation count: " << k_iter_sim << endl;

                // publish all redis keys at once to reduce multiple redis calls that slow down simulation
                // shown explicitly here, but you can define a helper function to publish data
                redis_data.at(2) = std::pair<string, string>(JOINT_ANGLES_KEY, redis_client.encodeEigenMatrixJSON(robot->_q));
                redis_data.at(3) = std::pair<string, string>(JOINT_VELOCITIES_KEY, redis_client.encodeEigenMatrixJSON(robot->_dq));
                redis_data.at(4) = std::pair<string, string>(BALL_POS_KEY, redis_client.encodeEigenMatrixJSON(object->_q));
                redis_data.at(5) = std::pair<string, string>(BALL_VEL_KEY, redis_client.encodeEigenMatrixJSON(object->_dq));
                redis_data.at(6) = std::pair<string, string>(CAMERA_POS_KEY, redis_client.encodeEigenMatrixJSON(camera_pos));
                redis_data.at(7) = std::pair<string, string>(CAMERA_ORI_KEY, redis_client.encodeEigenMatrixJSON(camera_ori));
                redis_data.at(8) = std::pair<string, string>(EE_FORCE_KEY, redis_client.encodeEigenMatrixJSON(sensed_force));
                redis_data.at(9) = std::pair<string, string>(EE_MOMENT_KEY, redis_client.encodeEigenMatrixJSON(sensed_moment));
                redis_data.at(10) = std::pair<string, string>(SIMULATION_LOOP_ITERATION, std::to_string(k_iter_sim)); // tell controller sim loop is done

                redis_client.pipeset(redis_data);
            }
        }
	}

	double end_time = timer.elapsedTime() / time_slowdown_factor;
	std::cout << "\n";
	std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	// std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	// std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
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
	/**
     * @brief Boolean check if specified object is inside camera fov.
     * @param object_pos Object position in world frame.
     * @param camera_pos Camera position in world frame.
     * @param camera_ori Camera DCM matrix from local to world frame.
     * @param radius Camera detection radius.
     * @param fov_angle Camera FOV angle
     */

bool cameraFOV(Vector3d object_pos, Vector3d camera_pos, Matrix3d camera_ori, double radius, double fov_angle) {
	// init
	Vector3d a, b, c, d;
	// Vector3d normal = camera_ori.col(2);  // normal vector in world frame

	// local camera frame vertex coordinates
	Vector3d v1, v2, v3;
	v1 << 0, -radius*tan(fov_angle), radius;
	v2 << radius*tan(fov_angle)*cos(M_PI/6), radius*tan(fov_angle)*sin(M_PI/6), radius;
	v3 << -radius*tan(fov_angle)*cos(M_PI/6), radius*tan(fov_angle)*sin(M_PI/6), radius;

	// world frame vertex coordinates centered at the object
	a = camera_pos - object_pos;
	b = camera_pos + camera_ori*v1 - object_pos;
	c = camera_pos + camera_ori*v2 - object_pos;
	d = camera_pos + camera_ori*v3 - object_pos;

	// calculate if object position is inside tetrahedron
    vector<double> B(4);
    B.at(0) = ( -1*(b(0)*c(1)*d(2) - b(0)*c(2)*d(1) - b(1)*c(0)*d(2) + b(1)*c(2)*d(0) + b(2)*c(0)*d(1) - b(2)*c(1)*d(0)) );
    B.at(1) = ( a(0)*c(1)*d(2) - a(0)*c(2)*d(1) - a(1)*c(0)*d(2) + a(1)*c(2)*d(0) + a(2)*c(0)*d(1) - a(2)*c(1)*d(0) );
    B.at(2) = ( -1*(a(0)*b(1)*d(2) - a(0)*b(2)*d(1) - a(1)*b(0)*d(2) + a(1)*b(2)*d(0) + a(2)*b(0)*d(1) - a(2)*b(1)*d(0)) );
    B.at(3) = ( a(0)*b(1)*c(2) - a(0)*b(2)*c(1) - a(1)*b(0)*c(2) + a(1)*b(2)*c(0) + a(2)*b(0)*c(1) - a(2)*b(1)*c(0) );
    double detM = B.at(0) + B.at(1) + B.at(2) + B.at(3);

	// sign check
	bool test;
	for (int i = 0; i < B.size(); ++i) {
		test = compareSigns(detM, B.at(i));
		if (test == false) {
			return false;
		}
	}
	return true;
}

//------------------------------------------------------------------------------

bool compareSigns(double a, double b) {
    if (a > 0 && b > 0) {
        return true;
    }
    else if (a < 0 && b < 0) {
        return true;
    }
    else {
        return false;
    }
}

//------------------------------------------------------------------------------

bool limitCheck(double a, double b) {
    if (a > 0 && b > 0) {
        return true;
    }
    else if (a < 0 && b < 0) {
        return true;
    }
    else {
        return false;
    }
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
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
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
