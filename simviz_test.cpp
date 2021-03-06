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

#include <signal.h>
bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;
using namespace Eigen;

const string world_file = "./resources/world_test.urdf";
const string robot_file = "./resources/mmp_panda.urdf";
const string obj_file = "./resources/volleyBall.urdf";
const string robot1_name = "mmp_panda1";
const string robot2_name = "mmp_panda2";
const string obj_name = "ball";
const string camera_name = "camera_fixed";
const string ee_link_name = "link7";
const string obj_link_name = "link6";

// redis keys:
// - write:
const std::string JOINT_ANGLES1_KEY = "sai2::cs225a::project1::sensors::q";
const std::string JOINT_VELOCITIES1_KEY = "sai2::cs225a::project1::sensors::dq";
const std::string JOINT_ANGLES2_KEY = "sai2::cs225a::project2::sensors::q";
const std::string JOINT_VELOCITIES2_KEY = "sai2::cs225a::project2::sensors::dq";
const std::string OBJ_POSITION_KEY  = "cs225a::robot::ball::sensors::q";
const std::string OBJ_VELOCITIES_KEY = "cs225a::robot::ball::sensors::dq";
const std::string OBJ_PREDICT_KEY = "cs225a::robot::ball::sensors::predict";
// - read
const std::string JOINT_TORQUES1_COMMANDED_KEY = "sai2::cs225a::project1::actuators::fgc";
const std::string JOINT_TORQUES2_COMMANDED_KEY = "sai2::cs225a::project2::actuators::fgc";

RedisClient redis_client;

// simulation function protot
void simulation(Sai2Model::Sai2Model* robot1, Sai2Model::Sai2Model* robot2, Sai2Model::Sai2Model* object, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget);

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

int main() {
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
    graphics->getCamera(camera_name)->setClippingPlanes(2,20);

	// load robot1
	Vector3d robot1_offset = Vector3d(0.0, -5.0, 0.5);
	Matrix3d R_world_robot1;
	R_world_robot1 = AngleAxisd(0.0, Vector3d::UnitZ())
						* AngleAxisd(0.0, Vector3d::UnitY())
						* AngleAxisd(0.0, Vector3d::UnitX());

	Affine3d T_world_robot1 = Affine3d::Identity();
	T_world_robot1.translation() = robot1_offset;
	T_world_robot1.linear() = R_world_robot1;

	auto robot1 = new Sai2Model::Sai2Model(robot_file, false, T_world_robot1);
	robot1->updateModel();

	// load robot2
	Vector3d robot2_offset = Vector3d(0.0, 4.0, 0.5);
	Matrix3d R_world_robot2;
	R_world_robot2 = AngleAxisd(0.0, Vector3d::UnitZ())
						* AngleAxisd(0.0, Vector3d::UnitY())
						* AngleAxisd(0, Vector3d::UnitX());

	Affine3d T_world_robot2 = Affine3d::Identity();
	T_world_robot2.translation() = robot2_offset;
	T_world_robot2.linear() = R_world_robot2;

	auto robot2 = new Sai2Model::Sai2Model(robot_file, false, T_world_robot2);
	robot2->_q(2) = 3.1415926535898;
	robot2->updateModel();

	// load objects
	Vector3d object_offset = Vector3d(0.0, 5.0, 3.0);
	Matrix3d R_world_object;
	R_world_object = AngleAxisd(0.0, Vector3d::UnitZ())
						* AngleAxisd(0.0, Vector3d::UnitY())
						* AngleAxisd(0.0, Vector3d::UnitX());

	Affine3d T_world_object = Affine3d::Identity();
	T_world_object.translation() = object_offset;
	T_world_object.linear() = R_world_object;

	auto object = new Sai2Model::Sai2Model(obj_file, false);
	object->updateModel();

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setCollisionRestitution(1.0); //0.759, tennis
	sim->setCoeffFrictionStatic(0.15);
	sim->setCoeffFrictionDynamic(0.15);

	// read joint positions, velocities, update kinematics
	sim->setJointPositions(robot1_name, robot1->_q);
	sim->setJointVelocities(robot1_name, robot1->_dq);
	// robot1->updateKinematics();

	sim->setJointPositions(robot2_name, robot2->_q);
	sim->setJointVelocities(robot2_name, robot2->_dq);
	// robot2->updateKinematics();

	// read object positions, velocities, update kinematics
	sim->setJointPositions(obj_name, object->_q);
	sim->setJointVelocities(obj_name, object->_dq);
	// object->updateKinematics();

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

	// init click force widget. Need to copy this??
	auto ui_force_widget = new UIForceWidget(robot1_name, robot1, graphics);
	ui_force_widget->setEnable(false);

	// cache variables
	double last_cursorx, last_cursory;

	// initialize glew
	glewInitialize();

	fSimulationRunning = true;
	thread sim_thread(simulation, robot1, robot2, object, sim, ui_force_widget);
	
	// while window is open:
	while (!glfwWindowShouldClose(window) && fSimulationRunning)
	{
		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot1_name, robot1);
		graphics->updateGraphics(robot2_name, robot2);
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
			object->_q(0) = 0.0;
			object->_q(1) = .0;
			object->_q(2) = 0.0;
			object->_dq(0) = +0.05;//-.5+0.01*(rand()%150);
			object->_dq(1) = -6.9;//-9.0+0.02*(rand()%100); // 7.9 is a good value
			object->_dq(2) = 5.0;
			object->_dq(3) = 0.0; // x spin
			object->_dq(4) = 0.0; // x spin
			object->_dq(5) = 0.0; // x spin

			sim->setJointPositions(obj_name, object->_q);
			sim->setJointVelocities(obj_name, object->_dq);
		}
	}

	// stop simulation
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
void simulation(Sai2Model::Sai2Model* robot1, Sai2Model::Sai2Model* robot2, Sai2Model::Sai2Model* object, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget) {

	int dof1 = robot1->dof();
	VectorXd command_torques1 = VectorXd::Zero(dof1);
	redis_client.setEigenMatrixJSON(JOINT_TORQUES1_COMMANDED_KEY, command_torques1);

	int dof2 = robot2->dof();
	VectorXd command_torques2 = VectorXd::Zero(dof2);
	redis_client.setEigenMatrixJSON(JOINT_TORQUES2_COMMANDED_KEY, command_torques2);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double time_slowdown_factor = 1.0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime() / time_slowdown_factor;
	double last_time = start_time; //secs
	

	// init variables
	VectorXd g1(dof1);
	VectorXd g2(dof2);

	Eigen::Vector3d ui_force;
	ui_force.setZero();

	Eigen::VectorXd ui_force_command_torques;
	ui_force_command_torques.setZero();
    
    // Init prediction
    Eigen::Vector3d predictedLanding;
    predictedLanding.setZero();

	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// get gravity torques
		robot1->gravityVector(g1);
		robot2->gravityVector(g2);

		// read arm torques from redis and apply to simulated robot
		command_torques1 = redis_client.getEigenMatrixJSON(JOINT_TORQUES1_COMMANDED_KEY);
		command_torques2 = redis_client.getEigenMatrixJSON(JOINT_TORQUES2_COMMANDED_KEY);
		
		ui_force_widget->getUIForce(ui_force);
		ui_force_widget->getUIJointTorques(ui_force_command_torques);

		if (fRobotLinkSelect)
			sim->setJointTorques(robot1_name, command_torques1 + ui_force_command_torques + g1);
		else
			sim->setJointTorques(robot1_name, command_torques1 + g1);
			sim->setJointTorques(robot2_name, command_torques2 + g2);

		// integrate forward
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time; 
		sim->integrate(0.001);

		// read joint positions, velocities, update model
		sim->getJointPositions(robot1_name, robot1->_q);
		sim->getJointVelocities(robot1_name, robot1->_dq);
		robot1->updateModel();

		sim->getJointPositions(robot2_name, robot2->_q);
		sim->getJointVelocities(robot2_name, robot2->_dq);
		robot2->updateModel();

		// read object positions, velocities, update model
		sim->getJointPositions(obj_name, object->_q);
		sim->getJointVelocities(obj_name, object->_dq);
		object->updateModel();

		// write new robot state to redis
		redis_client.setEigenMatrixJSON(JOINT_ANGLES1_KEY, robot1->_q);
		redis_client.setEigenMatrixJSON(JOINT_VELOCITIES1_KEY, robot1->_dq);

		redis_client.setEigenMatrixJSON(JOINT_ANGLES2_KEY, robot2->_q);
		redis_client.setEigenMatrixJSON(JOINT_VELOCITIES2_KEY, robot2->_dq);

		// write new object state to redis
		redis_client.setEigenMatrixJSON(OBJ_POSITION_KEY, object->_q);
		redis_client.setEigenMatrixJSON(OBJ_VELOCITIES_KEY, object->_dq);
        
        if (object->_environmental_contacts.size() != 0) {
        		std::cout << "In Contact" << std::endl;
                Vector3d ball_launch_pos;
                object->positionInWorld(ball_launch_pos, obj_link_name, Vector3d::Zero());
                Vector3d ball_launch_vel;
                object->linearVelocityInWorld(ball_launch_vel, obj_link_name, Vector3d::Zero());
                double time_flight = (-ball_launch_vel(2) + sqrt(pow(ball_launch_vel(2),2.0) - 4.0*(-9.81/2.0)*ball_launch_pos(2)))/(2*(-9.81/2)); // quadratic equation
                double x_f = ball_launch_vel(0)*time_flight + ball_launch_pos(0);
                double y_f = ball_launch_vel(1)*time_flight + ball_launch_pos(1);
                predictedLanding = Vector3d(x_f, y_f, 0);
            }
        
        // write predicted landing
        redis_client.setEigenMatrixJSON(OBJ_PREDICT_KEY, predictedLanding);
       
        //update last time
        last_time = curr_time;
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
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

