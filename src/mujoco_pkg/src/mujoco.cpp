#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context
GLFWwindow* window ;
std::thread   thread;

bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;
// sensor_msgs::msg::Imu imu_state;

// void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
//   // backspace: reset simulation
//   if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
//     mj_resetData(m, d);
//     mj_forward(m, d);
//   }
// }

void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  // update button state
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}

// void mouse_move(GLFWwindow* window, double xpos, double ypos) {
//   // no buttons down: nothing to do
//   if (!button_left && !button_middle && !button_right) {
//     return;
//   }

//   // compute mouse displacement, save
//   double dx = xpos - lastx;
//   double dy = ypos - lasty;
//   lastx = xpos;
//   lasty = ypos;

//   // get current window size
//   int width, height;
//   glfwGetWindowSize(window, &width, &height);

//   // get shift key state
//   bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
//                     glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

//   // determine action based on mouse button
//   mjtMouse action;
//   if (button_right) {
//     action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
//   } else if (button_left) {
//     action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
//   } else {
//     action = mjMOUSE_ZOOM;
//   }

//   // move camera
//   mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
// }

// void scroll(GLFWwindow* window, double xoffset, double yoffset) {
//   // emulate vertical mouse motion = 5% of window height
//   mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
// }

// void initial(){

//   char error[1000]="Could not load binary model";
//   m = mj_loadXML("/home/loaf/Pinocchio_ws/src/descrip_pino/urdf/Pino_model.xml", 0, error, 1000);
  
//   if (!m) {
//     mju_error("Load model error: %s", error);
//   }
//   // make data
//   d = mj_makeData(m);
//   mj_resetDataKeyframe(m,d,0);
//   // init GLFW
//   if (!glfwInit()) {
//     mju_error("Could not initialize GLFW");
//   }
//   window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
//   // create window, make OpenGL context current, request v-sync
//   glfwMakeContextCurrent(window);
//   glfwSwapInterval(1);

//   // initialize visualization data structures
//   mjv_defaultCamera(&cam);
//   mjv_defaultOption(&opt);
//   mjv_defaultScene(&scn);
//   mjr_defaultContext(&con);

//   // create scene and context
//   mjv_makeScene(m, &scn, 2000);
//   mjr_makeContext(m, &con, mjFONTSCALE_150);

//   // install GLFW mouse and keyboard callbacks
// //   glfwSetKeyCallback(window, keyboard);
//   glfwSetCursorPosCallback(window, mouse_move);
//   glfwSetMouseButtonCallback(window, mouse_button);
//   glfwSetScrollCallback(window, scroll);
// }

// void free(){

//   mjv_freeScene(&scn);
//   mjr_freeContext(&con);

//   mj_deleteData(d);
//   mj_deleteModel(m);
// }

// std::vector<float> get_sensor_data(const std::string & s_name){
//   int sensor_id = mj_name2id(m,mjOBJ_SENSOR,s_name.c_str());
//   int sensor_pos = m->sensor_adr[sensor_id];
//   std::vector<float> sensor_data(m->sensor_dim[sensor_id]);

//   for (int i = 0; i < sensor_data.size(); i++){
//     sensor_data[i] = d->sensordata[sensor_pos+i];
//   }
//   return sensor_data;
// }


class IOROS : public rclcpp::Node {
public:
    IOROS() : Node("mj_node") { // 仅初始化节点名称
        RCLCPP_INFO(this->get_logger(), "基础节点创建成功！");
    }
};

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node = std::make_shared<IOROS>();

  printf("11");
  rclcpp::spin(node);

  return 0;
}
