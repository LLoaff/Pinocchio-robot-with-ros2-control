#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <iostream>
// #include "IOROS.h"
#include "rclcpp/rclcpp.hpp"

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context
GLFWwindow* window ;
std::thread   thread;


// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;
// sensor_msgs::msg::Imu imu_state;
// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  // backspace: reset simulation
  if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);
    mj_forward(m, d);
  }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  // update button state
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

void initial(){

  char error[1000]="Could not load binary model";
  m = mj_loadXML("/home/loaf/Pinocchio_ws/src/descrip_pino/urdf/Pino_model.xml", 0, error, 1000);
  
  if (!m) {
    mju_error("Load model error: %s", error);
  }
  // make data
  d = mj_makeData(m);
  mj_resetDataKeyframe(m,d,0);
  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }
  window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  // create window, make OpenGL context current, request v-sync
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);
}

void free(){

  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  mj_deleteData(d);
  mj_deleteModel(m);
}

std::vector<float> get_sensor_data(const std::string & s_name){
  int sensor_id = mj_name2id(m,mjOBJ_SENSOR,s_name.c_str());
  int sensor_pos = m->sensor_adr[sensor_id];
  std::vector<float> sensor_data(m->sensor_dim[sensor_id]);

  for (int i = 0; i < sensor_data.size(); i++){
    sensor_data[i] = d->sensordata[sensor_pos+i];
  }
  return sensor_data;
}

void rosRun(){

}

class IOROS : public rclcpp::Node {
public:
    IOROS() : Node("mj_node") { // 仅初始化节点名称
        RCLCPP_INFO(this->get_logger(), "基础节点创建成功！");
    }
};

int main(int argc,char** argv) {
    rclcpp::init(argc,argv);
  auto node = std::make_shared<IOROS>();
  // thread = std::thread([&]{
  //   rclcpp::spin(node);
  // });
  printf("11");
  rclcpp::spin(node);

  // initial();

  // int imu_id = mj_name2id(m,mjOBJ_SITE,"imu");
  // int fr_hip_joint_p = mj_name2id(m,mjOBJ_SENSOR,"fr_hip_joint_p");

  // int fr_hip_id = mj_name2id(m,mjOBJ_JOINT,"fr_hip_joint");
  // int fr_knee_id = mj_name2id(m,mjOBJ_JOINT,"fr_knee_joint");
  // int fr_thigh_id = mj_name2id(m,mjOBJ_JOINT,"fr_thigh_joint");
  // int fl_hip_id = mj_name2id(m,mjOBJ_JOINT,"fl_hip_joint");
  // int fl_knee_id = mj_name2id(m,mjOBJ_JOINT,"fl_knee_joint");
  // int fl_thigh_id = mj_name2id(m,mjOBJ_JOINT,"fl_thigh_joint");
  // int br_hip_id = mj_name2id(m,mjOBJ_JOINT,"br_hip_joint");
  // int br_knee_id = mj_name2id(m,mjOBJ_JOINT,"br_knee_joint");
  // int br_thigh_id = mj_name2id(m,mjOBJ_JOINT,"br_thigh_joint");
  // int bl_hip_id = mj_name2id(m,mjOBJ_JOINT,"bl_hip_joint");
  // int bl_knee_id = mj_name2id(m,mjOBJ_JOINT,"bl_knee_joint");
  // int bl_thigh_id = mj_name2id(m,mjOBJ_JOINT,"bl_thigh_joint");
  // printf("imu_id:%d\n",imu_id);
  // printf("fr_hip_id:%d\n",fr_hip_id);
  // printf("fr_knee_id:%d\n",fr_knee_id);
  // printf("fr_thigh_id:%d\n",fr_thigh_id);
  // printf("fl_hip_id:%d\n",fl_hip_id);
  // printf("fl_knee_id:%d\n",fl_knee_id);
  // printf("fl_thigh_id:%d\n",fl_thigh_id);
  // printf("br_hip_id:%d\n",br_hip_id);
  // printf("br_knee_id:%d\n",br_knee_id);
  // printf("br_thigh_id:%d\n",br_thigh_id);
  // printf("bl_hip_id:%d\n",bl_hip_id);
  // printf("bl_knee_id:%d\n",bl_knee_id);
  // printf("bl_thigh_id:%d\n",bl_thigh_id);

  // printf("fr_hip_joint_p:%d\n",fr_hip_joint_p);


  // while (rclcpp::ok() && !glfwWindowShouldClose(window)) {

  //   mjtNum simstart = d->time;
  //   while (d->time - simstart < 1.0/60.0) {
  //     mj_step(m, d);
  //   }

  //   // auto sensor_data = get_sensor_data("acc");
  //   // std::cout<< sensor_data[0] << " "<<std::endl;
  //   // imu_state.header.stamp=node->get_clock()->now();
  //   // imu_state.linear_acceleration.x = sensor_data[0];
  //   // imu_state.linear_acceleration.y = sensor_data[1];
  //   // imu_state.linear_acceleration.z = sensor_data[2];

  //   // node->pubImu(imu_state) ;

  //   mjrRect viewport = {0, 0, 0, 0};
  //   glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

  //   mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
  //   mjr_render(viewport, &scn, &con);

  //   glfwSwapBuffers(window);

  //   glfwPollEvents();

  // }

  // free();
  return EXIT_SUCCESS;
}
