#include "Gripper.h"
#include "Quad.h"
int main() {
  // FastDDS default participant
  std::unique_ptr<DefaultParticipant> dp =
      std::make_unique<DefaultParticipant>(0, "raptor");

  // Maybe this needs to be changed...
  std::string quad_id = "Quad01";
  std::string gripper_id = "Gripper";
  std::string quad_sub_topic_name = "mocap_srl_quad";
  std::string quad_pub_topic_name = "pos_cmd";
  std::string grip_pub_topic_name = "grip_cmd";

  // create instance of quad
  Quad quad(quad_id, dp, quad_sub_topic_name, quad_pub_topic_name);
  Gripper gripper(gripper_id, dp, grip_pub_topic_name);
  // These parameters will be moved to the Quad class
  // (and maybe also to a yaml)
  // We'll then have getters and setters to change these variables.
  // Also, different function overloads of go_to_pos will be implemented so
  // those values do not have to be passed and the function call is more
  // compact
  // gripper test
  while (true) {
    gripper.open();
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    gripper.close();
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  }
  // flight test
  //   float x_thresh, y_thresh, z_thresh = 0.2;
  //   int delay_time = 20;

  //   // For now, we send the drone the position cmds as follows:
  //   quad.go_to_pos(-1.0, 1.0, 2.0, x_thresh, y_thresh, z_thresh, delay_time,
  //   2000,
  //                  false);

  return 0;
}