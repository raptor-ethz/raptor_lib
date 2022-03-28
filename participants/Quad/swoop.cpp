#include "Quad.h"

void Quad::swoop(Item &target, Gripper &gripper, float length, float dx,
                 float dy, float dz, float h0, int time, int grip_angle)
{
  gripper.setAngleSym(45);
  // start position
  goToPos(target.getPose().position.x + dx - length,
            target.getPose().position.y + dy, h0, 0, 3000, true);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // swoop to object
  goToPos(target.getPose().position.x + dx - 0.2,
            target.getPose().position.y + dy,
            target.getPose().position.z + dz + 0.45, 0, 4500, true);
  goToPos(target.getPose().position.x + dx,
            target.getPose().position.y + dy,
            target.getPose().position.z + dz + 0.28, 0, time, false);
  gripper.setAngleSym(grip_angle);
  std::this_thread::sleep_for(std::chrono::milliseconds(350));

  // swoop away from object
  goToPos(target.getPose().position.x + dx + length,
            target.getPose().position.y + dy, h0, 0, 3000, true);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

// only for demo
void Quad::release(Item &target, Gripper &gripper, float length, float h0,
                   int time)
{
  // start position
  goToPos(target.getPose().position.x + length,
            target.getPose().position.y, h0, 0, 3000, true);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // swoop to object
  goToPos(target.getPose().position.x,
            target.getPose().position.y,
            target.getPose().position.z + 0.50, 0, 2500, false);
  gripper.setAngleSym(45);
  std::this_thread::sleep_for(std::chrono::milliseconds(350));
  gripper.setAngleSym(0);
  // swoop away from object
  goToPos(target.getPose().position.x - length,
            target.getPose().position.y, h0, 0, 3000, true);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

// only for demo
// void Quad::quickRelease(Item &target, Gripper &gripper, float length, float h0,
//                          int time)
// {
//   // start position
//   goToPos(target.getPose().position.x + length,
//             target.getPose().position.y, h0, 0, 3000, false);

//   // define ref position
//   Vec3 pos_ref(target.getPose().position.x,
//                target.getPose().position.y,
//                target.getPose().position.z + 0.50);
//   Vec3 vel_ref(-0.5, 0, 0);
//   Vec3 acc_ref(0, 0, 0);

//   // swoop down
//   go_to_pos_min_jerk(pos_ref, vel_ref, acc_ref, 1.5);
//   std::this_thread::sleep_for(std::chrono::milliseconds(1500));
//   // drop
//   gripper.set_angle_sym(45);

//   // define next ref position
//   set_velocity(Vec3(-0.5, 0, 0));
//   pos_ref = Vec3(target.getPose().position.x - length,
//                  target.getPose().position.y, h0);
//   vel_ref = Vec3(0, 0, 0);

//   // swoop up
//   go_to_pos_min_jerk(pos_ref, vel_ref, acc_ref, 1.5);

//   // close gripper and reset
//   gripper.set_angle_sym(0);
//   set_velocity(Vec3(0, 0, 0));

//   std::this_thread::sleep_for(std::chrono::milliseconds(1000));
// }

void Quad::quickSwoop(Item &target, Gripper &gripper, float length, float dx,
                       float dy, float dz, float h0, int time, int grip_angle)
{
  // attack pose
  gripper.setAngleAsym(79, grip_angle);

  // go to start position
  goToPos(target.getPose().position.x + dx - length,
            target.getPose().position.y + dy, h0, 0, 4500, false);
  // std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // swoop to object
  goToPos(target.getPose().position.x + dx,
            target.getPose().position.y + dy,
            target.getPose().position.z + dz + 0.21, 0, 4500, true);
  // close gripper
  gripper.setAngleSym(grip_angle);
  std::this_thread::sleep_for(std::chrono::milliseconds(time));

  // swoop away from object
  goToPos(target.getPose().position.x + dx + length,
            target.getPose().position.y + dy, h0, 0, 3000, false);
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
}