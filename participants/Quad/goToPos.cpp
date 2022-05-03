#include "Quad.h"

/* Non-member functions */

inline bool checkReachedPos1D(const float &actual_pos,
                              const float &reference_pos,
                              const float &threshold)
{
  return std::abs(reference_pos - actual_pos) <= threshold;
}

bool checkReachedPos3D(const float &x_actual, const float &x_ref,
                       const float &x_thresh, const float &y_actual,
                       const float &y_ref, const float &y_thresh,
                       const float &z_actual, const float &z_ref,
                       const float &z_thresh)
{
  bool x_reach_flag = checkReachedPos1D(x_actual, x_ref, x_thresh);
  bool y_reach_flag = checkReachedPos1D(y_actual, y_ref, y_thresh);
  bool z_reach_flag = checkReachedPos1D(z_actual, z_ref, z_thresh);

  return x_reach_flag && y_reach_flag && z_reach_flag;
}

/* Member functions */

bool Quad::sendPosCmd(const float x_ref, const float y_ref, const float z_ref, const float yaw)
{
  // TODO feasibility checks
  // int X_std::MIN, X_std::MAX, Y_std::MIN, Y_std::MAX, Z_std::MIN, Z_std::MAX;

  // if (x < X_std::MIN || x > X_std::MAX || y < Y_std::MIN || y > Y_std::MAX || z < Z_std::MIN ||
  //     z > Z_std::MAX) {
  //   std::cout << "[ERROR][Participant: " << id_
  //             << "] Position Command not feasible." << std::endl;
  //   return false;
  // }

  // get absolute values
  const float x = (x_ref > 0) ? x_ref : -x_ref;
  const float y = (y_ref > 0) ? y_ref : -y_ref;
  const float z = (z_ref > 0) ? z_ref : -z_ref;

  if (x < 0.001 && y < 0.001 && z < 0.001)
  {
    consoleError("Position command not feasible (0).");
    return false;
  }

  pos_cmd_.position.x = x_ref;
  pos_cmd_.position.y = y_ref;
  pos_cmd_.position.z = z_ref;
  pos_cmd_.yaw_angle = yaw;
  // std::cout << pos_cmd_.position.x << " " << pos_cmd_.position.y << pos_cmd_.position.z << std::endl;
  // publish pos_cmd
  position_pub_->publish(pos_cmd_);

  return true;
}

// full config
bool Quad::goToPos(const float &x_ref, const float &y_ref, const float &z_ref,
                   const float &yaw_ref, const float &x_thresh,
                   const float &y_thresh, const float &z_thresh,
                   const int &delay_time, const float &max_time,
                   const bool &reached_pos_flag)
{
  consoleDebug("Going to position: [" + std::to_string(x_ref) + ", " +
               std::to_string(y_ref) + ", " + std::to_string(z_ref) + ", " +
               std::to_string(yaw_ref) + "] during max " +
               std::to_string(max_time) + "ms.");

  // timestamp
  std::chrono::time_point<std::chrono::steady_clock> loop_timer;
  // resulting bool
  bool result = false;

  for (float t = 0; t < max_time; t += delay_time)
  {
    // get start time
    loop_timer = std::chrono::steady_clock::now();
    // check mocap TODO
    if (!checkMocapData())
    {
      // state_ = State::hover;
    }
    // check external message
    // check if subscriber is connected, otherwise skip
    if (ui_sub_->listener->matched())
    {
      switch (ui_cmd_.command)
      {
      // skip on status
      case User_cmd::ui_null:
        break;

      case User_cmd::ui_hover:
        state_ = State::hover;
        ui_cmd_.command = User_cmd::ui_null;
        break;

      case User_cmd::ui_emg_land:
        state_ = State::emg_land;
        ui_cmd_.command = User_cmd::ui_null;
        break;

      case User_cmd::ui_land:
        state_ = State::land;
        ui_cmd_.command = User_cmd::ui_null;
        break;

      default:
        break;
      }
    }

    // check if interface is matched
    while (!px4_action_pub_->listener.matched())
    {
      consoleError("Connection to PX4 interface lost. Reconnecting...");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // check flag
    // TODO move out to (reusable) separate function
    switch (state_)
    {
    case State::land:
      consoleWarning("Flag interruption: Land in stand.");
      // check if a stand is registered
      if (stand_ != nullptr)
      {
        // change state to airborne
        state_ = State::airborne;
        // call land
        land(*stand_);
        // exit programm
        exit(0);
      }
      else
      {
        consoleError("No stand registered. Activate hover mode.");
        state_ = State::airborne;
        hover();
      }
      break;

    case State::emg_land:
      consoleWarning("Flag interruption: Emergency land.");
      emergencyLand();
      exit(0);
      break;

    case State::hover:
      consoleWarning("Flag interruption: Hover.");
      state_ = State::airborne;
      hover();
      break;
    }

    // check if reference position has been reached
    result =
        checkReachedPos3D(pose_.position.x, x_ref, x_thresh, pose_.position.y,
                          y_ref, y_thresh, pose_.position.z, z_ref, z_thresh);

    // return if position is reached and return is desired
    if (result && reached_pos_flag)
    {
      if (console_state_ == 0)
      {
        consoleDebug("Position reached before time limit.");
      }
      return result;
    }
    else
    {
      // TODO send pos cmd
      sendPosCmd(x_ref, y_ref, z_ref, yaw_ref);
      // send new pos_cmd if position hasn't been reached
      // pos_cmd_.position.x = x_ref;
      // pos_cmd_.position.y = y_ref;
      // pos_cmd_.position.z = z_ref;
      // pos_cmd_.yaw_angle = yaw_ref;
      // // publish pos_cmd
      // position_pub_->publish(pos_cmd_);
    }

    // control frequency
    loop_timer += std::chrono::milliseconds(delay_time);
    std::this_thread::sleep_until(loop_timer);
  }

  if (result)
  {
    consoleDebug("Position reached after time limit.");
  }
  else
  {
    consoleDebug("Position not reached within time limit.");
  }
  return result;
}

bool Quad::goToPos(const float &x_ref, const float &y_ref, const float &z_ref,
                   const float &yaw_ref, const int &delay_time,
                   const float &max_time, const bool &reached_pos_flag)
{
  return goToPos(x_ref, y_ref, z_ref, yaw_ref, x_thresh_, y_thresh_, z_thresh_,
                 delay_time, max_time, reached_pos_flag);
}
// using default threshold

bool Quad::goToPos(const float &x_ref, const float &y_ref, const float &z_ref,
                   const float &yaw_ref, const float &max_time,
                   const bool &reached_pos_flag)
{
  return goToPos(x_ref, y_ref, z_ref, yaw_ref, x_thresh_, y_thresh_, z_thresh_,
                 delay_time_, max_time, reached_pos_flag);
}
// using default threshold and delay

bool Quad::goToPos(Item &target, const float &x_offset, const float &y_offset,
                   const float &z_offset, const float &yaw_ref,
                   const float &max_time, const bool &reached_pos_flag)
{
  return goToPos(target.getPose().position.x + x_offset,
                 target.getPose().position.y + y_offset,
                 target.getPose().position.z + z_offset, yaw_ref, x_thresh_,
                 y_thresh_, z_thresh_, delay_time_, max_time, reached_pos_flag);
}
// go to target object with offset, default thresh and delay

///////////////////////////////////////////////////////////////////////TEMP -> will go to astar constructor

int gridx_size = 4, gridy_size = 4, gridz_size = 4;

// TODO: floor might not be imported yet, test
std::vector<int> convertPositionToGrid(std::vector<float> grid_start, std::vector<float> grid_end, std::vector<float> point)
{
  float x_0 = grid_start[0], x_1 = grid_end[0], y_0 = grid_start[1], y_1 = grid_end[1], z_0 = grid_start[2], z_1 = grid_end[2];
  float step_x = (x_1 - x_0) / gridx_size, step_y = (y_1 - y_0) / gridy_size, step_z = (z_1 - z_0) / gridz_size;
  return {(int)floor((point[0] + 0.01 - x_0) / step_x), (int)floor((point[1] + 0.01 - y_0) / step_y), (int)floor((point[2] + 0.01 - z_0) / step_z)};
}

std::vector<float> convertGridToPosition(std::vector<float> grid_start, std::vector<float> grid_end, std::vector<int> point)
{
  float x_0 = grid_start[0], x_1 = grid_end[0], y_0 = grid_start[1], y_1 = grid_end[1], z_0 = grid_start[2], z_1 = grid_end[2];
  float step_x = (x_1 - x_0) / gridx_size, step_y = (y_1 - y_0) / gridy_size, step_z = (z_1 - z_0) / gridz_size;
  return {x_0 + point[0] * step_x, y_0 + point[1] * step_y, z_0 + point[2] * step_z};
}

int pointToVertex(const std::vector<int> &point)
{
  return point[0] * gridy_size + point[1];
}

int pointToVertex3D(const std::vector<int> &point)
{
  return point[0] * gridy_size * gridz_size + point[1] * gridz_size + point[2];
}

std::vector<int> vertexToPoint(int vertex)
{
  std::vector<int> result = {vertex / gridy_size, vertex % gridy_size};
  return result;
}

std::vector<int> vertexToPoint3D(int vertex)
{
  std::vector<int> result = {vertex / (gridz_size * gridy_size), (vertex / gridz_size) % gridy_size, vertex % gridz_size};
  return result;
}

/// TODO assert if drone is in blocked position
void initializeGrid(const std::vector<std::vector<int>> &points,
                    std::vector<std::vector<int>> &grid)
{
  for (int i = 0; i < gridx_size; ++i)
  {
    std::vector<int> row;
    for (int j = 0; j < gridy_size; ++j)
    {
      row.push_back(0);
    }
    grid.push_back(row);
  }

  for (int i = 0, n = points.size(); i < n; ++i)
  {
    int x = points[i][0];
    int y = points[i][1];
    grid[x][y] = 1;
  }
}

void initializeGrid(const std::vector<std::vector<int>> &points,
                    std::vector<std::vector<std::vector<int>>> &grid)
{
  // initialize grid
  for (int i = 0; i < gridx_size; ++i)
  {
    std::vector<std::vector<int>> row;
    for (int j = 0; j < gridy_size; ++j)
    {
      std::vector<int> height;
      for (int k = 0; k < gridz_size; ++k)
      {
        height.push_back(0);
      }
      row.push_back(height);
    }
    grid.push_back(row);
  }

  for (int i = 0, n = points.size(); i < n; ++i)
  {
    int x = points[i][0], y = points[i][1], z = points[i][2];
    grid[x][y][z] = 1;
  }
}

///////////////////////////////////////////////////////////////////////TEMP

// go to position with a_star pathplanning
void Quad::goToPosAstar(std::vector<float> start_coords, std::vector<float> end_coords, Obstacle obstacle)
{
  // put
  std::vector<std::vector<float>> obs_coords;
  for (int i = 0; i < obstacle.getMarkers().length; i++)
  {
    std::vector<float> coord{obstacle.getMarkers().marker_x[i], obstacle.getMarkers().marker_y[i], obstacle.getMarkers().marker_z[i]};
    obs_coords.push_back(coord);
  }

  // grid
  std::vector<std::vector<std::vector<int>>> grid;

  // start / ending
  std::vector<float>
      grid_start = {std::min(start_coords[0], end_coords[0]) - 1, std::min(start_coords[1], end_coords[1]) - 1, std::min(start_coords[2], end_coords[2]) - 1};
  std::vector<float> grid_end = {std::max(start_coords[0], end_coords[0]) + 1, std::max(start_coords[1], end_coords[1]) + 1, std::max(start_coords[2], end_coords[2]) + 1};

  std::vector<int> start = convertPositionToGrid(grid_start, grid_end, start_coords);
  std::vector<int> end = convertPositionToGrid(grid_start, grid_end, end_coords);

  std::vector<std::vector<int>> coords;
  for (int i = 0, n = obs_coords.size(); i < n; ++i)
    coords.push_back(convertPositionToGrid(grid_start, grid_end, obs_coords[i]));

  for (int i = 0; i < gridx_size; i++)
  {
    std::vector<std::vector<int>> tmp2D;
    for (int j = 0; j < gridy_size; j++)
    {
      std::vector<int> tmp1D;
      for (int k = 0; k < gridz_size; k++)
      {
        tmp1D.push_back(0);
      }
      tmp2D.push_back(tmp1D);
    }
    grid.push_back(tmp2D);
  }

  initializeGrid(coords, grid);

  // graph
  Astar solver(grid, false);

  // std::chrono::steady_clock::time_point start_astar =
  // std::chrono::steady_clock::now();
  std::vector<int> vertices_astar =
      solver.astarPath(pointToVertex3D(start), pointToVertex3D(end));
  // std::chrono::steady_clock::time_point end_astar =
  // std::chrono::steady_clock::now(); std::cout << "A* time difference: " <<
  // std::chrono::duration_cast<std::chrono::milliseconds>(end_astar -
  // start_astar).count() << " milliseconds.\n";
  // assert(vertices_astar == vertices_dijkstra);
  std::vector<int> prev = vertexToPoint3D(vertices_astar[0]);
  for (int i = 1, n = vertices_astar.size(); i < n; ++i)
  {
    std::vector<int> point_grid = vertexToPoint3D(vertices_astar[i]);
    if (abs(point_grid[0] - prev[0]) + abs(point_grid[1] - prev[1]) + abs(point_grid[2] - prev[2]) == 1)
    {
      prev = {point_grid[0], point_grid[1], point_grid[2]};
      continue;
    }
    std::vector<float> point = convertGridToPosition(grid_start, grid_end, point_grid);
    // std::cout<<"x: "<<x_0 + point[0]*stepSize<<std::endl;
    // std::cout<<"y: "<<y_0 + point[1]*stepSize<<std::endl;
    // std::cout<<"z: "<<z_0 + point[2]*stepSize<<std::endl;
    goToPos(point[0], point[1], point[2], 0,
            3000, false);
  }
}