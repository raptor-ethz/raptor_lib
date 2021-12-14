#include "Quad.h"
#include <chrono>

inline bool check_reached_pos_1d(   const float &actual_pos, 
                                    const float &reference_pos, 
                                    const float &threshold)
{
    return std::abs(reference_pos - actual_pos) <= threshold;
}

inline bool check_reached_pos_3d(   const float &x_actual, 
                                    const float &x_ref, 
                                    const float &x_thresh,
                                    const float &y_actual, 
                                    const float &y_ref, 
                                    const float &y_thresh,
                                    const float &z_actual, 
                                    const float &z_ref, 
                                    const float &z_thresh)
{
    bool x_reach_flag = check_reached_pos_1d(  x_actual, x_ref, x_thresh);
    bool y_reach_flag = check_reached_pos_1d(  y_actual, y_ref, y_thresh);
    bool z_reach_flag = check_reached_pos_1d(  z_actual, z_ref, z_thresh);
    
    return x_reach_flag && y_reach_flag && z_reach_flag;
}

bool Quad::go_to_pos(   const float &x_ref, const float &y_ref, const float &z_ref, 
                        const float &x_thresh, const float &y_thresh, 
                        const float &z_thresh, const int &delay_time,
                        const float &max_time, const bool &reached_pos_flag)
{
    // DEBUG
    std::cout << "Go to position: [\t" 
                << x_ref << ",\t" << y_ref << ",\t" << z_ref 
                << "\t] during max" << max_time << "ms." << std::endl;
    // DEBUG END

    // resulting bool
    bool result = false;
    

    for (float timer = 0; timer < max_time; timer += delay_time)
    {
        // check if reference position has been reached
        result = check_reached_pos_3d(  pose_.pose.position.x, x_ref, x_thresh,
                                        pose_.pose.position.y, y_ref, y_thresh,
                                        pose_.pose.position.z, z_ref, z_thresh);


        if (result && reached_pos_flag)
        {
            // DEBUG
            std::cout << "Position reached (return)." << std::endl;
            // DEBUG END

            // return from the function direclty
            return result;
        } else {
            // send new pos_cmd if position hasn't been reached
            pos_cmd.position.x = x_ref;
            pos_cmd.position.y = y_ref;
            pos_cmd.position.z = z_ref;

            // publich pos_cmd
            position_pub->publish(pos_cmd);
        }

        // delay
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_time));
        
    }

    // DEBUG
    if (result)
    {
        std::cout << "Position reached after time limit." << std::endl;
    } else {
        std::cout << "Position wasn't reached within time limit." << std::endl;
    }
    // DEBUG END

    return result;
}