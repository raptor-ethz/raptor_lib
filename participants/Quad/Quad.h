#pragma once

#include "Participant.h"
#include "Item.h"
#include "QuadPositionCmdPubSubTypes.h"
#include "RapidTrajectoryGenerator.h"
#include "Vec3.h"

class Quad : public raptor::Participant{
    public:

    Quad(   const std::string &raptor_participant_id,
            std::unique_ptr<DefaultParticipant> &dp, 
            const std::string &sub_topic_name,
            const std::string &pub_topic_name);
    ~Quad();

    DDSPublisher *position_pub;

    /** 
     * Send a position command to this drone with a reference position.
     * Will return once the time is reached, or, if the reached_pos_flag is
     * set to true, if the position has been reached within the threshold.
     * @param [in] x_ref : [m] Reference position - x coordinate
     * @param [in] y_ref : [m] Reference position - y coordinate
     * @param [in] z_ref : [m] Reference position - z coordinate
     * @param [in] x_thresh : [m] maximal accepted deviation from x_ref
     * @param [in] y_thresh : [m] maximal accepted deviation from y_ref
     * @param [in] z_thresh : [m] maximal accepted deviation from z_ref
     * @param [in] delay_time : [ms] delay between commands
     * @param [in] max_time : [ms] The time after which the function latest ends, 
     * even if the position wasn't reached
     * @param [in] reached_pos_flag : Flag if the function should end once
     * the position was reached. If false, the function always waits for 
     * max_time before ending
     * @returns If the position has been reached when the function ends
     * 
    **/
    bool go_to_pos( const float &x_ref, const float &y_ref, const float &z_ref, 
                    const float &x_thresh, const float &y_thresh, 
                    const float &z_thresh, const int &delay_time,
                    const float &max_time, const bool &reached_pos_flag);
    
    /** 
     * Send a position command to this drone with a reference position.
     * Will return once the time is reached, or, if the reached_pos_flag is
     * set to true, if the position has been reached within the threshold
     * previously defined in the instance.
     * @param [in] x_ref : [m] Reference position - x coordinate
     * @param [in] y_ref : [m] Reference position - y coordinate
     * @param [in] z_ref : [m] Reference position - z coordinate
     * @param [in] delay_time : [ms] delay between commands
     * @param [in] max_time : [ms] The time after which the function latest ends, 
     * even if the position wasn't reached
     * @param [in] reached_pos_flag : Flag if the function should end once
     * the position was reached. If false, the function always waits for 
     * max_time before ending
     * @returns If the position has been reached when the function ends
     * 
    **/
    bool go_to_pos( const float &x_ref, const float &y_ref, const float &z_ref,
                    const int &delay_time, const float &max_time, 
                    const bool &reached_pos_flag);

    /** 
     * Send a position command to this drone with a reference position.
     * Will return once the time is reached, or, if the reached_pos_flag is
     * set to true, if the position has been reached within the threshold
     * and delay time previously defined in this instance.
     * @param [in] x_ref : [m] Reference position - x coordinate
     * @param [in] y_ref : [m] Reference position - y coordinate
     * @param [in] z_ref : [m] Reference position - z coordinate
     * @param [in] max_time : [ms] The time after which the function latest ends, 
     * even if the position wasn't reached
     * @param [in] reached_pos_flag : Flag if the function should end once
     * the position was reached. If false, the function always waits for 
     * max_time before ending
     * @returns If the position has been reached when the function ends
     * 
    **/
    bool go_to_pos( const float &x_ref, const float &y_ref, const float &z_ref,
                    const float &max_time, const bool &reached_pos_flag);

    /** 
     * Compute a trajectory between the current state of the drone and the 
     * specified reference state during the duration of completion_time.
     * Then, send position commands to the drone to follow the trajectory
     * using the previously defined delay_time in this instance.
     * @param [in] pos_ref : [m] Reference position
     * @param [in] vel_ref : [m] Reference velocity
     * @param [in] acc_ref : [m] Reference acceleration
     * @param [in] completion_time : [s] Reference time for the drone to fly
     * the trajectory
     * @returns If the position has been reached when the function ends
     * 
    **/
    bool go_to_pos_min_jerk(const Vec3 &pos_ref, 
                            const Vec3 &vel_ref,
                            const Vec3 &acc_ref,
                            const int &completion_time);

    void land(Item stand);


    void set_thresh(const float x_thresh, 
                    const float y_thresh, 
                    const float z_thresh)
    {
        x_thresh_ = x_thresh;
        y_thresh_ = y_thresh;
        z_thresh_ = z_thresh;
    }

    private:
    cpp_msg::QuadPositionCmd pos_cmd{};

    float x_thresh_{0.2};
    float y_thresh_{0.2};
    float z_thresh_{0.2};

    int delay_time_{20};

    Vec3 position_{};
    Vec3 velocity_{0, 0, 0};
    Vec3 acceleration_{0, 0, 0};

    const Vec3 gravity_{0, 0, -9.81};
};

inline bool check_reached_pos_1d(   const float &actual_pos, 
                                    const float &reference_pos, 
                                    const float &threshold);

inline bool check_reached_pos_3d(   const float &x_actual, 
                                    const float &x_ref, 
                                    const float &x_thresh,
                                    const float &y_actual, 
                                    const float &y_ref, 
                                    const float &y_thresh,
                                    const float &z_actual, 
                                    const float &z_ref, 
                                    const float &z_thresh);