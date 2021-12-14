#pragma once

#include "Participant.h"

class Quad : public raptor::Participant{
    public:

    Quad(   std::unique_ptr<DefaultParticipant> dp, 
            std::string sub_topic_name,
            std::string pub_topic_name);
    ~Quad();

    DDSPublisher *position_pub;

    /** 
     * Send a position command to this drone with a reference position.
     * Will return once the time is reached, or, if the reached_pos_flag is
     * set to true, if the position has been reached within the threshold.
     * @param [in] x_ref : Reference position - x coordinate
     * @param [in] y_ref : Reference position - y coordinate
     * @param [in] z_ref : Reference position - z coordinate
     * @param [in] x_thresh : maximal accepted deviation from x_ref
     * @param [in] y_thresh : maximal accepted deviation from y_ref
     * @param [in] z_thresh : maximal accepted deviation from z_ref
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

    private:
    cpp_msg::QuadPositionCmd pos_cmd{};
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