#pragma once

#include "MocapPubSubTypes.h"
#include "QuadPositionCmdPubSubTypes.h"
#include "default_participant.h"
#include "default_publisher.h"
#include "default_subscriber.h"
#include "geometry_msgs/msgs/Position.h"
#include "quadcopter_msgs/msgs/QuadPositionCmd.h"
#include "sensor_msgs/msgs/Mocap.h"

namespace raptor{

    class Participant {
        public:
            DDSSubscriber<idl_msg::MocapPubSubType, cpp_msg::Mocap> *mocap_sub;

            /**
             * Reads 30 datapoints from the subscriber and 
             * checks for a non-zero last datapoint.
             * 
             * @returns if (position.x != 0.0)
            **/
            virtual bool check_for_data();

            virtual const cpp_msg::Mocap& get_pose();

            virtual const std::string& get_id();

        protected:
            std::string id = "N/A";
            cpp_msg::Mocap pose_{};
    };

}