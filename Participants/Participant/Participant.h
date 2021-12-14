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
             * checks if the last entry is non-zero.
             * 
             * @returns if (position.x != 0.0)
            **/
            virtual bool check_for_data();

        protected:
            cpp_msg::Mocap pose_{};
    };

}