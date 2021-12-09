#include "Gripper.h"

Gripper::Gripper(   std::unique_ptr<DefaultParticipant> dp, 
                    std::string sub_topic_name,
                    std::string pub_topic_name) {

    mocap_sub = new DDSSubscriber(idl_msg::MocapPubSubType(), &pose_,
                                     sub_topic_name, dp->participant());

    mocap_sub->init();
    
    position_pub = new DDSPublisher(idl_msg::QuadPositionCmdPubSubType(),
                                    pub_topic_name, dp->participant());

    position_pub->init();
};