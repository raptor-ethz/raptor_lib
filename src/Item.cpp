#include "Item.h"

Item::Item( std::unique_ptr<DefaultParticipant> dp, 
            std::string sub_topic_name) {

    mocap_sub = new DDSSubscriber(idl_msg::MocapPubSubType(), &pose_,
                                     "mocap_srl_quad", dp->participant());

    mocap_sub->init();
    
};

Item::~Item () {
    delete mocap_sub;
}