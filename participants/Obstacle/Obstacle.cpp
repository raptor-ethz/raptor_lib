#include "Obstacle.h"

Obstacle::Obstacle(const std::string &raptor_participant_id,
                   std::unique_ptr<DefaultParticipant> &dp,
                   const std::string &sub_topic_name)
{
    // id_ = raptor_participant_id;

    mocap_marker_sub_ = new DDSSubscriber(idl_msg::MocapMarker_msgPubSubType(), &markers_,
                                          sub_topic_name, dp->participant());
};

Obstacle::~Obstacle() { delete mocap_marker_sub_; }