#pragma once

#include "MocapMarker_msg.h"
#include "MocapMarker_msgPubSubTypes.h"

#include "domain_participant.h"
#include "subscriber.h"

class Obstacle
{
public:
    DDSSubscriber<idl_msg::MocapMarker_msgPubSubType, cpp_msg::MocapMarker_msg> *mocap_marker_sub_;

    Obstacle(const std::string &raptor_participant_id,
             std::unique_ptr<DefaultParticipant> &dp,
             const std::string &sub_topic_name);
    ~Obstacle();

    const cpp_msg::MocapMarker_msg &getMarkers() { return markers_; }

private:
    cpp_msg::MocapMarker_msg markers_{};
};