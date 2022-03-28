#include "Item.h"

Item::Item(const std::string &raptor_participant_id,
           std::unique_ptr<DefaultParticipant> &dp,
           const std::string &sub_topic_name) {
  id_ = raptor_participant_id;

  mocap_sub_ = new DDSSubscriber(idl_msg::Mocap_msgPubSubType(), &pose_,
                                sub_topic_name, dp->participant());

};

Item::~Item() { delete mocap_sub_; }