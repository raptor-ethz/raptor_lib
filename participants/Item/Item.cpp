#include "Item.h"

Item::Item(const std::string &raptor_participant_id,
           std::unique_ptr<DefaultParticipant> &dp,
           const std::string &sub_topic_name) {
  id = raptor_participant_id;

  mocap_sub = new DDSSubscriber(idl_msg::MocapPubSubType(), &pose_,
                                sub_topic_name, dp->participant());

  mocap_sub->init();
};

Item::~Item() { delete mocap_sub; }