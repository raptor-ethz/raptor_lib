#include "Quad.h"

Quad::Quad(const std::string &raptor_participant_id,
           std::unique_ptr<DefaultParticipant> &dp,
           const std::string &sub_topic_name,
           const std::string &pub_topic_name) {
  id = raptor_participant_id;

  mocap_sub = new DDSSubscriber(idl_msg::MocapPubSubType(), &pose_,
                                sub_topic_name, dp->participant());

  mocap_sub->init();

  position_pub = new DDSPublisher(idl_msg::QuadPositionCmdPubSubType(),
                                  pub_topic_name, dp->participant());

  position_pub->init();
};

Quad::~Quad() {
  delete mocap_sub;
  delete position_pub;
}