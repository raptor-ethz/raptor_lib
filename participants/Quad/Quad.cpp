#include "Quad.h"

Quad::Quad(const std::string &raptor_participant_id,
           std::unique_ptr<DefaultParticipant> &dp,
           const std::string &sub_topic_name,
           const std::string &pub_topic_name) {
  id = raptor_participant_id;

  mocap_sub_ = new DDSSubscriber(idl_msg::MocapPubSubType(), &pose_,
                                sub_topic_name, dp->participant());

  
  position_pub = new DDSPublisher(idl_msg::QuadPositionCmdPubSubType(),
                                  pub_topic_name, dp->participant());
  px4_cmd_pub = new DDSPublisher(idl_msg::HeaderPubSubType(),
                                  "px4_commands", dp->participant());

  px4_error_sub = new DDSSubscriber(idl_msg::HeaderPubSubType(), &px4_error_msg,
                                "px4_error_msgs", dp->participant());
};

Quad::~Quad() {
  delete mocap_sub_;
  delete position_pub;
}