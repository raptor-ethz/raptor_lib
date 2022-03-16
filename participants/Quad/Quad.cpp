#include "Quad.h"

Quad::Quad(const std::string &raptor_participant_id,
           std::unique_ptr<DefaultParticipant> &dp,
           const std::string &sub_topic_name, const std::string &pub_topic_name)
{
  id_ = raptor_participant_id;

  /* Initialize subscribers */
  mocap_sub_ = new DDSSubscriber(idl_msg::MocapPubSubType(), &pose_,
                                 sub_topic_name, dp->participant());

  px4_info_sub_ = new DDSSubscriber(idl_msg::HeaderPubSubType(), &px4_info_,
                                 "px4_status_msgs", dp->participant());

  /* Initialize publishers */
  position_pub_ = new DDSPublisher(idl_msg::QuadPositionCmdPubSubType(),
                                   pub_topic_name, dp->participant());

  px4_action_pub_ = new DDSPublisher(idl_msg::HeaderPubSubType(),
                                     "px4_commands", dp->participant());
};

Quad::~Quad()
{
  delete mocap_sub_;
  delete position_pub_;
  delete px4_action_pub_;
  delete px4_info_sub_;
}