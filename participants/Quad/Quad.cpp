#include "Quad.h"

Quad::Quad(const std::string &raptor_participant_id, std::string *const log,
           std::unique_ptr<DefaultParticipant> &dp,
           const std::string &sub_topic_name,
           const std::string &pub_topic_name)
{
  id_ = raptor_participant_id;
  log_ = log;

  // initialize subscribers
  mocap_sub_ = new DDSSubscriber(idl_msg::Mocap_msgPubSubType(), &pose_,
                                 sub_topic_name, dp->participant());
  px4_feedback_sub_ =
      new DDSSubscriber(idl_msg::QuadFeedback_msgPubSubType(), &px4_feedback_,
                        "px4_status_msgs", dp->participant());
  ui_sub_ = new DDSSubscriber(idl_msg::UserCmd_msgPubSubType(), &ui_cmd_,
                              "ui_commands", dp->participant());

  // initialize publishers
  position_pub_ = new DDSPublisher(idl_msg::QuadPosCmd_msgPubSubType(),
                                   pub_topic_name, dp->participant());
  px4_action_pub_ = new DDSPublisher(idl_msg::QuadAction_msgPubSubType(),
                                     "px4_commands", dp->participant());
};

Quad::Quad(const std::string &raptor_participant_id, std::string *const log,
           std::unique_ptr<DefaultParticipant> &dp,
           const std::string &sub_topic_name, const std::string &pub_topic_name,
           Gripper *const gripper, Item *const stand)
    : Quad(raptor_participant_id, log, dp, sub_topic_name, pub_topic_name)
{
  gripper_ = gripper;
  stand_ = stand;
};

Quad::Quad(const std::string &raptor_participant_id, std::string *const log,
           std::unique_ptr<DefaultParticipant> &dp,
           const std::string &sub_topic_name, const std::string &pub_topic_name,
           Item *const stand)
    : Quad(raptor_participant_id, log, dp, sub_topic_name, pub_topic_name)
{
  stand_ = stand;
};

Quad::~Quad()
{
  delete mocap_sub_;
  delete position_pub_;
  delete px4_action_pub_;
  delete px4_feedback_sub_;
  delete ui_sub_;
}