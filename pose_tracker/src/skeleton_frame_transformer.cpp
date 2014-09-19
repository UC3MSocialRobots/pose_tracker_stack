#include <set>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
//#include <skeleton_msgs/Skeleton.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

#include <pi_tracker/Skeleton.h> // For skeleton.msg messages

namespace{ // to prevent naming pollution
   static const std::string DEFAULT_NODE_NAME =
                                    std::string ("skeleton_frame_transformer");
   static const std::string DEFAULT_TOPIC_NAME = std::string("/skeleton_rel");
   static const int DEFAULT_RATE = 20; // rate in Hz

}

class SkeletonFrameTransformer {
public:
   /**
    * Constructor
   **/
   SkeletonFrameTransformer() : tf_listener_() , br() {

      n_ = ros::NodeHandle("~"); // private node handle --> to pass it params
//      n_ = ros::NodeHandle("/skeleton_transformer"); // private node handle --> to pass it params

      int publish_rate;
      n_.param("publish_rate",publish_rate,DEFAULT_RATE);
      ROS_INFO("Publish rate: %d", publish_rate);

      n_.param("target_frame", target_frame_, std::string("torso"));
      ROS_INFO("Using target frame: %s", target_frame_.c_str());

      skeleton_sub_.subscribe(n_, "/skeleton", 10);
      tf_filter_ = new tf::MessageFilter<pi_tracker::Skeleton>
                   (skeleton_sub_,tf_listener_,
                    target_frame_, 10);
      tf_filter_->registerCallback(
            boost::bind(
                  &SkeletonFrameTransformer::skeletonCallback, this, _1)
            );
      skeleton_pub_ = n_.advertise
                      <pi_tracker::Skeleton> (DEFAULT_TOPIC_NAME,
                                              DEFAULT_RATE);

      ROS_INFO("Node %s created",ros::this_node::getName().c_str());
   }
   

////////////////////////////////////////////////////////////////////////////////

  /**
    * Transforms a skeleton message to make the all joints relative to the
    * target_frame of the class
    * The torso is relative to the frame_id of the skeleton's header.
    * \param skeleton_old the skeleton that is going to be transformed
    * \param skeleton_new where the transforms are going to be stored.
   **/
   void transform_skeleton(const pi_tracker::Skeleton & skeleton_old,
                           pi_tracker::Skeleton & skeleton_new){
      ROS_DEBUG("Starting to transform the skeleton message frames.");

      geometry_msgs::Vector3Stamped position_old, position_new ;
      geometry_msgs::QuaternionStamped orientation_old, orientation_new;

      /// Copying old_skeleton's fields that aren't needed to be transformed
      skeleton_new.user_id = skeleton_old.user_id;
      skeleton_new.name = skeleton_old.name;
      skeleton_new.confidence = skeleton_old.confidence;

      /// need headers from skeleton to make the transformations later
      position_old.header = skeleton_old.header;
      orientation_old.header = skeleton_old.header;

      unsigned int i;
      for (i = 0; i < skeleton_old.position.size(); i++)
      {
          ROS_DEBUG_COND(
                      (skeleton_old.position.size() < i) ||
                      (skeleton_old.orientation.size() < i) ||
                      (skeleton_old.name.size() < i),
                                "Trying to access to an element that "
                                "doesn't exist!");

          ROS_DEBUG ("Accessing to joint %d (%s)\n",
                     i, skeleton_old.name.at(i).c_str());

          position_old.vector = skeleton_old.position.at(i);
          orientation_old.quaternion = skeleton_old.orientation.at(i);

          // Performing transformations
          //(only if the joint name is different to the target frame)
          if (skeleton_old.name.at(i).compare(target_frame_) != 0)
          {
              ROS_DEBUG("Transforming from frame %s to frame %s",
                        position_old.header.frame_id.c_str(),
                        target_frame_.c_str());

             // Checking if the quaternion is well formed (normalized)
             // when receiving all parameters to 0.
             // This is because NITE api seems to send this kind of
             // quaternions when no data is received by the joint.
             if ( orientation_old.quaternion.x == 0 &&
                  orientation_old.quaternion.y == 0 &&
                  orientation_old.quaternion.z == 0 &&
                  orientation_old.quaternion.w != 1){
                ROS_WARN("Warning malformed quaternion received in joint %s. "
                         "w component modified to 1 to normalise it.",
                         skeleton_old.name.at(i).c_str());
                orientation_old.quaternion.w = 1;
             }

             tf_listener_.transformVector(target_frame_,
                                          position_old,
                                          position_new);
             tf_listener_.transformQuaternion(target_frame_,
                                              orientation_old,
                                              orientation_new);

             publish_joint_tf(position_new.vector, orientation_new.quaternion,
                              skeleton_old.name.at(i) + "_rel");
          } else {
              ROS_DEBUG("%s joint remains measured from frame %s",
                                         target_frame_.c_str(),
                                         skeleton_old.header.frame_id.c_str());
              position_new = position_old;
              orientation_new = orientation_old;
          }


         // Assigning the positions and orientations
         skeleton_new.position.push_back(position_new.vector);
         skeleton_new.orientation.push_back(orientation_new.quaternion);
         ROS_DEBUG("Skeleton_new message position and orientantion "
                  "elements added");
         ROS_DEBUG("skeleton_new.position size: %d",
                  skeleton_new.position.size());
      }

      // the header of skeleton_new is the same as skeleton_old.
      // Note the frame id_is also the same because the frame_id of the
      // torso is also the same.
      skeleton_new.header = skeleton_old.header;


   }

////////////////////////////////////////////////////////////////////////////////
   /**
     * Publishes the position and orientation of a joint to the tf tree
     * The joint is measured from SkeletonFrameTransformer::target_frame_
     * \param position of the transform, measured from target_frame_
     * \param orientation of the transform, measured from target_frame_
     * \param child_frame_id The name of the transform
     */
   void publish_joint_tf(const geometry_msgs::Vector3& position,
                         const geometry_msgs::Quaternion& orientation,
                         const std::string& child_frame_id){
      tf::Transform transform;
      tf::Vector3 tf_position;
      tf::Quaternion tf_orientation;

      // Need to convert from geometry_msgs to tf the position and orientation
      tf::vector3MsgToTF(position,tf_position);
      tf::quaternionMsgToTF(orientation,tf_orientation);

      transform.setOrigin(tf_position);
      transform.setRotation(tf_orientation);

      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                            target_frame_, child_frame_id));
   }

////////////////////////////////////////////////////////////////////////////////

   /**
    * Callback to register with tf::MessageFilter
    * to be called when transforms are available
    **/
   void skeletonCallback(
         const boost::shared_ptr<const pi_tracker::Skeleton>& skeleton_ptr)
   {
      ROS_DEBUG("Callback called");
      pi_tracker::Skeleton skeleton_out;
      try
      {
         transform_skeleton(*skeleton_ptr, skeleton_out);

         skeleton_pub_.publish(skeleton_out);
         ROS_DEBUG("Skeleton message published to %s",
                   DEFAULT_TOPIC_NAME.c_str());
      }
      catch (tf::TransformException &ex)
      {
         printf ("Failure %s\n", ex.what()); //Print exception which was caught
      }
   }

private:
   ros::Publisher skeleton_pub_;
   std::string fixed_frame;
   std::string target_frame_;
   tf::TransformListener tf_listener_;
   tf::TransformBroadcaster br;

   /// Filters to subscribe to the (stamped) skeleton messages
   message_filters::Subscriber<pi_tracker::Skeleton> skeleton_sub_;
   tf::MessageFilter<pi_tracker::Skeleton> * tf_filter_;

   ros::NodeHandle n_;
};

int main(int argc, char** argv){

   ros::init(argc, argv, DEFAULT_NODE_NAME);
   SkeletonFrameTransformer transformer;

   ros::NodeHandle node("~");

   ros::spin(); // Run until interupted



//   ros::Rate rate(30.0);
//   while (node.ok()){

////      transformer.publish_skeleton();

//      rate.sleep();
//   }
//   return 0;
};
