#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Image.h>

#include "skeltrack-0.1/skeltrack-skeleton.h"
#include "skeltrack-0.1/skeltrack-joint.h"

#define DEBUG 0

//#define ROS_DEPTH_PATH "/camera/depth/image_raw"
//#define ROS_DEPTH_PATH "/camera/depth_registered/image_raw"
#define ROS_DEPTH_PATH "/naoqi_driver_node/camera/depth/image_raw"
//#define FRAME_ID "depth_test"
#define FRAME_ID "CameraDepth_optical_frame"
//#define FRAME_ID "CameraDepth_frame"
//#define FRAME_ID "CameraDepth_depth_optical_frame"
//#define FRAME_ID "camera_rgb_optical_frame"
#define ROS_BUFFER_FRAMES 10

using std::string;


static SkeltrackSkeleton *skeleton = NULL;

static SkeltrackJointList list = NULL;

/**
  * Cut frame size by this factor to reduce processing power
  * Be wary, no subsampling done, so you lose detail.
  * Values: 1 - 16 (you could go higher, but you wouldn't have a lot of image)
  */
static char ENABLE_REDUCTION = 1;
static unsigned char DIMENSION_REDUCTION = 2;

/**
  * Enables smoothing together multiple frames for less jitter
  * Values: 0.0 - 1.0
  */
static char ENABLE_SMOOTHING = 1;
static float SMOOTHING_FACTOR = .0;


/** 
  * Filter out outlying depths by tweaking these thresholds.
  * Values: 0 - INT_MAX
  */
static char ENABLE_THRESHOLD = 1;
static unsigned int THRESHOLD_BEGIN = 500;
static unsigned int THRESHOLD_END   = 1700;

typedef struct
{
  uint16_t *reduced_buffer;
  unsigned int width;
  unsigned int height;
  unsigned int reduced_width;
  unsigned int reduced_height;
} BufferInfo;

void publishTransforms(const string frame_id);
void publishTransform(const SkeltrackJoint& joint,
                      const string& frame_id,
                      const string& joint_name);
static BufferInfo *
        process_buffer (const std::vector<uint8_t> buffer,
                        unsigned int width,
                        unsigned int height,

                        unsigned char dimension_factor,
                        unsigned int threshold_begin,
                        unsigned int threshold_end);
static void on_track_joints (GObject      *obj,
                             GAsyncResult *res,
                             gpointer      user_data);

/*
 *
 * End headers.
 *
 */


void publishTransforms(const string frame_id){
    SkeltrackJoint *head, *left_shoulder, *right_shoulder, *left_elbow, *right_elbow, *left_hand, *right_hand;
    
    ROS_INFO("Pushing out joint data.");

    head           = skeltrack_joint_list_get_joint(list, SKELTRACK_JOINT_ID_HEAD);
    left_shoulder  = skeltrack_joint_list_get_joint(list, SKELTRACK_JOINT_ID_LEFT_SHOULDER);
    right_shoulder = skeltrack_joint_list_get_joint(list, SKELTRACK_JOINT_ID_RIGHT_SHOULDER);
    left_elbow     = skeltrack_joint_list_get_joint(list, SKELTRACK_JOINT_ID_LEFT_ELBOW);
    right_elbow    = skeltrack_joint_list_get_joint(list, SKELTRACK_JOINT_ID_RIGHT_ELBOW);
    left_hand      = skeltrack_joint_list_get_joint(list, SKELTRACK_JOINT_ID_LEFT_HAND);
    right_hand     = skeltrack_joint_list_get_joint(list, SKELTRACK_JOINT_ID_RIGHT_HAND);


    publishTransform((const SkeltrackJoint &) head,           frame_id, "head");
    publishTransform((const SkeltrackJoint &) left_shoulder,  frame_id, "left_shoulder");
    publishTransform((const SkeltrackJoint &) right_shoulder, frame_id, "right_shoulder");
    publishTransform((const SkeltrackJoint &) left_elbow,     frame_id, "left_elbow");
    publishTransform((const SkeltrackJoint &) right_elbow,    frame_id, "right_elbow");
    publishTransform((const SkeltrackJoint &) left_hand,      frame_id, "left_hand");
    publishTransform((const SkeltrackJoint &) right_hand,     frame_id, "right_hand");

    
}


void publishTransform(const SkeltrackJoint& joint, const string& frame_id, 
                      const string& joint_name) {
                          
    if (&joint == NULL) return;
    
    static tf::TransformBroadcaster br;

    uint16_t x = (uint16_t) joint.x;
    uint16_t y = (uint16_t)  joint.y;
    uint16_t z = (uint16_t)  joint.z;

    char child_frame_no[128];
    snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", frame_id.c_str(), 1);
    // We can only track one (hence id 1), but maybe multiple instances in the future?
    
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));

    // TODO: Parameterize origin location
    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion frame_rotation;
//    frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
    change_frame.setRotation(frame_rotation);

    transform = change_frame * transform;

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));
}

/**
  * Scale, apply tresholds, and convert to 16-bit mm from 32FC1 
  */

static BufferInfo *
process_buffer (const std::vector<uint8_t> buffer,
                unsigned int width,
                unsigned int height,
				
                unsigned char dimension_factor,
                unsigned int threshold_begin,
                unsigned int threshold_end)
{
    BufferInfo *buffer_info;
    unsigned int i, j, reduced_width, reduced_height;
    uint16_t *reduced_buffer;

//    if (buffer == NULL) return NULL;
    
    reduced_width = ENABLE_REDUCTION ? (width - width % dimension_factor) / dimension_factor : width;
    reduced_height = ENABLE_REDUCTION ? (height - height % dimension_factor) / dimension_factor : height;

    reduced_buffer = (uint16_t*) malloc (reduced_width * reduced_height * sizeof(uint16_t));
    if (reduced_buffer == NULL) return NULL;
    
    unsigned int index;
    uint16_t value;
    for (i = 0; i < reduced_width; i++) {
        for (j = 0; j < reduced_height; j++) {
            index = (j * width * dimension_factor + i * dimension_factor) * 2; //2 int8 in a int16
            value = static_cast<uint16_t>(buffer[index+1]*256 + buffer[index]); 

            //if (i == 0 && j == 0) ROS_INFO("%f", (float) value);
            if (ENABLE_THRESHOLD && (value < threshold_begin || value > threshold_end)) {
              reduced_buffer[j * reduced_width + i] = 0;
            } else {
              reduced_buffer[j * reduced_width + i] = value;
			}
        }
    }

    buffer_info = (BufferInfo*) malloc(sizeof(BufferInfo));
    if (buffer_info == NULL) return NULL;
    buffer_info->reduced_buffer = reduced_buffer;
    buffer_info->reduced_width = reduced_width;
    buffer_info->reduced_height = reduced_height;
    buffer_info->width = width;
    buffer_info->height = height;

  return buffer_info;
}

/** onNewDepth Receives and routes an depth image to be processed
  *	dimage - Image struct containing mono uint16 data
  */
static void onNewDepth(const sensor_msgs::Image& dimage){
    GError *error = NULL;
    BufferInfo* buffer_info;
    //TODO: parameterize
    unsigned char dimension_factor = DIMENSION_REDUCTION;
    unsigned int thold_begin = THRESHOLD_BEGIN;
    unsigned int thold_end = THRESHOLD_END;


    if (&dimage==NULL) {
        ROS_WARN_THROTTLE(1,"NULL image received.");
        return;
    }
    ROS_INFO_THROTTLE(1, "Received frame with %s encoding.", dimage.encoding.c_str());
    

    buffer_info = process_buffer(dimage.data,
                                 dimage.width,
                                 dimage.height,
                                 dimension_factor,
                                 thold_begin,
                                 thold_end);
    //ROS_INFO_STREAM(buffer_info);
    // skeleton is global
    list = skeltrack_skeleton_track_joints_sync (skeleton,
                                     buffer_info->reduced_buffer, //(guint16*)
                                     (guint16) buffer_info->reduced_width,
                                     (guint16) buffer_info->reduced_height,
                                     NULL,
                                     &error);


    if (error != NULL) {
        ROS_WARN("%s", error->message);
    } else if (list != NULL) {
       publishTransforms(FRAME_ID);
    }
    else {
        ROS_INFO_STREAM(" --- skeleton not found");
    }
    // Free memory, even if things go wrong.
    free(buffer_info->reduced_buffer);
    free(buffer_info);
    skeltrack_joint_list_free(list);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "skeletor_processor");
	ros::NodeHandle n;
	
	//n.param<std::string>("default_param", default_param, "default_value");
    
	// any metadata subscriptions above here.
	ros::spinOnce();
	skeleton = skeltrack_skeleton_new();
	ROS_INFO("Skeltrack service started");
	ros::Subscriber sub_depth = n.subscribe(ROS_DEPTH_PATH, 10, onNewDepth);
	ros::spin();
    
    
}
