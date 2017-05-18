/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Jon Binney, Ioan Sucan */

#include <cmath>
#include <moveit/pointcloud_octomap_updater/pointcloud_octomap_updater.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <XmlRpcException.h>

#include <memory>

namespace occupancy_map_monitor
{
PointCloudOctomapUpdater::PointCloudOctomapUpdater()
  : OccupancyMapUpdater("PointCloudUpdater")
  , private_nh_("~")
  , scale_(1.0)
  , padding_(0.0)
  , resolution_(0.04)
  , max_range_(std::numeric_limits<double>::infinity())
  , point_subsample_(1)
  , point_cloud_subscriber_(NULL)
  , point_cloud_filter_(NULL)
{
}

PointCloudOctomapUpdater::~PointCloudOctomapUpdater()
{
  stopHelper();
}

bool PointCloudOctomapUpdater::setParams(XmlRpc::XmlRpcValue& params)
{
  try
  {
    if (!params.hasMember("point_cloud_topic"))
      return false;
    point_cloud_topic_ = static_cast<const std::string&>(params["point_cloud_topic"]);

    readXmlParam(params, "max_range", &max_range_);
    readXmlParam(params, "padding_offset", &padding_);
    readXmlParam(params, "padding_scale", &scale_);
    readXmlParam(params, "point_subsample", &point_subsample_);
    readXmlParam(params, "resolution", &resolution_);
    if (params.hasMember("filtered_cloud_topic"))
      filtered_cloud_topic_ = static_cast<const std::string&>(params["filtered_cloud_topic"]);
  }
  catch (XmlRpc::XmlRpcException& ex)
  {
    ROS_ERROR("XmlRpc Exception: %s", ex.getMessage().c_str());
    return false;
  }

  return true;
}

bool PointCloudOctomapUpdater::initialize()
{
  tf_ = monitor_->getTFClient();
  shape_mask_.reset(new point_containment_filter::ShapeMask());
  shape_mask_->setTransformCallback(boost::bind(&PointCloudOctomapUpdater::getShapeTransform, this, _1, _2));

  collision_object_subscriper_ = root_nh_.subscribe<moveit_msgs::CollisionObject>("collision_object", 1, &PointCloudOctomapUpdater::collisionObjectMsgCallback, this);
  attached_collision_object_subscriper_ = root_nh_.subscribe<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 1, &PointCloudOctomapUpdater::attachedCollisionObjectMsgCallback, this);

  if (!filtered_cloud_topic_.empty())
    filtered_cloud_publisher_ = private_nh_.advertise<sensor_msgs::PointCloud2>(filtered_cloud_topic_, 10, false);
  return true;
}

void PointCloudOctomapUpdater::attachedCollisionObjectMsgCallback(const moveit_msgs::AttachedCollisionObject::ConstPtr &obj_msg){
    std::vector<moveit_msgs::CollisionObject> collision_objects_backup = collision_objects_;
    collision_objects_.clear();
    for(auto object : collision_objects_backup){
        if(object.id != obj_msg->object.id){
            collision_objects_.push_back(object);
        }
    }
}

void PointCloudOctomapUpdater::collisionObjectMsgCallback(const moveit_msgs::CollisionObject::ConstPtr &obj_msg){
    if(obj_msg->operation == moveit_msgs::CollisionObject::ADD
            || obj_msg->operation == moveit_msgs::CollisionObject::MOVE
            || obj_msg->operation == moveit_msgs::CollisionObject::APPEND)
    {
        bool is_already_registered = false;
        for(auto& object : collision_objects_){
            if(object.id == obj_msg->id && obj_msg->primitives.size() != 0){
                is_already_registered = true;
                object = *obj_msg;
            }
        }
        if(!is_already_registered && obj_msg->primitives.size() != 0){
            collision_objects_.push_back(*obj_msg);
        }
    }
    else if(obj_msg->operation == moveit_msgs::CollisionObject::REMOVE){
        std::cout << "REMOVE OBJECT!!!!!!!!!!!!!!" << std::endl;
        std::vector<moveit_msgs::CollisionObject> collision_objects_backup = collision_objects_;
        collision_objects_.clear();
        for(auto object : collision_objects_backup){
            if(object.id != obj_msg->id){
                collision_objects_.push_back(object);
            }
        }
    }
}


void PointCloudOctomapUpdater::start()
{
  if (point_cloud_subscriber_)
    return;
  /* subscribe to point cloud topic using tf filter*/
  point_cloud_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(root_nh_, point_cloud_topic_, 5);
  if (tf_ && !monitor_->getMapFrame().empty())
  {
    point_cloud_filter_ =
        new tf::MessageFilter<sensor_msgs::PointCloud2>(*point_cloud_subscriber_, *tf_, monitor_->getMapFrame(), 5);
    point_cloud_filter_->registerCallback(boost::bind(&PointCloudOctomapUpdater::cloudMsgCallback, this, _1));
    ROS_INFO("Listening to '%s' using message filter with target frame '%s'", point_cloud_topic_.c_str(),
             point_cloud_filter_->getTargetFramesString().c_str());
  }
  else
  {
    point_cloud_subscriber_->registerCallback(boost::bind(&PointCloudOctomapUpdater::cloudMsgCallback, this, _1));
    ROS_INFO("Listening to '%s'", point_cloud_topic_.c_str());
  }
}

void PointCloudOctomapUpdater::stopHelper()
{
  delete point_cloud_filter_;
  delete point_cloud_subscriber_;
}

void PointCloudOctomapUpdater::stop()
{
  stopHelper();
  point_cloud_filter_ = NULL;
  point_cloud_subscriber_ = NULL;
}

ShapeHandle PointCloudOctomapUpdater::excludeShape(const shapes::ShapeConstPtr& shape)
{
  ShapeHandle h = 0;
  if (shape_mask_)
    h = shape_mask_->addShape(shape, scale_, padding_);
  else
    ROS_ERROR("Shape filter not yet initialized!");
  return h;
}

void PointCloudOctomapUpdater::forgetShape(ShapeHandle handle)
{
  if (shape_mask_)
    shape_mask_->removeShape(handle);
}

bool PointCloudOctomapUpdater::getShapeTransform(ShapeHandle h, Eigen::Affine3d& transform) const
{
  ShapeTransformCache::const_iterator it = transform_cache_.find(h);
  if (it == transform_cache_.end())
  {
    ROS_ERROR("Internal error. Shape filter handle %u not found", h);
    return false;
  }
  transform = it->second;
  return true;
}

void PointCloudOctomapUpdater::updateMask(const sensor_msgs::PointCloud2& cloud, const Eigen::Vector3d& sensor_origin,
                                          std::vector<int>& mask)
{
}


void PointCloudOctomapUpdater::registerOccupancyKeySetForCollisionObjects(octomap::KeySet& cells){

    for(auto object : collision_objects_){
        for(std::size_t i = 0; i < object.primitives.size(); ++i){
            geometry_msgs::PoseStamped object_pose;
            object_pose.header.stamp = ros::Time::now();
            object_pose.header.frame_id = "base_footprint";
            object_pose.pose = object.primitive_poses[i];

            tf::Stamped<tf::Pose> object_global_pose_tf;
            tf::Stamped<tf::Pose>  object_pose_tf;
            tf::poseStampedMsgToTF(object_pose, object_pose_tf);

            try{
                tf_->transformPose("map", ros::Time(0), object_pose_tf, object_pose.header.frame_id, object_global_pose_tf);
            } catch (tf::TransformException& ex){
                ROS_ERROR("TRANSFORM EXCEPTION: %s", ex.what());
                return;
            }
            geometry_msgs::PoseStamped object_global_pose;
            tf::poseStampedTFToMsg(object_global_pose_tf, object_global_pose);

            Eigen::Vector3d center;
            center << object_global_pose.pose.position.x, object_global_pose.pose.position.y, object_global_pose.pose.position.z;
            shape_msgs::SolidPrimitive solid = object.primitives[i];
            if(solid.type == shape_msgs::SolidPrimitive::BOX){
                double dim_x = solid.dimensions[shape_msgs::SolidPrimitive::BOX_X];
                double dim_y = solid.dimensions[shape_msgs::SolidPrimitive::BOX_Y];
                double dim_z = solid.dimensions[shape_msgs::SolidPrimitive::BOX_Z];

                for(double x = -dim_x/2.0-resolution_; x <= dim_x/2.0+resolution_; x = x+resolution_){
                    for(double y = -dim_y/2.0-resolution_; y <= dim_y/2.0+resolution_; y = y+resolution_){
                        for(double z = -dim_z/2.0-resolution_; z <= dim_z/2.0+resolution_; z = z+resolution_){
                            cells.insert(tree_->coordToKey(x+center(0), y+center(1), z+center(2)));
                        }
                    }
                }
            }
            else if(solid.type == shape_msgs::SolidPrimitive::CYLINDER){
                double dim_x = solid.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]*2.0;
                double dim_z = solid.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT];

                for(double x = -dim_x/2.0-resolution_; x <= dim_x/2.0+resolution_; x = x+resolution_){
                    for(double y = -dim_x/2.0-resolution_; y <= dim_x/2.0+resolution_; y = y+resolution_){
                        for(double z = -dim_z/2.0-resolution_; z <= dim_z/2.0+resolution_; z = z+resolution_){
                            cells.insert(tree_->coordToKey(x+center(0), y+center(1), z+center(2)));
                        }
                    }
                }
            }
            else if(solid.type == shape_msgs::SolidPrimitive::SPHERE){
                double dim_x = solid.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]*2.0;
                for(double x = -dim_x/2.0-resolution_; x <= dim_x/2.0+resolution_; x = x+resolution_){
                    for(double y = -dim_x/2.0-resolution_; y <= dim_x/2.0+resolution_; y = y+resolution_){
                        for(double z = -dim_x/2.0-resolution_; z <= dim_x/2.0+resolution_; z = z+resolution_){
                            cells.insert(tree_->coordToKey(x+center(0), y+center(1), z+center(2)));
                        }   
                    }
                }
            }

        }

    }
}


void PointCloudOctomapUpdater::cloudMsgCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  ROS_DEBUG("Received a new point cloud message");
  ros::WallTime start = ros::WallTime::now();

  if (monitor_->getMapFrame().empty())
    monitor_->setMapFrame(cloud_msg->header.frame_id);

  /* get transform for cloud into map frame */
  tf::StampedTransform map_H_sensor;
  if (monitor_->getMapFrame() == cloud_msg->header.frame_id)
    map_H_sensor.setIdentity();
  else
  {
    if (tf_)
    {
      try
      {
        tf_->lookupTransform(monitor_->getMapFrame(), cloud_msg->header.frame_id, cloud_msg->header.stamp,
                             map_H_sensor);
      }
      catch (tf::TransformException& ex)
      {
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << "; quitting callback");
        return;
      }
    }
    else
      return;
  }

  /* compute sensor origin in map frame */
  const tf::Vector3& sensor_origin_tf = map_H_sensor.getOrigin();
  octomap::point3d sensor_origin(sensor_origin_tf.getX(), sensor_origin_tf.getY(), sensor_origin_tf.getZ());
  Eigen::Vector3d sensor_origin_eigen(sensor_origin_tf.getX(), sensor_origin_tf.getY(), sensor_origin_tf.getZ());

  if (!updateTransformCache(cloud_msg->header.frame_id, cloud_msg->header.stamp))
  {
    ROS_ERROR_THROTTLE(1, "Transform cache was not updated. Self-filtering may fail.");
    return;
  }

  /* mask out points on the robot */
  shape_mask_->maskContainment(*cloud_msg, sensor_origin_eigen, 0.0, max_range_, mask_);
  updateMask(*cloud_msg, sensor_origin_eigen, mask_);

  octomap::KeySet free_cells, occupied_cells, model_cells, clip_cells;
  std::unique_ptr<sensor_msgs::PointCloud2> filtered_cloud;

  // We only use these iterators if we are creating a filtered_cloud for
  // publishing. We cannot default construct these, so we use unique_ptr's
  // to defer construction
  std::unique_ptr<sensor_msgs::PointCloud2Iterator<float> > iter_filtered_x;
  std::unique_ptr<sensor_msgs::PointCloud2Iterator<float> > iter_filtered_y;
  std::unique_ptr<sensor_msgs::PointCloud2Iterator<float> > iter_filtered_z;

  if (!filtered_cloud_topic_.empty())
  {
    filtered_cloud.reset(new sensor_msgs::PointCloud2());
    filtered_cloud->header = cloud_msg->header;
    sensor_msgs::PointCloud2Modifier pcd_modifier(*filtered_cloud);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
    pcd_modifier.resize(cloud_msg->width * cloud_msg->height);

    // we have created a filtered_out, so we can create the iterators now
    iter_filtered_x.reset(new sensor_msgs::PointCloud2Iterator<float>(*filtered_cloud, "x"));
    iter_filtered_y.reset(new sensor_msgs::PointCloud2Iterator<float>(*filtered_cloud, "y"));
    iter_filtered_z.reset(new sensor_msgs::PointCloud2Iterator<float>(*filtered_cloud, "z"));
  }
  size_t filtered_cloud_size = 0;

  tree_->lockRead();

  try
  {
    /* do ray tracing to find which cells this point cloud indicates should be free, and which it indicates
     * should be occupied */
    registerOccupancyKeySetForCollisionObjects(model_cells);

    for (unsigned int row = 0; row < cloud_msg->height; row += point_subsample_)
    {
      unsigned int row_c = row * cloud_msg->width;
      sensor_msgs::PointCloud2ConstIterator<float> pt_iter(*cloud_msg, "x");
      // set iterator to point at start of the current row
      pt_iter += row_c;

      for (unsigned int col = 0; col < cloud_msg->width; col += point_subsample_, pt_iter += point_subsample_)
      {
        // if (mask_[row_c + col] == point_containment_filter::ShapeMask::CLIP)
        //  continue;

        /* check for NaN */
        if (!std::isnan(pt_iter[0]) && !std::isnan(pt_iter[1]) && !std::isnan(pt_iter[2]))
        {
          /* transform to map frame */
          tf::Vector3 point_tf = map_H_sensor * tf::Vector3(pt_iter[0], pt_iter[1], pt_iter[2]);

          /* occupied cell at ray endpoint if ray is shorter than max range and this point
             isn't on a part of the robot*/
          if (mask_[row_c + col] == point_containment_filter::ShapeMask::INSIDE)
            model_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
          else if (mask_[row_c + col] == point_containment_filter::ShapeMask::CLIP)
            clip_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
          else
          {
            occupied_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
            // build list of valid points if we want to publish them
            if (filtered_cloud)
            {
              **iter_filtered_x = pt_iter[0];
              **iter_filtered_y = pt_iter[1];
              **iter_filtered_z = pt_iter[2];
              ++filtered_cloud_size;
              ++*iter_filtered_x;
              ++*iter_filtered_y;
              ++*iter_filtered_z;
            }
          }
        }
      }
    }

    /* compute the free cells along each ray that ends at an occupied cell */
    for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
      if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(*it), key_ray_))
        free_cells.insert(key_ray_.begin(), key_ray_.end());

    /* compute the free cells along each ray that ends at a model cell */
    for (octomap::KeySet::iterator it = model_cells.begin(), end = model_cells.end(); it != end; ++it)
      if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(*it), key_ray_))
        free_cells.insert(key_ray_.begin(), key_ray_.end());

    /* compute the free cells along each ray that ends at a clipped cell */
    for (octomap::KeySet::iterator it = clip_cells.begin(), end = clip_cells.end(); it != end; ++it)
      if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(*it), key_ray_))
        free_cells.insert(key_ray_.begin(), key_ray_.end());
  }
  catch (...)
  {
    tree_->unlockRead();
    return;
  }

  tree_->unlockRead();

  /* cells that overlap with the model are not occupied */
  for (octomap::KeySet::iterator it = model_cells.begin(), end = model_cells.end(); it != end; ++it)
    occupied_cells.erase(*it);

  /* occupied cells are not free */
  for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
    free_cells.erase(*it);

  tree_->lockWrite();

  try
  {
    /* mark free cells only if not seen occupied in this cloud */
    for (octomap::KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it)
      tree_->updateNode(*it, false);

    /* now mark all occupied cells */
    for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
      tree_->updateNode(*it, true);

    // set the logodds to the minimum for the cells that are part of the model
    const float lg = tree_->getClampingThresMinLog() - tree_->getClampingThresMaxLog();
    for (octomap::KeySet::iterator it = model_cells.begin(), end = model_cells.end(); it != end; ++it)
      tree_->updateNode(*it, lg);
  }
  catch (...)
  {
    ROS_ERROR("Internal error while updating octree");
  }
  tree_->unlockWrite();
  ROS_DEBUG("Processed point cloud in %lf ms", (ros::WallTime::now() - start).toSec() * 1000.0);
  tree_->triggerUpdateCallback();

  if (filtered_cloud)
  {
    sensor_msgs::PointCloud2Modifier pcd_modifier(*filtered_cloud);
    pcd_modifier.resize(filtered_cloud_size);
    filtered_cloud_publisher_.publish(*filtered_cloud);
  }
}
}
