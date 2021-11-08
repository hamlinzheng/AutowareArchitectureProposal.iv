// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "aap_auto_msgs_converter/aap_auto_perception_msgs_converter.hpp"

namespace aap_auto_msgs_converter
{
AAPAutoPerceptionMsgsConverter::AAPAutoPerceptionMsgsConverter(
  const rclcpp::NodeOptions & node_options)
: Node("aap_auto_perception_msgs_converter_node", node_options)
{
  dynamic_object_array_sub_ = create_subscription<autoware_perception_msgs::msg::DynamicObjectArray>(
    "~/input/dynamic_object_array", rclcpp::QoS{1},
    std::bind(&AAPAutoPerceptionMsgsConverter::onDynamicObjectArray, this, std::placeholders::_1));
  detected_objects_pub_ =
    create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>("~/output/detected_objects", rclcpp::QoS{1});
}

void AAPAutoPerceptionMsgsConverter::onDynamicObjectArray(
  const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr input_msg)
{
  publishDetectedObjects(input_msg);
}

void AAPAutoPerceptionMsgsConverter::publishDetectedObjects(
  const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr input_msg)
{
  using autoware_auto_perception_msgs::msg::DetectedObjects;
  using autoware_auto_perception_msgs::msg::DetectedObject;
  DetectedObjects detected_obects;

  for(const auto & dynamic_obj : input_msg->objects) {
    DetectedObject detected_obj;
    // Semantic -> ObjectClassification
    autoware_auto_perception_msgs::msg::ObjectClassification object_cls;
    object_cls.label = convertLabel(dynamic_obj.semantic.type);
    object_cls.probability = dynamic_obj.semantic.confidence;
    detected_obj.classification.push_back(object_cls);

    // State -> DetectedObjectKinematics
    detected_obj.kinematics.pose_with_covariance = dynamic_obj.state.pose_covariance;
    detected_obj.kinematics.twist_with_covariance = dynamic_obj.state.twist_covariance;

    // Shape -> Shape
    detected_obj.shape.type = dynamic_obj.shape.type;
    detected_obj.shape.footprint = dynamic_obj.shape.footprint;
    detected_obj.shape.dimensions = dynamic_obj.shape.dimensions;

    detected_obects.objects.push_back(detected_obj);
  }

  detected_obects.header = input_msg->header;
  detected_objects_pub_->publish(detected_obects);
}

}  // namespace aap_auto_msgs_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(aap_auto_msgs_converter::AAPAutoPerceptionMsgsConverter)
