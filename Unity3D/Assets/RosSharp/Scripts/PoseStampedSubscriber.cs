/*
© Federal Univerity of Minas Gerais (Brazil), 2017
Author: Lucas Coelho Figueiredo (me@lucascoelho.net)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

<http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/


using UnityEngine;


namespace RosSharp.RosBridgeClient
{

    [RequireComponent(typeof(RosConnector))]
    public class PoseStampedSubscriber : MonoBehaviour
    {
        public GameObject UrdfModel;
        private PoseStampedTransformManager poseStampedTransformManager;
        private RosSocket rosSocket;
        public string topic = "";
        public int UpdateTime = 1;

        public void Start()
        {
            rosSocket = transform.GetComponent<RosConnector>().RosSocket;
            rosSocket.Subscribe(topic, "geometry_msgs/PoseStamped", updatePoseStamped, UpdateTime);

            poseStampedTransformManager = UrdfModel.GetComponent<PoseStampedTransformManager>();
        }

        private void updatePoseStamped(Message message)
        {
            GeometryPoseStamped geometryPoseStamped = (GeometryPoseStamped)message;
            poseStampedTransformManager.updateTransform(getPosition(geometryPoseStamped), getRotation(geometryPoseStamped));
        }

        private static Vector3 getPosition(GeometryPoseStamped geometryPoseStamped)
        {
            return new Vector3(
                -geometryPoseStamped.pose.position.y,
                geometryPoseStamped.pose.position.z,
                geometryPoseStamped.pose.position.x);
        }

        private static Quaternion getRotation(GeometryPoseStamped geometryPoseStamped)
        {
            return new Quaternion(
                geometryPoseStamped.pose.orientation.x,
                -geometryPoseStamped.pose.orientation.z,
                geometryPoseStamped.pose.orientation.y,
                geometryPoseStamped.pose.orientation.w);
        }
        /*
        private static Vector3 getLinearTwist(NavigationPoseStamped navigationPoseStamped)
        {
            return new Vector3(
                -navigationPoseStamped.twist.twist.linear.y,
                navigationPoseStamped.twist.twist.linear.z,
                navigationPoseStamped.twist.twist.linear.x);
        }
        private static Vector3 getAngularTwist(NavigationPoseStamped navigationPoseStamped)
        {
            return new Vector3(
                -navigationPoseStamped.twist.twist.angular.y,
                navigationPoseStamped.twist.twist.angular.z,
                navigationPoseStamped.twist.twist.angular.x);
        }*/
    }
}