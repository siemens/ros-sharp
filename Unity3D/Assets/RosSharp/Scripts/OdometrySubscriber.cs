/*
© Siemens AG, 2017
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

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
    public class OdometrySubscriber : MonoBehaviour
    {
        public GameObject UrdfModel;
        private OdometryTransformManager odometryTransformManager;
        private RosSocket rosSocket;
        public int UpdateTime = 1;

        public void Start()
        {
            rosSocket = transform.GetComponent<RosConnector>().RosSocket;
            rosSocket.Subscribe("/odom", "nav_msgs/Odometry", updateOdometry, UpdateTime);

            odometryTransformManager = UrdfModel.GetComponent<OdometryTransformManager>();
        }

        private void updateOdometry(Message message)
        {
            NavigationOdometry navigationOdometry = (NavigationOdometry)message;
            odometryTransformManager.updateTransform(getPosition(navigationOdometry), getRotation(navigationOdometry));
        }

        private static Vector3 getPosition(NavigationOdometry navigationOdometry)
        {
            return new Vector3(
                -navigationOdometry.pose.pose.position.y,
                navigationOdometry.pose.pose.position.z,
                navigationOdometry.pose.pose.position.x);
        }

        private static Quaternion getRotation(NavigationOdometry navigationOdometry)
        {
            return new Quaternion(
                navigationOdometry.pose.pose.orientation.x,
                -navigationOdometry.pose.pose.orientation.z,
                navigationOdometry.pose.pose.orientation.y,
                navigationOdometry.pose.pose.orientation.w);
        }
        /*
        private static Vector3 getLinearTwist(NavigationOdometry navigationOdometry)
        {
            return new Vector3(
                -navigationOdometry.twist.twist.linear.y,
                navigationOdometry.twist.twist.linear.z,
                navigationOdometry.twist.twist.linear.x);
        }
        private static Vector3 getAngularTwist(NavigationOdometry navigationOdometry)
        {
            return new Vector3(
                -navigationOdometry.twist.twist.angular.y,
                navigationOdometry.twist.twist.angular.z,
                navigationOdometry.twist.twist.angular.x);
        }*/
    }
}