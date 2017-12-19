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
    public class PosePublisher : MonoBehaviour
    {
        private RosSocket rosSocket;
        public string topic = "/pose";
        public int UpdateTime = 1;
        private int advertizer;
        public GameObject UnityGameObject;
        private PoseTransformManager poseTransformManager;
        public enum PublishingStates
        {
            OnChange,
            Continously,
            Never
        };
        public PublishingStates PublishingOption;

        private void Start()
        {
            rosSocket = transform.GetComponent<RosConnector>().RosSocket;
            advertizer = rosSocket.Advertize(topic, "geometry_msgs/Pose");
            if(UnityGameObject != null)
                poseTransformManager = UnityGameObject.GetComponent<PoseTransformManager>();
        }

        private void publishPose(GeometryPose message)
        {
            rosSocket.Publish(advertizer, message);
        }

        private void setPublisher(string topic)
        {
            rosSocket.Unadvertize(advertizer);
            advertizer = rosSocket.Advertize(topic, "geometry_msgs/Pose");
        }

        private void Update()
        {
            if (PublishingOption == PublishingStates.Never)
                return;
            if(poseTransformManager != null)
            {
                if (PublishingOption == PublishingStates.OnChange && !poseTransformManager.transform.hasChanged)
                    return;
                GeometryPose msg = gameObjectTransformToGeometryPose(poseTransformManager.transform);
                this.publishPose(msg);
            }
        }

        private static GeometryPose gameObjectTransformToGeometryPose(Transform transform)
        {
            GeometryPose msg = new GeometryPose();
            msg.position.x = transform.position.z;
            msg.position.y = -transform.position.x;
            msg.position.z = transform.position.y;

            msg.orientation.x = transform.rotation.x;
            msg.orientation.y = transform.rotation.z;
            msg.orientation.z = -transform.rotation.y;
            msg.orientation.w = transform.rotation.w;

            return msg;
        }
    }
}
