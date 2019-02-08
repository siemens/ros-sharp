/*
© Siemens AG, 2019
Author: Berkay Alp Cakal (berkay_alp.cakal.ct@siemens.com)

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

namespace RosBridgeClient.Messages
{
    public enum MessageType
    {
        Header, @int, Int32MultiArray, MultiArrayDimension, MultiArrayLayout, @float, String, Time,
        Accel, Point, PointStamped, Pose, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance, Vector3,
        CompressedImage, Image, JointState, Joy, LaserScan, PointCloud2, PointField,
        Odometry, MapMetaData, OccupancyGrid,
        GoalID, GoalStatus, GoalStatusArray
    };

    [System.Serializable]
    public class MessageElement
    {
        public MessageType messageType;
        public string messageName;
        public bool isArray;

        private string messageTypeString;
        private string declerationString;
        private string definitionString;
        private bool isCustom = false;

        public MessageElement()
        { }

        public MessageElement(string msgType, string msgName, bool isArray)
        {
            isCustom = true;
            messageTypeString = msgType;
            messageName = msgName;

        }

        public string getDeclerationString()
        {
            if (!isCustom)
                messageTypeString = messageType.ToString();
            if (!isArray)
                declerationString = "public " + messageTypeString + " " + messageName + ";";
            else
                declerationString = "public " + messageTypeString + "[] " + messageName + ";";
            return declerationString;
        }

        public string getDefinitionString()
        {
            if (!isCustom)
                messageTypeString = messageType.ToString();
            if (!isArray)
                definitionString = messageName + " = new " + messageTypeString + "();";
            else
                definitionString = messageName + " = new " + messageTypeString + "[0];";
            return definitionString;
        }

    }

}
