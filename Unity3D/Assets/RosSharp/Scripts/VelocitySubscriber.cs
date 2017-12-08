/*
© CentraleSupelec, 2017
Author: Dr. Jeremy Fix (jeremy.fix@centralesupelec.fr)

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
using System.Collections;

namespace RosSharp.RosBridgeClient
{

	[RequireComponent(typeof(RosConnector))]
	public class VelocitySubscriber : MonoBehaviour
	{
		public GameObject UrdfModel;

		public string topic = "/cmd_vel";

		private RosSocket rosSocket;
		private VelocityTransformManager velocityTransformManager;

		public int UpdateTime = 1;

		// Use this for initialization
		void Start ()
		{
			rosSocket = transform.GetComponent<RosConnector>().RosSocket;
			rosSocket.Subscribe(topic, "geometry_msgs/Twist", updateVelocity, UpdateTime);

			velocityTransformManager = UrdfModel.GetComponent<VelocityTransformManager>();		
		}

		private void updateVelocity(Message message)
		{
			GeometryTwist geometryTwist = (GeometryTwist)message;
			// In ROS conventions, the coordinate system is right-handed with 
			// x pointing forward
			// y pointing to the left
			// z pointing to the top

			// In Unity conventions, the coordinate system is left-handed with 
			// x pointing to the right
			// y pointing to the top
			// z pointing forward

			// Hopefully, the conversion below should handle the transformation between
			// the two coordinate systems
			// But I'm not completely sure :)
			velocityTransformManager.updateTransform (
								  new Vector3 (-geometryTwist.linear.y,
									       geometryTwist.linear.z ,
									       geometryTwist.linear.x ),
								  new Vector3 (geometryTwist.angular.y,
									       -geometryTwist.angular.z,
									       -geometryTwist.angular.x));
		}
	}

}
