using UnityEngine;
using System.Collections;

namespace RosSharp.RosBridgeClient
{

	[RequireComponent(typeof(RosConnector))]
	public class VelocitySubscriber : MonoBehaviour
	{
		public GameObject Model;

		public string topic = "/cmd_vel";

		private TransformManager transformManager;
		private RosSocket rosSocket;

		public int UpdateTime = 1;

		// Use this for initialization
		void Start ()
		{
			rosSocket = transform.GetComponent<RosConnector>().RosSocket;
			rosSocket.Subscribe(topic, "geometry_msgs/Twist", updateCmdVel, UpdateTime);

			transformManager = Model.GetComponent<TransformManager>();		
		}

		private void updateCmdVel(Message message)
		{
			GeometryTwist twist_msg = (GeometryTwist)message;
			// In ROS conventions : 
			// x points forward
			// y points to the left
			// z points to the top

			// In Unity conventions: 
			// x points to the right
			// y points to the top
			// z points forward
			transformManager.updateTransform (new Vector3 (twist_msg.linear.y, twist_msg.linear.z , twist_msg.linear.x ),
				new Vector3 (-twist_msg.angular.y, -twist_msg.angular.z, -twist_msg.angular.x));
		}
	}

}