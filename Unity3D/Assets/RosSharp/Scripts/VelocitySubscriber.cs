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
