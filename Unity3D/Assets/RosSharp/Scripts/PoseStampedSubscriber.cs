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