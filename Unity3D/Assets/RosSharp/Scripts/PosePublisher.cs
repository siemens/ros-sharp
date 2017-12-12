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

        public void Start()
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

        public void Update()
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

        public static GeometryPose gameObjectTransformToGeometryPose(Transform transform)
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
