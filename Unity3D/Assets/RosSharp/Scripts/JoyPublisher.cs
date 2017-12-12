using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosConnector))]
    public class JoyPublisher : MonoBehaviour
    {
        private RosSocket rosSocket;
        public string topic = "/joy";
        public int UpdateTime = 1;
        private int advertizer;
        private OculusButtonsManager joyTransformManager;

        public void Start()
        {
            rosSocket = transform.GetComponent<RosConnector>().RosSocket;
            advertizer = rosSocket.Advertize(topic, "sensor_msgs/Joy");
            joyTransformManager = this.GetComponent<OculusButtonsManager>();
        }

        private void publishJoy(SensorJoy message)
        {
            rosSocket.Publish(advertizer, message);
        }

        private void setPublisher(string topic)
        {
            rosSocket.Unadvertize(advertizer);
            advertizer = rosSocket.Advertize(topic, "sensor_msgs/Joy");
        }

        public void Update()
        {
            if(joyTransformManager.HasChanged())
            {
                this.publishJoy(joyTransformManager.GetSensorJoy());
            }
        }
    }
}
