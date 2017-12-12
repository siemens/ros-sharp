

using UnityEngine;

namespace RosSharp.RosBridgeClient
{

    [RequireComponent(typeof(RosConnector))]
    public class ImageSubscriber : MonoBehaviour
    {
        public GameObject Texture;
        private ImageManager imageManager;
        private RosSocket rosSocket;
        public string topic = "/image_raw";
        public int UpdateTime = 1;

        public void Start()
        {
            rosSocket = transform.GetComponent<RosConnector>().RosSocket;
            rosSocket.Subscribe(topic, "sensor_msgs/Image", updateTexture, UpdateTime);

            imageManager = Texture.GetComponent<ImageManager>();
        }

        private void updateTexture(Message message)
        {
            SensorImage sensorImage = (SensorImage) message;
            imageManager.UpdateTexture(sensorImage);
        }
    }
}
