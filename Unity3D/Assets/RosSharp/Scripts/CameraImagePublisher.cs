using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosConnector))]
    public class CameraImagePublisher : MonoBehaviour {

	public string topic = "/image_raw/compressed";
	public int resolutionWidth = 640; 
	public int resolutionHeight = 480;

	[Range(0, 100)]
	public int qualityLevel = 50;
	
	private RosSocket rosSocket;
	private int publicationId;
	private SensorCompressedImage message;
	private int sequenceId;
	public string frameId = "camera";


	void Start () {
	    // The ROS part
	    rosSocket = transform.GetComponent<RosConnector>().RosSocket;
	    publicationId = rosSocket.Advertize (topic, "sensor_msgs/CompressedImage");
	    message = new SensorCompressedImage ();
	    sequenceId = 0;
	}


	public void Publish (Texture2D texture2D) {
	    // Build up the message and publish
	    message.header.frame_id = frameId;
	    message.header.seq = sequenceId;
	    message.format = "jpeg";
	    message.data = texture2D.EncodeToJPG (qualityLevel);
	    rosSocket.Publish (publicationId, message);

	    ++sequenceId;
	}
    }
}
