using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosConnector))]
    public class ImagePublisher : MonoBehaviour {
		
	public Camera camera;
	public string topic = "/image_raw/compressed";
	public string frameId = "camera";

	public int resolutionWidth = 640; 
	public int resolutionHeight = 480;

	[Range(0, 100)]
	public int qualityLevel = 50;

	private RosSocket rosSocket;
	private int publicationId;
	private SensorCompressedImage message;
	private int sequenceId;

	private Texture2D texture2D;
	private RenderTexture renderTexture;
	private Rect rect;

	void Start () {
	    // The ROS part
	    rosSocket = transform.GetComponent<RosConnector>().RosSocket;
	    publicationId = rosSocket.Advertize (topic, "sensor_msgs/CompressedImage");
	    message = new SensorCompressedImage ();
	    sequenceId = 0;

	    // The Unity part for rendering the image 
	    texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RGB24, false);
	    rect = new Rect (0, 0, resolutionWidth, resolutionHeight);
	    renderTexture = new RenderTexture(resolutionWidth, resolutionHeight, 24);
	}

	void Update () {

	    // Render the image 
	    camera.targetTexture = renderTexture;
	    camera.Render();
	    RenderTexture.active = renderTexture;
	    texture2D.ReadPixels(rect, 0, 0);
	    camera.targetTexture = null;
	    RenderTexture.active = null;

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
