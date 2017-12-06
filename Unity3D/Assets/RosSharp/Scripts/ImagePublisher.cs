using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
	[RequireComponent(typeof(RosConnector))]
	public class ImagePublisher : MonoBehaviour {
		
		public Camera cameraToPublish;
		public string baseTopic = "/image_raw";

		public int resWidth = 640; 
		public int resHeight = 480;

		[Range(0, 100)]
		public int qualityLevel = 50;

		private RosSocket rosSocket;
		private int pub_id;

		private Texture2D screenShot;
		private RenderTexture rt;
		private Rect viewport;
		private SensorCompressedImage message;

		void Start () {
			rosSocket = transform.GetComponent<RosConnector>().RosSocket;
			pub_id = rosSocket.Advertize (baseTopic + "/compressed", "sensor_msgs/CompressedImage");
			screenShot = new Texture2D(resWidth, resHeight, TextureFormat.RGB24, false);
			viewport = new Rect (0, 0, resWidth, resHeight);
			rt = new RenderTexture(resWidth, resHeight, 24);
			message = new SensorCompressedImage ();
		}

		void Update () {

			//
			cameraToPublish.targetTexture = rt;
			cameraToPublish.Render();
			RenderTexture.active = rt;
			screenShot.ReadPixels(viewport, 0, 0);
			cameraToPublish.targetTexture = null;
			RenderTexture.active = null; // JC: added to avoid errors

			// Build up the message and publish
			message.header.frame_id = "test";
			message.format = "jpeg";
			message.data = screenShot.EncodeToJPG (qualityLevel);
			rosSocket.Publish (pub_id, message);
		}
	}
}
