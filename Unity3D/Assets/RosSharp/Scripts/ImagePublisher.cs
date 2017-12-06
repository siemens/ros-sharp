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


		void Start () {
			rosSocket = transform.GetComponent<RosConnector>().RosSocket;
			pub_id = rosSocket.Advertize (baseTopic + "/compressed", "sensor_msgs/CompressedImage");
		}

		void Update () {

			//
			RenderTexture rt = new RenderTexture(resWidth, resHeight, 24);
			cameraToPublish.targetTexture = rt;

			Texture2D screenShot = new Texture2D(resWidth, resHeight, TextureFormat.RGB24, false);
			cameraToPublish.Render();
			RenderTexture.active = rt;
			screenShot.ReadPixels(new Rect(0, 0, resWidth, resHeight), 0, 0);
			cameraToPublish.targetTexture = null;
			RenderTexture.active = null; // JC: added to avoid errors
			Destroy(rt);

			// Build up the message and publish
			SensorCompressedImage message = new SensorCompressedImage ();
			message.header.frame_id = "test";
			message.format = "jpeg";
			message.data = screenShot.EncodeToJPG (qualityLevel);
			rosSocket.Publish (pub_id, message);
		}
	}
}
