using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(Camera))]
    public class CameraImageManager : MonoBehaviour {

	public GameObject RosObject;
	
	private CameraImagePublisher cameraImagePublisher;

	public int resolutionWidth = 640; 
	public int resolutionHeight = 480;
	
	private Texture2D texture2D;
	private RenderTexture renderTexture;
	private Rect rect;

	private Camera _camera;
	
	void Start () {
	    cameraImagePublisher = RosObject.GetComponent<CameraImagePublisher>();

	    _camera = GetComponent<Camera>();
	    
	    texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RGB24, false);
	    rect = new Rect (0, 0, resolutionWidth, resolutionHeight);
	    renderTexture = new RenderTexture(resolutionWidth, resolutionHeight, 24);
	}

	void Update () {
	    // Render the image 
	    _camera.targetTexture = renderTexture;
	    _camera.Render();
	    RenderTexture.active = renderTexture;
	    texture2D.ReadPixels(rect, 0, 0);
	    _camera.targetTexture = null;
	    RenderTexture.active = null;

	    cameraImagePublisher.Publish(texture2D);
	}	
    }
}
