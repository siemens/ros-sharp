using UnityEngine;
using System.Collections;

namespace RosSharp.RosBridgeClient
{
    public class VelocityTransformManager : MonoBehaviour
    {
	private Vector3 linearVelocity;
	private Vector3 angularVelocity;

	void Start()
	{
	    linearVelocity = Vector3.zero;
	    angularVelocity = Vector3.zero;
	}

	// Update is called once per frame
	void Update ()
	{
	    transform.Translate (linearVelocity * Time.deltaTime);
	    transform.Rotate(Vector3.forward, angularVelocity.x * Time.deltaTime);	
	    transform.Rotate(Vector3.up     , angularVelocity.y * Time.deltaTime);	
	    transform.Rotate(Vector3.left   , angularVelocity.z * Time.deltaTime);	
	}

	public void updateTransform(Vector3 _linearVelocity, Vector3 _angularVelocity)
	{
	    linearVelocity = _linearVelocity;
	    angularVelocity = _angularVelocity;
	}
    }
}
