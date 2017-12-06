using UnityEngine;
using System.Collections;

namespace RosSharp
{
	public class TransformManager : MonoBehaviour
	{
		public bool useForceTorque = false;

		private Vector3 linear_velocity;
		private Vector3 angular_velocity;
		private bool doUpdate;

		void Start()
		{
			linear_velocity = Vector3.zero;
			angular_velocity = Vector3.zero;
		}

		// Update is called once per frame
		void Update ()
		{
			//if (doUpdate)
			//{
				transform.Translate (linear_velocity * Time.deltaTime);
				transform.Rotate(Vector3.up, angular_velocity.y * Time.deltaTime);
				//doUpdate = false;
			//}		
		}

		public void updateTransform(Vector3 _linear_velocity, Vector3 _angular_velocity)
		{
			linear_velocity = _linear_velocity;
			angular_velocity = _angular_velocity;
			//doUpdate = true;
		}
	}
}
