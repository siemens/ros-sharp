/*
© CentraleSupelec AG, 2017
Author: Dr. Jeremy Fix (jeremy.fix@centralesupelec.fr)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

<http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

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
