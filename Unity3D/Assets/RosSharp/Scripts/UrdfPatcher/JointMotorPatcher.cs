/*
© Siemens AG, 2017
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

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

namespace RosSharp
{

    public class JointMotorPatcher : MonoBehaviour
    {
        public GameObject UrdfModel;
        public float MotorForce;
        public float MaxVelocity;

        public void patch()
        {
            foreach (Transform child in UrdfModel.GetComponentsInChildren<Transform>())
            {
                if (child.name.Contains("continuous Joint") || child.name.Contains("revolute Joint"))
                    patch(child.gameObject);
            }
        }
        private void patch(GameObject _gameObject)
        {
            JointMotorManager jointMotorManager = _gameObject.GetComponent<JointMotorManager>();
            if (jointMotorManager != null)
                DestroyImmediate(jointMotorManager);
            jointMotorManager = _gameObject.AddComponent<JointMotorManager>();
            jointMotorManager.MaxVelocity = MaxVelocity;

            HingeJoint hingeJoint = _gameObject.GetComponent<HingeJoint>();
            hingeJoint.motor = patch(hingeJoint.motor);
            hingeJoint.useMotor = true;

            Debug.Log("Joint Motor manager applied to joint: " + _gameObject.name);
        }

        private JointMotor patch (JointMotor jointMotor)
        {
            jointMotor.force = MotorForce;
            return jointMotor;
        }
    }
}