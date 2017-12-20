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
    [RequireComponent(typeof(HingeJoint))]
    public class JointMotorManager : MonoBehaviour
    {
        private HingeJoint _hingeJoint;
        public string AxisName;
        public float MaxVelocity;
        public float ControlInputValue;

        private void Start()
        {
            _hingeJoint = GetComponent<HingeJoint>();
        }

        private void Update()
        {
            UpdateAxisInput();            
            JointMotor jointMotor = _hingeJoint.motor;
            jointMotor.targetVelocity = ControlInputValue * MaxVelocity;
            _hingeJoint.motor = jointMotor;
            ControlInputValue = 0;
        }

        private void UpdateAxisInput()
        {
            if (Input.GetAxis(AxisName) != 0)
                ControlInputValue = Input.GetAxis(AxisName);
        }
    }
}