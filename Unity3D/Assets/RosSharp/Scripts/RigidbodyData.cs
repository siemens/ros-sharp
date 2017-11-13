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


namespace RosSharp.UrdfImporter
{
    [ExecuteInEditMode]
    [RequireComponent(typeof(Rigidbody))]
    public class RigidbodyData : MonoBehaviour
    {
        public bool UseUrdfData;
        public Vector3 CenterOfMass;
        public Vector3 InertiaTensor;
        public Quaternion InertiaTensorRotation;

        private Rigidbody _rigidbody;

        private void Start()
        {
            UpdateRigidBodyData();
        }

        public void UpdateRigidBodyData()
        {
            _rigidbody = GetComponent<Rigidbody>();

            if (UseUrdfData)
            {
                _rigidbody.centerOfMass = CenterOfMass;
                _rigidbody.inertiaTensor = InertiaTensor;
                _rigidbody.inertiaTensorRotation = InertiaTensorRotation;
            }
            else
            {
                _rigidbody.ResetCenterOfMass();
                _rigidbody.ResetInertiaTensor();
            }

        }

        public void GetValuesFromUrdf(Vector3 centerOfMass, Vector3 inertiaTensor, Quaternion inertiaTensorRotation)
        {
            CenterOfMass = centerOfMass;
            InertiaTensor = inertiaTensor;
            InertiaTensorRotation = inertiaTensorRotation;
        }
    }
}