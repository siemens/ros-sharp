/*
© Siemens AG, 2018
Author: Suzannah Smith (suzannah.smith@siemens.com)

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

namespace RosSharp.Urdf
{
    [RequireComponent(typeof(Rigidbody))]
    public class UrdfInertial : MonoBehaviour
    {
        public bool DisplayInertiaGizmo;

        public bool UseUrdfData;
        public Vector3 CenterOfMass;
        public Vector3 InertiaTensor;
        public Quaternion InertiaTensorRotation;

       #region ManageRigidbodyData

        private void Start()
        {
            UpdateRigidBodyData();
        }

        public void UpdateRigidBodyData()
        {
            Rigidbody _rigidbody = GetComponent<Rigidbody>();

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

        private void OnDrawGizmosSelected()
        {
            if (DisplayInertiaGizmo)
            {
                Gizmos.color = Color.blue;
                Gizmos.DrawRay(transform.position, GetComponent<Rigidbody>().inertiaTensorRotation * Vector3.forward * GetComponent<Rigidbody>().inertiaTensor.z);
                Gizmos.color = Color.green;
                Gizmos.DrawRay(transform.position, GetComponent<Rigidbody>().inertiaTensorRotation * Vector3.up * GetComponent<Rigidbody>().inertiaTensor.y);
                Gizmos.color = Color.red;
                Gizmos.DrawRay(transform.position, GetComponent<Rigidbody>().inertiaTensorRotation * Vector3.right * GetComponent<Rigidbody>().inertiaTensor.x);
            }
        }

        #endregion

    }
}
