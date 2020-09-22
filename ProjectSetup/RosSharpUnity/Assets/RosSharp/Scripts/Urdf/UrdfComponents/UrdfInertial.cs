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

using System;
using UnityEngine;

namespace RosSharp.Urdf
{
    [RequireComponent(typeof(Rigidbody))]
    public class UrdfInertial : MonoBehaviour
    {
        [SerializeField] private Rigidbody _rigidbody;
        public bool DisplayInertiaGizmo;

        public enum RigidbodyDataSource { Urdf, Unity, Manual};
        public RigidbodyDataSource rigidbodyDataSource;

        public float Mass;
        public Vector3 CenterOfMass;
        public Vector3 InertiaTensor;
        public Quaternion InertiaTensorRotation;

        public float UrdfMass;
        public Vector3 UrdfCenterOfMass;
        public Vector3 UrdfInertiaTensor;
        public Quaternion UrdfInertiaTensorRotation;

        private const int RoundDigits = 10;
        private const float MinInertia = 1e-6f;

        private bool isCreated;

        public static void Create(GameObject linkObject, Link.Inertial inertial = null)
        {
            UrdfInertial urdfInertial = linkObject.AddComponent<UrdfInertial>();
            urdfInertial.UrdfMass = (float)inertial.mass;

            if (inertial.origin != null)
                urdfInertial.UrdfCenterOfMass = UrdfOrigin.GetPositionFromUrdf(inertial.origin);

            urdfInertial.ImportInertiaData(inertial.inertia);
            urdfInertial.Initialize();
            urdfInertial.isCreated = true;
        }

        private void Initialize()
        {
            rigidbodyDataSource = RigidbodyDataSource.Urdf;
  
            Mass = UrdfMass;
            CenterOfMass = UrdfCenterOfMass;
            InertiaTensor = UrdfInertiaTensor;
            InertiaTensorRotation =  UrdfInertiaTensorRotation;

            DisplayInertiaGizmo = false;

            UpdateRigidBodyData();
        }

        #region Runtime
        private void Reset()
        {
            if(isCreated)
                Initialize();
        }

         private void OnValidate()
        {
            if (isCreated)
                UpdateRigidBodyData();
        }

        public void UpdateRigidBodyData()
        {
            _rigidbody = GetComponent<Rigidbody>();

            switch (rigidbodyDataSource)
            {
                case RigidbodyDataSource.Urdf:
                    {
                        _rigidbody.mass = UrdfMass;
                        _rigidbody.centerOfMass = UrdfCenterOfMass;
                        _rigidbody.inertiaTensor = UrdfInertiaTensor;
                        _rigidbody.inertiaTensorRotation = UrdfInertiaTensorRotation;
                        return;
                    }
                case RigidbodyDataSource.Unity:
                    {
                        _rigidbody.mass = Mass;
                        bool isKinematic = _rigidbody.isKinematic;
                        _rigidbody.isKinematic = false;
                        _rigidbody.ResetCenterOfMass();
                        _rigidbody.ResetInertiaTensor();
                        _rigidbody.isKinematic = isKinematic;
                        CenterOfMass = _rigidbody.centerOfMass;
                        InertiaTensor = _rigidbody.inertiaTensor;
                        InertiaTensorRotation = _rigidbody.inertiaTensorRotation;
                        return;
                    }
                case RigidbodyDataSource.Manual:
                    {
                        _rigidbody.mass = Mass;
                        _rigidbody.centerOfMass = CenterOfMass;
                        _rigidbody.inertiaTensor = InertiaTensor;
                        _rigidbody.inertiaTensorRotation = InertiaTensorRotation;
                        return;
                    }
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

        #region Import

        private void ImportInertiaData(Link.Inertial.Inertia inertia)
        {
            Vector3 eigenvalues;
            Vector3[] eigenvectors;
            Matrix3x3 rotationMatrix = ToMatrix3x3(inertia);
            rotationMatrix.DiagonalizeRealSymmetric(out eigenvalues, out eigenvectors);

            UrdfInertiaTensor = ToUnityInertiaTensor(FixMinInertia(eigenvalues));
            Debug.Log(UrdfInertiaTensor);
            UrdfInertiaTensorRotation = ToQuaternion(eigenvectors[0], eigenvectors[1], eigenvectors[2]).Ros2Unity();
            Debug.Log(UrdfInertiaTensorRotation);
        }

        private static Vector3 ToUnityInertiaTensor(Vector3 vector3)
        {
            return new Vector3(vector3.y, vector3.z, vector3.x);
        }

        private static Matrix3x3 ToMatrix3x3(Link.Inertial.Inertia inertia)
        {
            return new Matrix3x3(
                new[] { (float)inertia.ixx, (float)inertia.ixy, (float)inertia.ixz,
                                             (float)inertia.iyy, (float)inertia.iyz,
                                                                 (float)inertia.izz });
        }

        private static Vector3 FixMinInertia(Vector3 vector3)
        {
            for (int i = 0; i < 3; i++)
            {
                if (vector3[i] < MinInertia)
                    vector3[i] = MinInertia;
            }
            return vector3;
        }

        private static Quaternion ToQuaternion(Vector3 eigenvector0, Vector3 eigenvector1, Vector3 eigenvector2)
        {
            //From http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
            float tr = eigenvector0[0] + eigenvector1[1] + eigenvector2[2];
            float qw, qx, qy, qz;
            if (tr > 0)
            {
                float s = Mathf.Sqrt(tr + 1.0f) * 2f; // S=4*qw 
                qw = 0.25f * s;
                qx = (eigenvector1[2] - eigenvector2[1]) / s;
                qy = (eigenvector2[0] - eigenvector0[2]) / s;
                qz = (eigenvector0[1] - eigenvector1[0]) / s;
            }
            else if ((eigenvector0[0] > eigenvector1[1]) & (eigenvector0[0] > eigenvector2[2]))
            {
                float s = Mathf.Sqrt(1.0f + eigenvector0[0] - eigenvector1[1] - eigenvector2[2]) * 2; // S=4*qx 
                qw = (eigenvector1[2] - eigenvector2[1]) / s;
                qx = 0.25f * s;
                qy = (eigenvector1[0] + eigenvector0[1]) / s;
                qz = (eigenvector2[0] + eigenvector0[2]) / s;
            }
            else if (eigenvector1[1] > eigenvector2[2])
            {
                float s = Mathf.Sqrt(1.0f + eigenvector1[1] - eigenvector0[0] - eigenvector2[2]) * 2; // S=4*qy
                qw = (eigenvector2[0] - eigenvector0[2]) / s;
                qx = (eigenvector1[0] + eigenvector0[1]) / s;
                qy = 0.25f * s;
                qz = (eigenvector2[1] + eigenvector1[2]) / s;
            }
            else
            {
                float s = Mathf.Sqrt(1.0f + eigenvector2[2] - eigenvector0[0] - eigenvector1[1]) * 2; // S=4*qz
                qw = (eigenvector0[1] - eigenvector1[0]) / s;
                qx = (eigenvector2[0] + eigenvector0[2]) / s;
                qy = (eigenvector2[1] + eigenvector1[2]) / s;
                qz = 0.25f * s;
            }
            return new Quaternion(qx, qy, qz, qw);
        }

        #endregion

        #region Export
        public Link.Inertial ExportInertialData()
        {
            // should we read the data from this clas instead of _rigidbody?
            Origin inertialOrigin = new Origin(_rigidbody.centerOfMass.Unity2Ros().ToRoundedDoubleArray(), new double[] { 0, 0, 0 });
            Link.Inertial.Inertia inertia = ExportInertiaData(_rigidbody);

            return new Link.Inertial(Math.Round(_rigidbody.mass, RoundDigits), inertialOrigin, inertia);
        }

        private static Link.Inertial.Inertia ExportInertiaData(Rigidbody _rigidbody)
        {
            Matrix3x3 lamdaMatrix = new Matrix3x3(new[] {
                _rigidbody.inertiaTensor[0],
                _rigidbody.inertiaTensor[1],
                _rigidbody.inertiaTensor[2] });

            Matrix3x3 qMatrix = Quaternion2Matrix(_rigidbody.inertiaTensorRotation);

            Matrix3x3 qMatrixTransposed = qMatrix.Transpose();

            Matrix3x3 inertiaMatrix = qMatrix * lamdaMatrix * qMatrixTransposed;

            return ToRosCoordinates(ToInertia(inertiaMatrix));
        }

        private static Matrix3x3 Quaternion2Matrix(Quaternion quaternion)
        {
            Quaternion rosQuaternion = Quaternion.Normalize(quaternion);
            float qx = rosQuaternion.x;
            float qy = rosQuaternion.y;
            float qz = rosQuaternion.z;
            float qw = rosQuaternion.w;

            //From http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
            return new Matrix3x3(new float[] {
                1 - (2 * qy * qy) - (2 * qz * qz),
                (2 * qx * qy) - (2 * qz * qw),
                (2 * qx * qz) + (2 * qy * qw),

                (2 * qx * qy) + (2 * qz * qw),
                1 - (2 * qx * qx) - (2 * qz * qz),
                (2 * qy * qz) - (2 * qx * qw),

                (2 * qx * qz) - (2 * qy * qw),
                (2 * qy * qz) + (2 * qx * qw),
                1 - (2 * qx * qx) - (2 * qy * qy)});
        }

        private static Link.Inertial.Inertia ToInertia(Matrix3x3 matrix)
        {
            return new Link.Inertial.Inertia(matrix[0][0], matrix[0][1], matrix[0][2],
                matrix[1][1], matrix[1][2],
                matrix[2][2]);
        }

        private static Link.Inertial.Inertia ToRosCoordinates(Link.Inertial.Inertia unityInertia)
        {
            return new Link.Inertial.Inertia(0, 0, 0, 0, 0, 0)
            {
                ixx = unityInertia.izz,
                iyy = unityInertia.ixx,
                izz = unityInertia.iyy,

                ixy = -unityInertia.ixz,
                ixz = unityInertia.iyz,
                iyz = -unityInertia.ixy
            };
        }
        #endregion
    }
}
