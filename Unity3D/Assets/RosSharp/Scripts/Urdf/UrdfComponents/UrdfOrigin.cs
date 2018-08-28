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
    class UrdfOrigin
    {
        public static void SetTransform(Transform transform, Origin origin)
        {
            if (origin != null)
            {
                transform.Translate(GetPosition(origin));
                transform.Rotate(GetRotation(origin));
            }
        }
        public static Vector3 GetPosition(Origin origin)
        {
            if (origin.Xyz != null)
                return new Vector3(
                    (float)-origin.Xyz[1],
                    (float)+origin.Xyz[2],
                    (float)+origin.Xyz[0]);
            
            return Vector3.zero;
        }
        public static Vector3 GetRotation(Origin origin)
        {
            if (origin.Rpy != null)
                return new Vector3(
                    (float)+origin.Rpy[1] * Mathf.Rad2Deg,
                    (float)-origin.Rpy[2] * Mathf.Rad2Deg,
                    (float)-origin.Rpy[0] * Mathf.Rad2Deg);

            return Vector3.zero;
        }

        public static Origin GetOriginData(Transform transform)
        {
            double[] xyz = GetUrdfXyz(transform);
            double[] rpy = GetUrdfRpy(transform);

            if (xyz != null || rpy != null)
                return new Origin(xyz, rpy);

            return null;
        }

        private static double[] GetUrdfXyz(Transform transform)
        {
            Vector3 xyzVector = transform.localPosition.Unity2Ros();
            return xyzVector == Vector3.zero ? null : xyzVector.ToRoundedDoubleArray();
        }

        private static double[] GetUrdfRpy(Transform transform)
        {
            Vector3 rpyVector = new Vector3(
                -transform.localEulerAngles.z * Mathf.Deg2Rad,
                transform.localEulerAngles.x * Mathf.Deg2Rad,
                -transform.localEulerAngles.y * Mathf.Deg2Rad);

            return rpyVector == Vector3.zero ? null : rpyVector.ToRoundedDoubleArray();
        }
    }
}
