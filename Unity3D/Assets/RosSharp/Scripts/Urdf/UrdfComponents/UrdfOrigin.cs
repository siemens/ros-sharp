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
        #region Import

        public static void SetTransformFromUrdf(Transform transform, Origin origin)
        {
            if (origin != null)
            {
                transform.Translate(GetPositionFromUrdf(origin));
                transform.Rotate(GetRotationFromUrdf(origin));
            }
        }
        public static Vector3 GetPositionFromUrdf(Origin origin)
        {
            if (origin.Xyz != null)
                return origin.Xyz.ToVector3().Ros2Unity();
            
            return Vector3.zero;
        }
        public static Vector3 GetRotationFromUrdf(Origin origin)
        {
            if (origin.Rpy != null)
            {
                Vector3 rotation = origin.Rpy.ToVector3();
            }
            return new Vector3(
                (float)+origin.Rpy[1] * Mathf.Rad2Deg,
                (float)-origin.Rpy[2] * Mathf.Rad2Deg,
                (float)-origin.Rpy[0] * Mathf.Rad2Deg);

            return Vector3.zero;
        }

        #endregion

        #region Export

        public static Origin ExportOriginToUrdf(Transform transform)
        {
            double[] xyz = ExportXyzToUrdf(transform);
            double[] rpy = ExportRpyToUrdf(transform);

            if (xyz != null || rpy != null)
                return new Origin(xyz, rpy);

            return null;
        }

        private static double[] ExportXyzToUrdf(Transform transform)
        {
            Vector3 xyzVector = transform.localPosition.Unity2Ros();
            return xyzVector == Vector3.zero ? null : xyzVector.ToRoundedDoubleArray();
        }

        private static double[] ExportRpyToUrdf(Transform transform)
        {
            Vector3 rpyVector = new Vector3(
                -transform.localEulerAngles.z * Mathf.Deg2Rad,
                transform.localEulerAngles.x * Mathf.Deg2Rad,
                -transform.localEulerAngles.y * Mathf.Deg2Rad);

            return rpyVector == Vector3.zero ? null : rpyVector.ToRoundedDoubleArray();
        }

        #endregion
    }
}
