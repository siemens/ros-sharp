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
    public static class UrdfOriginExtensions
    {
        public static void SetTransform(this Origin origin, GameObject gameObject)
        {
            gameObject.transform.Translate(origin.GetPosition());
            gameObject.transform.Rotate(origin.GetRotation());
        }
        public static Vector3 GetPosition(this Origin origin)
        {
            if (origin.Xyz != null)
                return new Vector3(
                    (float)-origin.Xyz[1],
                    (float)+origin.Xyz[2],
                    (float)+origin.Xyz[0]);
            else
                return Vector3.zero;
        }
        public static Vector3 GetRotation(this Origin origin)
        {
            if (origin.Rpy != null)
                return new Vector3(
                    (float)+origin.Rpy[1] * Mathf.Rad2Deg,
                    (float)-origin.Rpy[2] * Mathf.Rad2Deg,
                    (float)-origin.Rpy[0] * Mathf.Rad2Deg);
            else
                return Vector3.zero;
        }
    }
}