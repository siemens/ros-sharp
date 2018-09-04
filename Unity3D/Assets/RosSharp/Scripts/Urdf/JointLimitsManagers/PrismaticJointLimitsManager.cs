/*
© Siemens AG, 2017-2018
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

namespace RosSharp
{
    public class PrismaticJointLimitsManager : MonoBehaviour
    {
        public float PositionLimitMin;
        public float PositionLimitMax;

        public float Tolerance = 0.01f;

        private ConfigurableJoint configurableJoint;
        private float referencePosition;

        private void Awake()
        {
            configurableJoint = GetComponent<ConfigurableJoint>();
            referencePosition = Vector3.Dot(transform.localPosition, configurableJoint.axis);
        }

        private void FixedUpdate()
        {
            ApplyLimits();
        }

        private void OnValidate()
        {
            if (PositionLimitMax < PositionLimitMin)
                PositionLimitMax = PositionLimitMin;
        }

        private void ApplyLimits()
        {
            float position = Vector3.Dot(transform.localPosition, configurableJoint.axis) - referencePosition;

            Debug.Log(position);

            if (position - PositionLimitMin < Tolerance)
            {
                configurableJoint.xMotion = ConfigurableJointMotion.Limited;
                configurableJoint.linearLimit = UpdateLimit(configurableJoint.linearLimit, -PositionLimitMin);
            }
            else if (PositionLimitMax - position < Tolerance)
            {
                configurableJoint.xMotion = ConfigurableJointMotion.Limited;
                configurableJoint.linearLimit = UpdateLimit(configurableJoint.linearLimit, PositionLimitMax);
            }
            else
            {
                configurableJoint.xMotion = ConfigurableJointMotion.Free;
            }
        }

        private static SoftJointLimit UpdateLimit(SoftJointLimit softJointLimit, float limit)
        {
            softJointLimit.limit = limit;
            return softJointLimit;
        }

        public void InitializeLimits(Urdf.Joint.Limit limit)
        {
            PositionLimitMax = (float)limit.upper;
            PositionLimitMin = (float)limit.lower;
        }

    }
}
