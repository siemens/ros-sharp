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
    [ExecuteInEditMode]
    public class JointLimitsManager : MonoBehaviour
    {
        public float LargeAngleLimitMin;
        public float LargeAngleLimitMax;

        public float Tolerance = 5;

        public float AngleActual { get; private set; }
        private float AnglePrevious;

        public int RotationNumberActual { get; private set; }

        public int RotationLimitMin { get; private set; }
        public int RotationLimitMax { get; private set; }

        public float AngleLimitMin { get; private set; }
        public float AngleLimitMax { get; private set; }

        private HingeJoint _hingeJoint;
        private Vector3 RotationAxis;

        private void Awake()
        {
            _hingeJoint = GetComponent<HingeJoint>();
            RecalculateJointLimits();
        }

        private void Update()
        {
            RecalculateJointLimits();
        }

        private void FixedUpdate()
        {
            UpdateAngles();
            ApplyJointLimits();
        }

        private void RecalculateJointLimits()
        {
            AngleLimitMin = ((LargeAngleLimitMin - 180) % 360) + 180;
            RotationLimitMin = (int)(LargeAngleLimitMin - 180) / 360;

            if (180 - AngleLimitMin < Tolerance)
            {
                AngleLimitMin = -180;
                RotationLimitMin += 1;
            }

            AngleLimitMax = ((LargeAngleLimitMax + 180) % 360) - 180;
            RotationLimitMax = (int)(LargeAngleLimitMax + 180) / 360;

            if (180 + AngleLimitMax < Tolerance)
            {
                AngleLimitMax = 180;
                RotationLimitMax -= 1;
            }
        }

        private void UpdateAngles()
        {
            AnglePrevious = AngleActual;

            AngleActual = _hingeJoint.angle;
            
            if (AnglePrevious < -90 && AngleActual > 90)
                RotationNumberActual -= 1;
            else if (AnglePrevious > 90 && AngleActual < -90)
                RotationNumberActual += 1;
        }

        private void ApplyJointLimits()
        {
            if (RotationNumberActual == RotationLimitMin && RotationNumberActual == RotationLimitMax)
            {
                _hingeJoint.limits = updateLimits(_hingeJoint.limits, AngleLimitMin, AngleLimitMax);
                _hingeJoint.useLimits = true;
            }
            else if (RotationNumberActual == RotationLimitMin && AngleActual - AngleLimitMin <= Tolerance)
            {
                _hingeJoint.limits = updateLimits(_hingeJoint.limits, AngleLimitMin, 180);
                _hingeJoint.useLimits = true;
            }
            else if (RotationNumberActual == RotationLimitMax && AngleLimitMax - AngleActual <= Tolerance)
            {
                _hingeJoint.limits = updateLimits(_hingeJoint.limits, -180, AngleLimitMax);
                _hingeJoint.useLimits = true;
            }
            else
                _hingeJoint.useLimits = false;
        }

        private static JointLimits updateLimits(JointLimits jointLimits, float min, float max)
        {
            jointLimits.min = min;
            jointLimits.max = max;
            return jointLimits;
        }
    }
}
