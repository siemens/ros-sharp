/*
© Siemens AG, 2017-2018
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
    public class HingeJointLimitsManager : MonoBehaviour
    {
        public float LargeAngleLimitMin;
        public float LargeAngleLimitMax;

        public float Tolerance = 5;

        public float AngleActual; // { get; private set; }
        private float anglePrevious;

        public int RotationNumberActual; // { get; private set; }

        public int RotationNumberMin; // { get; private set; }
        public int RotationNumberMax; // { get; private set; }

        public float AngleLimitMin; // { get; private set; }
        public float AngleLimitMax; // { get; private set; }

        private HingeJoint _hingeJoint;

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
            RecalculateJointLimits();

            UpdateAngles();
        }
        
        private void RecalculateJointLimits()
        {
            if (LargeAngleLimitMax < LargeAngleLimitMin)
                LargeAngleLimitMin = LargeAngleLimitMax;

            AngleLimitMin = GetAngleLimit(LargeAngleLimitMin);
            RotationNumberMin = GetRotationLimit(LargeAngleLimitMin);

            AngleLimitMax = GetAngleLimit(LargeAngleLimitMax);
            RotationNumberMax = GetRotationLimit(LargeAngleLimitMax);
            
            FixAngleLimits();

            ApplyJointLimits();
        }

        //Keeps limits from getting near +/- 180, where the joint gets stuck
        private void FixAngleLimits()
        {
            if (LargeAngleLimitMin < 0 && 180 - AngleLimitMin < Tolerance)
            {
                AngleLimitMin = -180;
                RotationNumberMin += 1;
            }
            else if (LargeAngleLimitMin > 0 && 180 + AngleLimitMin < Tolerance)
            {
                AngleLimitMin = 180;
                RotationNumberMin -= 1;
            }

            if (LargeAngleLimitMax > 0 && 180 + AngleLimitMax < Tolerance)
            {
                AngleLimitMax = 180;
                RotationNumberMax -= 1;
            }
            else if (LargeAngleLimitMax < 0 && 180 - AngleLimitMax < Tolerance)
            {
                AngleLimitMax = -180;
                RotationNumberMax += 1;
            }
        }

        private float GetAngleLimit(float largeAngleLimit)
        {
            if (largeAngleLimit > 0)
                return ((largeAngleLimit + 180) % 360) - 180;
            else
                return ((largeAngleLimit - 180) % 360) + 180;
        }

        private int GetRotationLimit(float largeAngleLimit)
        {
            if (largeAngleLimit > 0)
                return (int) ((largeAngleLimit + 180) / 360);
            else
                return (int) ((largeAngleLimit - 180) / 360);

        }

        private void UpdateAngles()
        {
            anglePrevious = AngleActual;

            AngleActual = _hingeJoint.angle;
            
            if (anglePrevious < -90 && AngleActual > 90)
                RotationNumberActual -= 1;
            else if (anglePrevious > 90 && AngleActual < -90)
                RotationNumberActual += 1;
        }

        private void ApplyJointLimits()
        {
            if (RotationNumberActual == RotationNumberMin && RotationNumberActual == RotationNumberMax)
            {
                _hingeJoint.limits = UpdateLimits(_hingeJoint.limits, AngleLimitMin, AngleLimitMax);
                _hingeJoint.useLimits = true;
            }
            else if (RotationNumberActual == RotationNumberMin && AngleActual - AngleLimitMin <= Tolerance)
            {
                _hingeJoint.limits = UpdateLimits(_hingeJoint.limits, AngleLimitMin, 180);
                _hingeJoint.useLimits = true;
            }
            else if (RotationNumberActual == RotationNumberMax && AngleLimitMax - AngleActual <= Tolerance)
            {
                _hingeJoint.limits = UpdateLimits(_hingeJoint.limits, -180, AngleLimitMax);
                _hingeJoint.useLimits = true;
            }
            else
                _hingeJoint.useLimits = false;
        }

        private static JointLimits UpdateLimits(JointLimits jointLimits, float min, float max)
        {
            jointLimits.min = min;
            jointLimits.max = max;
            return jointLimits;
        }

        public void InitializeLimits(Urdf.Joint.Limit limit, HingeJoint joint)
        {
            LargeAngleLimitMin = (float)limit.upper * -1.0f * Mathf.Rad2Deg;
            LargeAngleLimitMax = (float)limit.lower * -1.0f * Mathf.Rad2Deg;

            _hingeJoint = joint;

            RecalculateJointLimits();
        }
    }
}
