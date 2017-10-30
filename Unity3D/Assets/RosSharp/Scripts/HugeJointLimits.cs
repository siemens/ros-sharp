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
    public class HugeJointLimits : MonoBehaviour
    {
        public float ActualAngle;
        public float PreviousAngle;

        public int RotationCounter;
        public int RotationOfMinLimit;
        public int RotationOfMaxLimit;

        public float MinLimit;
        public float MaxLimit;

        public float ActualMinLimit;
        public float ActualMaxLimit;

        public float LimitActivationDistance;

        private HingeJoint _hingeJoint;
        private Vector3 RotationAxis;

        private void Awake()
        {
            _hingeJoint = GetComponent<HingeJoint>();
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
            ActualMinLimit = ((MinLimit - 180) % 360) + 180;
            RotationOfMinLimit = (int)(MinLimit - 180) / 360;

            if (ActualMinLimit == 180)
            {
                ActualMinLimit = -180;
                RotationOfMinLimit += 1;
            }

            ActualMaxLimit = ((MaxLimit + 180) % 360) - 180;
            RotationOfMaxLimit = (int)(MaxLimit + 180) / 360;

            if (ActualMaxLimit == -180)
            {
                ActualMaxLimit = 180;
                RotationOfMaxLimit -= 1;
            }
        }

        private void UpdateAngles()
        {
            PreviousAngle = ActualAngle;

            RotationAxis = transform.rotation * _hingeJoint.axis;
            ActualAngle = Vector3.Dot(transform.localRotation.eulerAngles, RotationAxis);
            ActualAngle = (ActualAngle > 180 ? ActualAngle - 360 : ActualAngle);

            if (PreviousAngle < -90 && ActualAngle > 90)
                RotationCounter -= 1;
            else if (PreviousAngle > 90 && ActualAngle < -90)
                RotationCounter += 1;
        }

        private void ApplyJointLimits()
        {
            if (RotationCounter == RotationOfMinLimit && ActualAngle - ActualMinLimit <= LimitActivationDistance)
            {
                _hingeJoint.limits = lowerLimit;
                _hingeJoint.useLimits = true;
            }

            else if (RotationCounter == RotationOfMaxLimit && ActualMaxLimit - ActualAngle <= LimitActivationDistance)
            {
                _hingeJoint.limits = upperLimit;
                _hingeJoint.useLimits = true;
            }
            else
                _hingeJoint.useLimits = false;
        }

        private JointLimits lowerLimit
        {
            get
            {
                JointLimits limits = _hingeJoint.limits;
                limits.min = ActualMinLimit;
                limits.max = 180;
                if (limits.max - limits.min <= 2 * LimitActivationDistance)
                    limits.min = limits.max - 2 * LimitActivationDistance;
                return limits;
            }
        }

        private JointLimits upperLimit
        {
            get
            {
                JointLimits limits = _hingeJoint.limits;
                limits.min = -180;
                limits.max = ActualMaxLimit;
                if (limits.max - limits.min <= 2 * LimitActivationDistance)
                    limits.max = limits.min + 2 * LimitActivationDistance;
                return limits;
            }
        }
    }
}
