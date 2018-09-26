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

using System;
using UnityEngine;

namespace RosSharp.Urdf
{
    [RequireComponent(typeof(Joint))]
    public abstract class UrdfJoint : MonoBehaviour
    {
        public enum JointTypes
        {
            Fixed,
            Continuous,
            Revolute,
            Floating,
            Prismatic,
            Planar
        }

        public UnityEngine.Joint UnityJoint;
        public string JointName;

        public abstract JointTypes JointType { get; }
        public bool IsRevoluteOrContinuous => JointType == JointTypes.Revolute || JointType == JointTypes.Revolute;

        public double EffortLimit = double.PositiveInfinity;
        public double VelocityLimit = double.PositiveInfinity;

        protected const int RoundDigits = 6;
        protected const float Tolerance = 0.0000001f;

        #region Runtime

        public void Start()
        {
            UnityJoint = GetComponent<UnityEngine.Joint>();
        }
        public virtual float GetPosition()
        {
            return 0;
        }
        public virtual float GetVelocity()
        {
            return 0;
        }
        public virtual float GetEffort()
        {
            return 0;
        }

        public void UpdateJointState(float deltaState)
        {
            OnUpdateJointState(deltaState);
        }

        protected virtual void OnUpdateJointState(float deltaState) { }

        #endregion

        #region Import Helpers
        
        public virtual void ImportJointData(Joint joint) { }

        protected static Vector3 GetAxis(Joint.Axis axis)
        {
            return axis.xyz.ToVector3().Ros2Unity();
        }

        protected static Vector3 GetDefaultAxis()
        {
            return new Vector3(-1, 0, 0);
        }

        protected static JointDrive GetJointDrive(Joint.Dynamics dynamics)
        {
            return new JointDrive
            {
                maximumForce = float.MaxValue,
                positionDamper = (float)dynamics.damping,
                positionSpring = (float)dynamics.friction
            };
        }

        protected static JointSpring GetJointSpring(Joint.Dynamics dynamics)
        {
            return new JointSpring
            {
                damper = (float)dynamics.damping,
                spring = (float)dynamics.friction,
                targetPosition = 0
            };
        }

        protected static SoftJointLimit GetLinearLimit(Joint.Limit limit)
        {
            return new SoftJointLimit { limit = (float)limit.upper };
        }

        #endregion


        #region ExportHelpers

        public virtual Joint ExportSpecificJointData(Joint joint)
        {
            return joint;
        }

        public virtual Joint.Limit ExportLimitData()
        {
            return null; // limits aren't used
        }

        public virtual bool AreLimitsCorrect()
        {
            return true; // limits aren't needed
        }

        public virtual bool IsJointAxisDefined()
        {
            UnityEngine.Joint joint = GetComponent<UnityEngine.Joint>();
            return !(Math.Abs(joint.axis.x) < Tolerance &&
                     Math.Abs(joint.axis.y) < Tolerance &&
                     Math.Abs(joint.axis.z) < Tolerance);
        }

        protected Joint.Axis GetAxisData(Vector3 axis)
        {
            double[] rosAxis = axis.Unity2Ros().ToRoundedDoubleArray();
            return new Joint.Axis(rosAxis);
        }

        #endregion

    }
}

