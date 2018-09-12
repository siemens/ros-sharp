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

        protected UnityEngine.Joint UnityJoint;
        public string JointName;

        public abstract JointTypes JointType { get; }
        public bool IsRevoluteOrContinuous => JointType == JointTypes.Revolute || JointType == JointTypes.Revolute;

        public double EffortLimit = double.PositiveInfinity;
        public double VelocityLimit = double.PositiveInfinity;

        protected const int RoundDigits = 6;
        protected const float Tolerance = 0.0000001f;

        public static void Create(GameObject linkObject, JointTypes jointType, Joint joint = null)
        {
            UrdfJoint urdfJoint = AddCorrectJointType(linkObject, jointType);

            if (joint != null)
            {
                urdfJoint.JointName = joint.name;
                urdfJoint.ImportJointData(joint);
            }
        }

        private static UrdfJoint AddCorrectJointType(GameObject linkObject, JointTypes jointType)
        {
            UrdfJoint urdfJoint = null;

            switch (jointType)
            {
                case JointTypes.Fixed:
                    urdfJoint = UrdfJointFixed.Create(linkObject);
                    break;
                case JointTypes.Continuous:
                    urdfJoint = UrdfJointContinuous.Create(linkObject);
                    break;
                case JointTypes.Revolute:
                    urdfJoint = UrdfJointRevolute.Create(linkObject);
                    break;
                case JointTypes.Floating:
                    urdfJoint = UrdfJointFloating.Create(linkObject);
                    break;
                case JointTypes.Prismatic:
                    urdfJoint = UrdfJointPrismatic.Create(linkObject);
                    break;
                case JointTypes.Planar:
                    urdfJoint = UrdfJointPlanar.Create(linkObject);
                    break;
            }

            UnityEngine.Joint unityJoint = linkObject.GetComponent<UnityEngine.Joint>();
            unityJoint.connectedBody = linkObject.transform.parent.gameObject.GetComponent<Rigidbody>();
            unityJoint.autoConfigureConnectedAnchor = true;

            return urdfJoint;
        }
        public static void ChangeJointType(GameObject linkObject, JointTypes newJointType)
        {
            linkObject.transform.DestroyImmediateIfExists<UrdfJoint>();
            linkObject.transform.DestroyImmediateIfExists<HingeJointLimitsManager>();
            linkObject.transform.DestroyImmediateIfExists<PrismaticJointLimitsManager>();
            linkObject.transform.DestroyImmediateIfExists<UnityEngine.Joint>();

            AddCorrectJointType(linkObject, newJointType);
        }

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
        
        public static JointTypes GetJointType(string jointType)
        {
            switch (jointType)
            {
                case "fixed":
                    return JointTypes.Fixed;
                case "continuous":
                    return JointTypes.Continuous;
                case "revolute":
                    return JointTypes.Revolute;
                case "floating":
                    return JointTypes.Floating;
                case "prismatic":
                    return JointTypes.Prismatic;
                case "planar":
                    return JointTypes.Planar;
                default:
                    return JointTypes.Fixed;
            }
        }

        protected virtual void ImportJointData(Joint joint) { }
        
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
                positionDamper = (float) dynamics.damping,
                positionSpring = (float) dynamics.friction
            };
        }

        protected static JointSpring GetJointSpring(Joint.Dynamics dynamics)
        {
            return new JointSpring
            {
                damper = (float) dynamics.damping,
                spring = (float) dynamics.friction,
                targetPosition = 0
            };
        }

        protected static SoftJointLimit GetLinearLimit(Joint.Limit limit)
        {
            return new SoftJointLimit { limit = (float) limit.upper };
        }

        #endregion

        #region Export

        public Joint ExportJointData()
        {
            UnityJoint = GetComponent<UnityEngine.Joint>();

            CheckForUrdfCompatibility();
            GenerateUniqueJointName();

            //Data common to all joints
            Joint joint = new Joint(
                JointName,
                JointType.ToString().ToLower(),
                gameObject.transform.parent.name,
                gameObject.name,
                UrdfOrigin.ExportOriginData(transform));

            joint.limit = ExportLimitData();

            return ExportSpecificJointData(joint);
        }

        #region ExportHelpers

        protected virtual Joint ExportSpecificJointData(Joint joint)
        {
            return joint;
        }

        protected virtual Joint.Limit ExportLimitData()
        {
            return null; // limits aren't used
        }

        public virtual bool AreLimitsCorrect()
        {
            return true; // limits aren't needed
        }

        protected virtual bool IsJointAxisDefined()
        {
            UnityEngine.Joint joint = GetComponent<UnityEngine.Joint>();
            return !(Math.Abs(joint.axis.x) < Tolerance &&
                     Math.Abs(joint.axis.y) < Tolerance &&
                     Math.Abs(joint.axis.z) < Tolerance);
        }

        private void GenerateUniqueJointName()
        {
            JointName = transform.parent.name + "_" + transform.name + "_joint";
        }

        protected Joint.Axis GetAxisData(Vector3 axis)
        {
            double[] rosAxis = axis.Unity2Ros().ToRoundedDoubleArray();
            return new Joint.Axis(rosAxis);
        }
        
        private bool IsAnchorTransformed()
        {
            UnityEngine.Joint joint = GetComponent<UnityEngine.Joint>();

            return Math.Abs(joint.anchor.x) > Tolerance || 
                Math.Abs(joint.anchor.x) > Tolerance ||
                Math.Abs(joint.anchor.x) > Tolerance;
        }
        
        private void CheckForUrdfCompatibility()
        {
            if (!AreLimitsCorrect())
                Debug.LogWarning("Limits are not defined correctly for Joint " + JointName + " in Link " + name +
                                 ". This may cause problems when visualizing the robot in RVIZ or Gazebo.", 
                                 gameObject);
            if (!IsJointAxisDefined())
                Debug.LogWarning("Axis for joint " + JointName + " is undefined. Axis will not be written to URDF, " +
                                 "and the default axis will be used instead.", 
                                 gameObject);
            if(IsAnchorTransformed())
                Debug.LogWarning("The anchor position defined in the joint connected to " + name + " will be" +
                                 " ignored in URDF. Instead of modifying anchor, change the position of the link.", 
                                 gameObject);
        }

        #endregion

        #endregion
    }
}

