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
    public class UrdfJoint : MonoBehaviour
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
       
        public string JointName;
        public JointTypes JointType;

        public bool IsRevoluteOrContinuous => JointType == JointTypes.Continuous || JointType == JointTypes.Revolute;
        public bool IsPrismatic => JointType == JointTypes.Prismatic;
        public bool IsPlanar => JointType == JointTypes.Planar;

        //TODO: figure out better default limits. Or how to get info from Unity joint
        public double effortLimit = 50000;
        public double velocityLimit = 10000;

        private const int RoundDigits = 4;
        private const float Tolerance = 0.0000001f;

        public static void Create(GameObject linkObject, string jointName, JointTypes jointType)
        {
            UrdfJoint urdfJoint = linkObject.AddComponent<UrdfJoint>();
            urdfJoint.JointName = jointName;
            urdfJoint.JointType = jointType;

            urdfJoint.AddCorrectJointType();
        }

        public static void Create(GameObject linkObject, Joint joint)
        {
            UrdfJoint urdfJoint = linkObject.AddComponent<UrdfJoint>();
            urdfJoint.JointName = joint.name;
            urdfJoint.JointType = GetJointType(joint.type);

            urdfJoint.AddCorrectJointType();

            urdfJoint.SetJointData(joint);
        }
        
        private static JointTypes GetJointType(string jointType)
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

        private static string GetJointTypeName(JointTypes jointType)
        {
            return jointType.ToString().ToLower();
        }

        public void ChangeJointType(JointTypes newJointType)
        {
            JointType = newJointType;
            
            transform.DestroyImmediateIfExists<HingeJointLimitsManager>();
            transform.DestroyImmediateIfExists<PrismaticJointLimitsManager>();
            transform.DestroyImmediateIfExists<UnityEngine.Joint>();

            AddCorrectJointType();
        }

        public void GenerateUniqueJointName()
        {
            JointName = transform.parent.name + "_" + transform.name + "_joint";
        }

        private void AddCorrectJointType()
        {
            UnityEngine.Joint unityJoint = null;

            if (JointType == JointTypes.Fixed)
            {
                unityJoint = gameObject.AddComponent<FixedJoint>();
            }
            else if (IsRevoluteOrContinuous)
            {
                unityJoint = gameObject.AddComponent<HingeJoint>();
                unityJoint.autoConfigureConnectedAnchor = true;

                if (JointType == JointTypes.Revolute)
                {
                    ((HingeJoint)unityJoint).useLimits = true;
                    gameObject.AddComponent<HingeJointLimitsManager>();
                }
            }
            else
            {
                unityJoint = gameObject.AddComponent<ConfigurableJoint>();
                unityJoint.autoConfigureConnectedAnchor = true;

                ConfigurableJoint cJoint = (ConfigurableJoint)unityJoint;
                if (IsPrismatic)
                {
                    // degrees of freedom:
                    cJoint.xMotion = ConfigurableJointMotion.Limited;
                    cJoint.yMotion = ConfigurableJointMotion.Locked;
                    cJoint.zMotion = ConfigurableJointMotion.Locked;
                    cJoint.angularXMotion = ConfigurableJointMotion.Locked;
                    cJoint.angularYMotion = ConfigurableJointMotion.Locked;
                    cJoint.angularZMotion = ConfigurableJointMotion.Locked;

                    gameObject.AddComponent<PrismaticJointLimitsManager>();
                }
                else if (IsPlanar)
                {
                    // degrees of freedom:
                    cJoint.xMotion = ConfigurableJointMotion.Free;
                    cJoint.yMotion = ConfigurableJointMotion.Free;
                    cJoint.zMotion = ConfigurableJointMotion.Locked;
                    cJoint.angularXMotion = ConfigurableJointMotion.Locked;
                    cJoint.angularYMotion = ConfigurableJointMotion.Locked;
                    cJoint.angularZMotion = ConfigurableJointMotion.Locked;
                }
            }

            if (unityJoint != null)
                unityJoint.connectedBody = gameObject.transform.parent.gameObject.GetComponent<Rigidbody>();
        }


        #region SetJointData

        private void SetJointData(Joint joint)
        {
            if (IsRevoluteOrContinuous)
                ConfigureRevoluteOrContinuousJoint(joint);
            else if (IsPrismatic)
                ConfigurePrismaticJoint(joint);
            else if (IsPlanar)
                ConfigurePlanarJoint(joint);
        }

        private void ConfigureRevoluteOrContinuousJoint(Joint joint)
        {
            HingeJoint hingeJoint = GetComponent<HingeJoint>();
            hingeJoint.axis = (joint.axis != null) ? GetAxis(joint.axis) : GetDefaultAxis();

            if (joint.dynamics != null)
                hingeJoint.spring = GetJointSpring(joint.dynamics);

            if (JointType == JointTypes.Revolute && joint.limit != null)
            {
                HingeJointLimitsManager hingeJointLimitsManager = GetComponent<HingeJointLimitsManager>();
                hingeJointLimitsManager.InitializeLimits(joint.limit, hingeJoint);
            }
        }

        private void ConfigurePrismaticJoint(Joint joint)
        {
            ConfigurableJoint prismaticJoint = GetComponent<ConfigurableJoint>();
            prismaticJoint.axis = (joint.axis != null) ? GetAxis(joint.axis) : GetDefaultAxis();

            if (joint.dynamics != null)
                prismaticJoint.xDrive = GetJointDrive(joint.dynamics);

            if (joint.limit != null)
            {
                PrismaticJointLimitsManager prismaticLimits = GetComponent<PrismaticJointLimitsManager>();
                prismaticLimits.InitializeLimits(joint.limit);
            }
        }

        private void ConfigurePlanarJoint(Joint joint)
        {
            ConfigurableJoint planarJoint = GetComponent<ConfigurableJoint>();

            Vector3 normal = (joint.axis != null) ? GetAxis(joint.axis) : GetDefaultAxis();
            Vector3 axisX = Vector3.forward;
            Vector3 axisY = Vector3.left;
            Vector3.OrthoNormalize(ref normal, ref axisX, ref axisY);
            planarJoint.axis = axisX;
            planarJoint.secondaryAxis = axisY;
            
            // spring, damper & max. force:
            if (joint.dynamics != null)
            {
                planarJoint.xDrive = GetJointDrive(joint.dynamics);
                planarJoint.yDrive = GetJointDrive(joint.dynamics);
            }
        }

        
        #region ImportHelpers

        private static Vector3 GetAxis(Joint.Axis axis)
        {
            return axis.xyz.ToVector3().Ros2Unity();
        }

        private static Vector3 GetDefaultAxis()
        {
            return new Vector3(-1, 0, 0);
        }

        private static JointDrive GetJointDrive(Joint.Dynamics dynamics)
        {
            return new JointDrive
            {
                maximumForce = float.MaxValue,
                positionDamper = (float) dynamics.damping,
                positionSpring = (float) dynamics.friction
            };
        }

        private static JointSpring GetJointSpring(Joint.Dynamics dynamics)
        {
            return new JointSpring
            {
                damper = (float) dynamics.damping,
                spring = (float) dynamics.friction,
                targetPosition = 0
            };
        }

        private static JointLimits GetJointLimits(Joint.Limit limit)
        {
            return new JointLimits
            {
                min = (float) limit.lower * Mathf.Rad2Deg,
                max = (float) limit.upper * Mathf.Rad2Deg
            };
            
        }

        private static SoftJointLimit GetLinearLimit(Joint.Limit limit)
        {
            return new SoftJointLimit
            {
                limit = (float) limit.upper
            };
        }

        private static Vector3 GetConnectedAnchor(Joint joint)
        {
            return joint.origin == null ? Vector3.zero : UrdfOrigin.GetPosition(joint.origin);
        }

        #endregion

        #endregion

        #region GetJointData

        public Joint GetJointData()
        {
            CheckForUrdfCompatibility();

            //Data common to all joints
            Joint joint = new Joint(
                JointName,
                GetJointTypeName(JointType),
                gameObject.transform.parent.name,
                gameObject.name,
                UrdfOrigin.GetOriginData(transform));

            UnityEngine.Joint unityJoint = GetComponent<UnityEngine.Joint>();

            if (unityJoint == null)
                return joint;

            if (IsPlanar)
            {
                ConfigurableJoint cJoint = GetComponent<ConfigurableJoint>();
                joint.axis = GetAxisData(Vector3.Cross(cJoint.axis, cJoint.secondaryAxis));
            }
            else if (JointType != JointTypes.Fixed)
                joint.axis = GetAxisData(unityJoint.axis);

            //HingeJoint data
            if (IsRevoluteOrContinuous)
            {
                HingeJoint hingeJoint = (HingeJoint)unityJoint;
                joint.dynamics = new Joint.Dynamics(hingeJoint.spring.damper, hingeJoint.spring.spring);
                
            }
            //ConfigurableJoint data
            else if (IsPrismatic || IsPlanar)
            {
                ConfigurableJoint configurableJoint = GetComponent<ConfigurableJoint>();
                joint.dynamics = new Joint.Dynamics(configurableJoint.xDrive.positionDamper, configurableJoint.xDrive.positionSpring);
            }

            joint.limit = GetLimitData();
            
            return joint;
        }

        #region ExportHelpers

        private Joint.Limit GetLimitData()
        {
            if (IsPrismatic)
            {
                PrismaticJointLimitsManager prismaticLimits = GetComponent<PrismaticJointLimitsManager>();
                return new Joint.Limit(
                    Math.Round(prismaticLimits.PositionLimitMin, RoundDigits),
                    Math.Round(prismaticLimits.PositionLimitMax, RoundDigits),
                    effortLimit,
                    velocityLimit);
            }
            if (JointType == JointTypes.Revolute)
            {
                HingeJointLimitsManager hingeJointLimits = GetComponent<HingeJointLimitsManager>();
                return new Joint.Limit(
                    Math.Round(hingeJointLimits.LargeAngleLimitMin * Mathf.Deg2Rad, RoundDigits), 
                    Math.Round(hingeJointLimits.LargeAngleLimitMax * Mathf.Deg2Rad, RoundDigits), 
                    effortLimit, 
                    velocityLimit);
            }
            if (IsPlanar)
            {
                ConfigurableJoint configurableJoint = GetComponent<ConfigurableJoint>();
                return new Joint.Limit(
                    Math.Round(-configurableJoint.linearLimit.limit, RoundDigits),
                    Math.Round(configurableJoint.linearLimit.limit, RoundDigits),
                    effortLimit, velocityLimit);
            }

            return null;
        }

        private Joint.Axis GetAxisData(Vector3 axis)
        {
            double[] rosAxis = axis.Unity2Ros().ToRoundedDoubleArray();
            return new Joint.Axis(rosAxis);
        }

        public bool AreLimitsCorrect()
        {
            if (IsPrismatic || IsPlanar)
            {
                ConfigurableJoint joint = GetComponent<ConfigurableJoint>();
                return joint != null && joint.linearLimit.limit != 0;
            }
            if (JointType == JointTypes.Revolute)
            {
                HingeJoint joint = GetComponent<HingeJoint>();
                return joint != null && joint.useLimits && joint.limits.max > joint.limits.min;
            }

            return true; //limits aren't needed
        }

        private bool IsJointAxisDefined()
        {
            if (IsRevoluteOrContinuous || IsPrismatic)
            {
                UnityEngine.Joint joint = GetComponent<UnityEngine.Joint>();
                return !(Math.Abs(joint.axis.x) < Tolerance && 
                         Math.Abs(joint.axis.y) < Tolerance && 
                         Math.Abs(joint.axis.z) < Tolerance);
            }
            if (IsPlanar)
            {
                ConfigurableJoint joint = GetComponent<ConfigurableJoint>();
                return !(Math.Abs(joint.axis.x) < Tolerance &&
                         Math.Abs(joint.axis.y) < Tolerance &&
                         Math.Abs(joint.axis.z) < Tolerance)
                       && !(Math.Abs(joint.secondaryAxis.x) < Tolerance &&
                            Math.Abs(joint.secondaryAxis.y) < Tolerance &&
                            Math.Abs(joint.secondaryAxis.z) < Tolerance);
            }

            return true; // axis isn't needed
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

