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

        private void SetJointData(Joint joint)
        {
            if (IsRevoluteOrContinuous)
                ConfigureHingeJoint(joint);
            else if (IsPrismatic)
                ConfigurePrismaticJoint(joint);
            else if(JointType == JointTypes.Floating)
                ConfigureFloatingJoint(joint);
            else if (IsPlanar)
                ConfigurePlanarJoint(joint);
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
                    ((HingeJoint) unityJoint).useLimits = true;
                    gameObject.AddComponent<JointLimitsManager>();
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

            if (unityJoint != null) unityJoint.connectedBody = gameObject.transform.parent.gameObject.GetComponent<Rigidbody>();
        }

        public Joint GetJointData()
        {
            CheckForUrdfCompatibility();

            //Data common to all joints
            Joint joint = new Joint(
                JointName,
                GetJointTypeName(JointType),
                gameObject.transform.parent.name, 
                gameObject.name,
                transform.GetOriginData());

            UnityEngine.Joint unityJoint = GetComponent<UnityEngine.Joint>();

            if (unityJoint == null)
                return joint;

            if (IsPlanar)
            {
                ConfigurableJoint cJoint = GetComponent<ConfigurableJoint>();
                joint.axis = GetAxisData(Vector3.Cross(cJoint.axis, cJoint.secondaryAxis));
            }
            else if (JointType != JointTypes.Fixed)
            {
                joint.axis = GetAxisData(unityJoint.axis);
                 
                //TODO: how to combine info from connectedAnchor and the link's transform origin? Which
                //is more important? Don't allow user to change connectedAnchor? 
                //Currently if connectedAnchor is stored in joint.origin, is overrides the link's origin
                //Vector3 xpyVector = unityJoint.connectedAnchor.Unity2Ros();
                //joint.origin = new Origin(xpyVector.ToRoundedDoubleArray(), null);
            }

            //HingeJoint data
            if (IsRevoluteOrContinuous)
            {
                HingeJoint hingeJoint = (HingeJoint) unityJoint;

                joint.dynamics = new Joint.Dynamics(hingeJoint.spring.damper, hingeJoint.spring.spring);
                
                if (JointType == JointTypes.Revolute)
                    joint.limit = GetLimitData(hingeJoint.limits.min, hingeJoint.limits.max);
            }
            //ConfigurableJoint data
            else if (IsPrismatic || IsPlanar)
            {
                ConfigurableJoint configurableJoint = GetComponent<ConfigurableJoint>();
                joint.dynamics = new Joint.Dynamics(configurableJoint.xDrive.positionDamper, configurableJoint.xDrive.positionSpring);
                
                //linear limit, not angular
                joint.limit = new Joint.Limit(
                    -configurableJoint.linearLimit.limit, 
                    configurableJoint.linearLimit.limit, 
                    effortLimit, velocityLimit);
               }
            
            //TODO: Get Floating and Planar joints working
            return joint;
        }

        private HingeJoint ConfigureHingeJoint(Joint joint)
        {
            HingeJoint hingeJoint = GetComponent<HingeJoint>();
            
            // axis:
            hingeJoint.axis = (joint.axis != null) ? GetAxis(joint.axis) : GetDefaultAxis();

            // origin:
            hingeJoint.autoConfigureConnectedAnchor = false;
            hingeJoint.connectedAnchor = GetConnectedAnchor(joint);

            // spring, damper & position:
            if (joint.dynamics != null)
                hingeJoint.spring = GetJointSpring(joint.dynamics);

            // limits:        
            if (joint.type == "revolute" && joint.limit != null)
            {
                hingeJoint.limits = GetJointLimits(joint.limit);

                // large joint limits:
                if (hingeJoint.limits.min < -180 || hingeJoint.limits.max > 180)
                {
                    JointLimitsManager jointLimitsManager = gameObject.AddComponent<JointLimitsManager>();
                    jointLimitsManager.LargeAngleLimitMin = hingeJoint.limits.min;
                    jointLimitsManager.LargeAngleLimitMax = hingeJoint.limits.max;
                    JointLimits jointLimits = hingeJoint.limits;
                    jointLimits.min = -180;
                    jointLimits.max = +180;
                    hingeJoint.limits = jointLimits;
                }
                else
                    hingeJoint.useLimits = true;
            }
            
            return hingeJoint;
        }

        private void ConfigureFloatingJoint(Joint joint)
        {
            ConfigurableJoint floatingJoint = GetComponent<ConfigurableJoint>();
            // origin:
            floatingJoint.autoConfigureConnectedAnchor = false;
            floatingJoint.connectedAnchor = GetConnectedAnchor(joint);
        }

        private void ConfigurePrismaticJoint(Joint joint)
        {
            ConfigurableJoint prismaticJoint = GetComponent<ConfigurableJoint>();

            prismaticJoint.axis = (joint.axis != null) ? GetAxis(joint.axis) : GetDefaultAxis();

            // origin:
            prismaticJoint.autoConfigureConnectedAnchor = false;
            prismaticJoint.connectedAnchor = GetConnectedAnchor(joint);

            // spring, damper & max. force:
            if (joint.dynamics != null)
                prismaticJoint.xDrive = GetJointDrive(joint.dynamics);

            // limits:
            if (joint.limit != null)
            {
                prismaticJoint.lowAngularXLimit = GetLowSoftJointLimit(joint.limit);
                prismaticJoint.highAngularXLimit = GetHighSoftJointLimit(joint.limit);

                prismaticJoint.linearLimit = GetLinearLimit(joint.limit);
                //Todo: use lower limit as well? GetLinearLimit only uses upper.
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

            // origin:
            planarJoint.autoConfigureConnectedAnchor = false;
            planarJoint.connectedAnchor = GetConnectedAnchor(joint);

            // spring, damper & max. force:
            if (joint.dynamics != null)
            {
                planarJoint.xDrive = GetJointDrive(joint.dynamics);
                planarJoint.yDrive = GetJointDrive(joint.dynamics);
            }
        }

        #region ImportHelpers


        public static Vector3 GetAxis(Joint.Axis axis)
        {
            return new Vector3(
                (float)-axis.xyz[1],
                (float)axis.xyz[2],
                (float)axis.xyz[0]);
        }
        public static Vector3 GetDefaultAxis()
        {
            return new Vector3(-1, 0, 0);
        }

        public static JointDrive GetJointDrive(Joint.Dynamics dynamics)
        {
            return new JointDrive
            {
                maximumForce = float.MaxValue,
                positionDamper = (float) dynamics.damping,
                positionSpring = (float) dynamics.friction
            };
        }
        public static JointSpring GetJointSpring(Joint.Dynamics dynamics)
        {
            return new JointSpring
            {
                damper = (float) dynamics.damping,
                spring = (float) dynamics.friction,
                targetPosition = 0
            };
        }
        
        public static JointLimits GetJointLimits(Joint.Limit limit)
        {
            JointLimits jointLimits = new JointLimits();
            jointLimits.min = (float)limit.lower * Mathf.Rad2Deg;
            jointLimits.max = (float)limit.upper * Mathf.Rad2Deg;
            return jointLimits;
        }

        public static SoftJointLimit GetLowSoftJointLimit(Joint.Limit limit)
        {
            SoftJointLimit softJointLimit = new SoftJointLimit();
            softJointLimit.limit = (float)limit.lower * Mathf.Rad2Deg;
            return softJointLimit;
        }
        public static SoftJointLimit GetHighSoftJointLimit(Joint.Limit limit)
        {
            SoftJointLimit softJointLimit = new SoftJointLimit();
            softJointLimit.limit = (float)limit.upper * Mathf.Rad2Deg;
            return softJointLimit;
        }

        public static SoftJointLimit GetLinearLimit(Joint.Limit limit)
        {
            SoftJointLimit softJointLimit = new SoftJointLimit();
            softJointLimit.limit = (float)limit.upper;
            return softJointLimit;
        }

        public static Vector3 GetConnectedAnchor(Joint joint)
        {
            if (joint.origin != null)
                return UrdfOrigin.GetPosition(joint.origin); // todo: where to put rotation (if it exists in URDF)?
            else
                return Vector3.zero;
        }
        #endregion

        #region ExportHelpers

        private Joint.Limit GetLimitData(float min, float max)
        {
            return new Joint.Limit(min * Mathf.Deg2Rad, max * Mathf.Deg2Rad, effortLimit, velocityLimit);
        }

        private Joint.Axis GetAxisData(Vector3 axis)
        {
            Vector3 rosVector = axis.Unity2Ros();
            return new Joint.Axis(new double[] {
                rosVector.x,
                rosVector.y,
                rosVector.z});
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
                return !(joint.axis.x == 0 && joint.axis.y == 0 && joint.axis.z == 0);
            }
            if (IsPlanar)
            {
                ConfigurableJoint joint = GetComponent<ConfigurableJoint>();
                return !(joint.axis.x == 0 && joint.axis.y == 0 && joint.axis.z == 0)
                       && !(joint.secondaryAxis.x == 0 && joint.secondaryAxis.y == 0 && joint.secondaryAxis.z == 0);
            }

            return true; // axis isn't needed
        }

        private void CheckForUrdfCompatibility()
        {
            if (!AreLimitsCorrect())
            {
                Debug.LogWarning("Limits are not defined correctly for Joint " + JointName + " in Link " + name +
                                 ". This may cause problems when visualizing the robot in RVIZ or Gazebo.",
                                 gameObject);
            }
            if (!IsJointAxisDefined())
            {
                Debug.LogWarning("Axis for joint " + JointName + " is undefined. Axis will not be written to URDF, and the default axis will be used instead.", gameObject);
            }
        }

        #endregion

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

        private static string GetJointTypeName(JointTypes jointType)
        {
            return jointType.ToString().ToLower();
        }

        public void ChangeJointType(JointTypes newJointType)
        {
            JointType = newJointType;

            //TODO: Fix "object is destroyed but still trying to reference it" error
            transform.DestroyImmediateIfExists<JointLimitsManager>();
            transform.DestroyImmediateIfExists<UnityEngine.Joint>();

            AddCorrectJointType();
        }
    }
}

