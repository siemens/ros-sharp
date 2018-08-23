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

using UnityEditor;
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

        public void Initialize(string jointName, string jointType)
        {
            JointName = jointName;
            JointType = GetJointType(jointType);

            AddCorrectJointType();

            //TODO: Test all joint types
        }

        private void AddCorrectJointType()
        {
            UnityEngine.Joint joint = null;

            if (JointType == JointTypes.Fixed)
            {
                joint = gameObject.AddComponent<FixedJoint>();
            }
            else if (IsRevoluteOrContinuous)
            {
                joint = gameObject.AddComponent<HingeJoint>();
                joint.autoConfigureConnectedAnchor = true;

                if (JointType == JointTypes.Revolute)
                    ((HingeJoint) joint).useLimits = true;
                    gameObject.AddComponent<JointLimitsManager>();
            }
            else
            {
                joint = gameObject.AddComponent<ConfigurableJoint>();
                joint.autoConfigureConnectedAnchor = true;

                ConfigurableJoint cJoint = (ConfigurableJoint)joint;
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

            if (joint != null) joint.connectedBody = gameObject.transform.parent.gameObject.GetComponent<Rigidbody>();
        }

        public Joint GetJointData()
        {
            //TODO: consider only getting dynamics data if "Use Spring" is true
            //TODO: consider only getting limits data if "Use Limits" is true
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

            //TODO: Test all joint types
            //TODO: Get Floating and Planar joints working
            return joint;
        }

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

        public bool IsJointAxisDefined()
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
    }
}

