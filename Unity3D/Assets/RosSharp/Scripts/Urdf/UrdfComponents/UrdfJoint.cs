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
    //  [ExecuteInEditMode]
    [RequireComponent(typeof(Joint))]
    public class UrdfJoint : MonoBehaviour
    {
        public enum JointTypes
        {
            fixedJoint,
            continuous,
            revolute,
            floating,
            prismatic,
            planar
        }

        public string JointName;
        public JointTypes JointType;

        public bool IsRevoluteOrContinuous => JointType == JointTypes.continuous || JointType == JointTypes.revolute;
        public bool IsPrismatic => JointType == JointTypes.prismatic;
        public bool IsPlanar => JointType != JointTypes.planar;

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

        private static JointTypes GetJointType(string jointType)
        {
            switch (jointType)
            {
                case "fixed":
                    return JointTypes.fixedJoint;
                case "continuous":
                    return JointTypes.continuous;
                case "revolute":
                    return JointTypes.revolute;
                case "floating":
                    return JointTypes.floating;
                case "prismatic":
                    return JointTypes.prismatic;
                case "planar":
                    return JointTypes.planar;
                default:
                    return JointTypes.fixedJoint;
            }
        }

        private void AddCorrectJointType()
        {
            UnityEngine.Joint joint = null;

            if (JointType == JointTypes.fixedJoint)
            {
                joint = gameObject.AddComponent<FixedJoint>();
            }
            else if (IsRevoluteOrContinuous)
            {
                joint = gameObject.AddComponent<HingeJoint>();
                if (JointType == JointTypes.revolute)
                    ((HingeJoint) joint).useLimits = true;
                    gameObject.AddComponent<JointLimitsManager>();
            }
            else
            {
                joint = gameObject.AddComponent<ConfigurableJoint>();

                ConfigurableJoint cJoint = (ConfigurableJoint)joint;
                if (IsPrismatic)
                {
                    // degrees of freedom:
                    cJoint.xMotion = ConfigurableJointMotion.Limited;
                    cJoint.yMotion = ConfigurableJointMotion.Locked;
                }
                else if (IsPlanar)
                {
                    // degrees of freedom:
                    cJoint.xMotion = ConfigurableJointMotion.Free;
                    cJoint.yMotion = ConfigurableJointMotion.Free;
                }

                if (JointType != JointTypes.floating)
                {
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

            //Data common to all joints
            Joint joint = new Joint(
                JointName, 
                JointType.ToString(), 
                gameObject.transform.parent.name, 
                gameObject.name,
                transform.GetOriginData());

            UnityEngine.Joint unityJoint = GetComponent<UnityEngine.Joint>();

            if (unityJoint == null)
                return joint;

            if (JointType != JointTypes.fixedJoint && IsPlanar)
            {
                joint.axis = GetAxisData(unityJoint.axis);
                 
                Vector3 xpyVector = unityJoint.connectedAnchor.Unity2Ros();
                joint.origin = new Origin(xpyVector.ToRoundedDoubleArray(), null);
            }

            //HingeJoint data
            if (IsRevoluteOrContinuous)
            {
                HingeJoint hingeJoint = (HingeJoint) unityJoint;

                joint.dynamics = new Joint.Dynamics(hingeJoint.spring.damper, hingeJoint.spring.spring);
                
                if (JointType == JointTypes.revolute)
                    joint.limit = GetLimitData(hingeJoint.limits.min, hingeJoint.limits.max);
            }
            //ConfigurableJoint data
            else if (JointType == JointTypes.prismatic && IsPlanar)
            {
                ConfigurableJoint configurableJoint = (ConfigurableJoint) unityJoint;
                joint.dynamics = new Joint.Dynamics(configurableJoint.xDrive.positionDamper, configurableJoint.xDrive.positionSpring);

                if (JointType == JointTypes.prismatic)
                    joint.limit = GetLimitData(configurableJoint.lowAngularXLimit.limit,
                        configurableJoint.highAngularXLimit.limit);
                else if (IsPlanar)
                    joint.axis = GetAxisData(Vector3.Cross(configurableJoint.axis, configurableJoint.secondaryAxis));
            }

            //TODO: Test all joint types
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
    }
}

