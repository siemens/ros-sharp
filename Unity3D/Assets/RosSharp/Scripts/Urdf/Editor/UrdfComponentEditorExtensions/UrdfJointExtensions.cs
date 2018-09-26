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
    public static class UrdfJointExtensions
    {
        private const float Tolerance = 0.0000001f;

        public static void Create(GameObject linkObject, UrdfJoint.JointTypes jointType, Joint joint = null)
        {
            Rigidbody parentRigidbody = linkObject.transform.parent.gameObject.GetComponent<Rigidbody>();
            if (parentRigidbody == null) return;

            UrdfJoint urdfJoint = AddCorrectJointType(linkObject, jointType);

            if (joint != null)
            {
                urdfJoint.JointName = joint.name;
                urdfJoint.ImportJointData(joint);
            }
        }

        private static UrdfJoint AddCorrectJointType(GameObject linkObject, UrdfJoint.JointTypes jointType)
        {
            UrdfJoint urdfJoint = null;

            switch (jointType)
            {
                case UrdfJoint.JointTypes.Fixed:
                    urdfJoint = UrdfJointFixed.Create(linkObject);
                    break;
                case UrdfJoint.JointTypes.Continuous:
                    urdfJoint = UrdfJointContinuous.Create(linkObject);
                    break;
                case UrdfJoint.JointTypes.Revolute:
                    urdfJoint = UrdfJointRevolute.Create(linkObject);
                    break;
                case UrdfJoint.JointTypes.Floating:
                    urdfJoint = UrdfJointFloating.Create(linkObject);
                    break;
                case UrdfJoint.JointTypes.Prismatic:
                    urdfJoint = UrdfJointPrismatic.Create(linkObject);
                    break;
                case UrdfJoint.JointTypes.Planar:
                    urdfJoint = UrdfJointPlanar.Create(linkObject);
                    break;
            }

            UnityEngine.Joint unityJoint = linkObject.GetComponent<UnityEngine.Joint>();
            unityJoint.connectedBody = linkObject.transform.parent.gameObject.GetComponent<Rigidbody>();
            unityJoint.autoConfigureConnectedAnchor = true;

            return urdfJoint;
        }
        public static void ChangeJointType(GameObject linkObject, UrdfJoint.JointTypes newJointType)
        {
            linkObject.transform.DestroyImmediateIfExists<UrdfJoint>();
            linkObject.transform.DestroyImmediateIfExists<HingeJointLimitsManager>();
            linkObject.transform.DestroyImmediateIfExists<PrismaticJointLimitsManager>();
            linkObject.transform.DestroyImmediateIfExists<UnityEngine.Joint>();

            AddCorrectJointType(linkObject, newJointType);
        }

        #region Import Helpers
        
        public static UrdfJoint.JointTypes GetJointType(string jointType)
        {
            switch (jointType)
            {
                case "fixed":
                    return UrdfJoint.JointTypes.Fixed;
                case "continuous":
                    return UrdfJoint.JointTypes.Continuous;
                case "revolute":
                    return UrdfJoint.JointTypes.Revolute;
                case "floating":
                    return UrdfJoint.JointTypes.Floating;
                case "prismatic":
                    return UrdfJoint.JointTypes.Prismatic;
                case "planar":
                    return UrdfJoint.JointTypes.Planar;
                default:
                    return UrdfJoint.JointTypes.Fixed;
            }
        }
        
        #endregion

        #region Export

        public static Joint ExportJointData(this UrdfJoint urdfJoint)
        {
            urdfJoint.UnityJoint = urdfJoint.GetComponent<UnityEngine.Joint>();

            urdfJoint.CheckForUrdfCompatibility();
            urdfJoint.GenerateUniqueJointName();

            //Data common to all joints
            Joint joint = new Joint(
                urdfJoint.JointName,
                urdfJoint.JointType.ToString().ToLower(),
                urdfJoint.transform.parent.name,
                urdfJoint.name,
                UrdfOrigin.ExportOriginData(urdfJoint.transform));

            joint.limit = urdfJoint.ExportLimitData();

            return urdfJoint.ExportSpecificJointData(joint);
        }

        public static Joint ExportDefaultJoint(Transform transform)
        {
            return new Joint(
                transform.parent.name + "_" + transform.name + "_joint",
                UrdfJoint.JointTypes.Fixed.ToString().ToLower(),
                transform.parent.name,
                transform.name,
                UrdfOrigin.ExportOriginData(transform));
        }

        #region ExportHelpers

        public static void GenerateUniqueJointName(this UrdfJoint urdfJoint)
        {
            urdfJoint.JointName = urdfJoint.transform.parent.name + "_" + urdfJoint.name + "_joint";
        }
        
        private static bool IsAnchorTransformed(this UrdfJoint urdfJoint)
        {
            UnityEngine.Joint joint = urdfJoint.GetComponent<UnityEngine.Joint>();

            return Math.Abs(joint.anchor.x) > Tolerance || 
                Math.Abs(joint.anchor.x) > Tolerance ||
                Math.Abs(joint.anchor.x) > Tolerance;
        }
        
        private static void CheckForUrdfCompatibility(this UrdfJoint urdfJoint)
        {
            if (!urdfJoint.AreLimitsCorrect())
                Debug.LogWarning("Limits are not defined correctly for Joint " + urdfJoint.JointName + " in Link " + urdfJoint.name +
                                 ". This may cause problems when visualizing the robot in RVIZ or Gazebo.",
                    urdfJoint.gameObject);
            if (!urdfJoint.IsJointAxisDefined())
                Debug.LogWarning("Axis for joint " + urdfJoint.JointName + " is undefined. Axis will not be written to URDF, " +
                                 "and the default axis will be used instead.",
                    urdfJoint.gameObject);
            if(urdfJoint.IsAnchorTransformed())
                Debug.LogWarning("The anchor position defined in the joint connected to " + urdfJoint.name + " will be" +
                                 " ignored in URDF. Instead of modifying anchor, change the position of the link.",
                    urdfJoint.gameObject);
        }

        #endregion

        #endregion
    }
}

