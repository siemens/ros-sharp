/*
© Siemens AG, 2018-2019
Author: Suzannah Smith (suzannah.smith@siemens.com)

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
    public class UrdfJointPrismatic : UrdfJoint
    {
        public override JointTypes JointType => JointTypes.Prismatic;

        public static UrdfJoint Create(GameObject linkObject)
        {
            UrdfJointPrismatic urdfJoint = linkObject.AddComponent<UrdfJointPrismatic>();

            urdfJoint.UnityJoint = linkObject.AddComponent<ConfigurableJoint>();
            urdfJoint.UnityJoint.autoConfigureConnectedAnchor = true;

            ConfigurableJoint configurableJoint = (ConfigurableJoint) urdfJoint.UnityJoint;

            // degrees of freedom:
            configurableJoint.xMotion = ConfigurableJointMotion.Limited;
            configurableJoint.yMotion = ConfigurableJointMotion.Locked;
            configurableJoint.zMotion = ConfigurableJointMotion.Locked;
            configurableJoint.angularXMotion = ConfigurableJointMotion.Locked;
            configurableJoint.angularYMotion = ConfigurableJointMotion.Locked;
            configurableJoint.angularZMotion = ConfigurableJointMotion.Locked;

            linkObject.AddComponent<PrismaticJointLimitsManager>();

            return urdfJoint;
        }

        #region Runtime

        public override float GetPosition()
        {
            return Vector3.Dot(UnityJoint.transform.localPosition - UnityJoint.connectedAnchor, UnityJoint.axis);
        }

        protected override void OnUpdateJointState(float deltaState)
        {
            transform.Translate(UnityJoint.axis * deltaState);
        }

        #endregion

        #region Import

        protected override void ImportJointData(Joint joint)
        {
            ConfigurableJoint prismaticJoint = (ConfigurableJoint) UnityJoint;
            prismaticJoint.axis = (joint.axis != null) ? GetAxis(joint.axis) : GetDefaultAxis();

            if (joint.dynamics != null)
                prismaticJoint.xDrive = GetJointDrive(joint.dynamics);

            if (joint.limit != null)
            {
                PrismaticJointLimitsManager prismaticLimits = GetComponent<PrismaticJointLimitsManager>();
                prismaticLimits.InitializeLimits(joint.limit);
            }
        }

        #endregion

        #region Export

        protected override Joint ExportSpecificJointData(Joint joint)
        {
            ConfigurableJoint configurableJoint = (ConfigurableJoint)UnityJoint;

            joint.axis = GetAxisData(configurableJoint.axis);
            joint.dynamics = new Joint.Dynamics(configurableJoint.xDrive.positionDamper, configurableJoint.xDrive.positionSpring);
            joint.limit = ExportLimitData();

            return joint;
        }

        public override bool AreLimitsCorrect()
        {
            PrismaticJointLimitsManager limits = GetComponent<PrismaticJointLimitsManager>();
            return limits != null && limits.PositionLimitMin < limits.PositionLimitMax;
        }

        protected override Joint.Limit ExportLimitData()
        {
            PrismaticJointLimitsManager prismaticLimits = GetComponent<PrismaticJointLimitsManager>();
            return new Joint.Limit(
                Math.Round(prismaticLimits.PositionLimitMin, RoundDigits),
                Math.Round(prismaticLimits.PositionLimitMax, RoundDigits),
                EffortLimit,
                VelocityLimit);
        }

        #endregion
    }
}

