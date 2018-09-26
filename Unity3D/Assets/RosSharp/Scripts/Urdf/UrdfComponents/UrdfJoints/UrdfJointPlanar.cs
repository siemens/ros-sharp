/*
© Siemens AG, 2018
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
    public class UrdfJointPlanar : UrdfJoint
    {
        public override JointTypes JointType => JointTypes.Planar;

        public static UrdfJoint Create(GameObject linkObject)
        {
            UrdfJointPlanar urdfJoint = linkObject.AddComponent<UrdfJointPlanar>();

            urdfJoint.UnityJoint = linkObject.AddComponent<ConfigurableJoint>();
            urdfJoint.UnityJoint.autoConfigureConnectedAnchor = true;

            ConfigurableJoint configurableJoint = (ConfigurableJoint) urdfJoint.UnityJoint;

            // degrees of freedom:
            configurableJoint.xMotion = ConfigurableJointMotion.Free;
            configurableJoint.yMotion = ConfigurableJointMotion.Free;
            configurableJoint.zMotion = ConfigurableJointMotion.Locked;
            configurableJoint.angularXMotion = ConfigurableJointMotion.Locked;
            configurableJoint.angularYMotion = ConfigurableJointMotion.Locked;
            configurableJoint.angularZMotion = ConfigurableJointMotion.Locked;

            return urdfJoint;
        }

        public override float GetPosition()
        {
            Vector3 distanceFromAnchor = UnityJoint.transform.localPosition - UnityJoint.connectedAnchor;
            return distanceFromAnchor.magnitude;
        }

        public override void ImportJointData(Joint joint)
        {
            ConfigurableJoint configurableJoint = (ConfigurableJoint) UnityJoint;

            Vector3 normal = (joint.axis != null) ? GetAxis(joint.axis) : GetDefaultAxis();
            Vector3 axisX = Vector3.forward;
            Vector3 axisY = Vector3.left;
            Vector3.OrthoNormalize(ref normal, ref axisX, ref axisY);
            configurableJoint.axis = axisX;
            configurableJoint.secondaryAxis = axisY;

            // spring, damper & max. force:
            if (joint.dynamics != null)
            {
                configurableJoint.xDrive = GetJointDrive(joint.dynamics);
                configurableJoint.yDrive = GetJointDrive(joint.dynamics);
            }

            if (joint.limit != null)
                configurableJoint.linearLimit = GetLinearLimit(joint.limit);
        }

        #region Export

        public override Joint ExportSpecificJointData(Joint joint)
        {
            ConfigurableJoint configurableJoint = (ConfigurableJoint) UnityJoint;

            joint.axis = GetAxisData(Vector3.Cross(configurableJoint.axis, configurableJoint.secondaryAxis));
            joint.dynamics = new Joint.Dynamics(configurableJoint.xDrive.positionDamper, configurableJoint.xDrive.positionSpring);
            joint.limit = ExportLimitData();

            return joint;
        }

        public override Joint.Limit ExportLimitData()
        {
            ConfigurableJoint configurableJoint = (ConfigurableJoint)UnityJoint;
            return new Joint.Limit(
                Math.Round(-configurableJoint.linearLimit.limit, RoundDigits),
                Math.Round(configurableJoint.linearLimit.limit, RoundDigits),
                EffortLimit, VelocityLimit);
        }

        public override bool AreLimitsCorrect()
        {
            ConfigurableJoint joint = (ConfigurableJoint)UnityJoint;
            return joint != null && joint.linearLimit.limit != 0;
        }

        public override bool IsJointAxisDefined()
        {
            ConfigurableJoint joint = (ConfigurableJoint)UnityJoint;
            return !(Math.Abs(joint.axis.x) < Tolerance &&
                     Math.Abs(joint.axis.y) < Tolerance &&
                     Math.Abs(joint.axis.z) < Tolerance)
                   && !(Math.Abs(joint.secondaryAxis.x) < Tolerance &&
                        Math.Abs(joint.secondaryAxis.y) < Tolerance &&
                        Math.Abs(joint.secondaryAxis.z) < Tolerance);
        }

        #endregion
    }
}

