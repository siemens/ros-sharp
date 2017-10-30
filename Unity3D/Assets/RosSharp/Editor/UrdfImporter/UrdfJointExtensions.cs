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

namespace RosSharp.UrdfImporter
{
    public static class UrdfJointExtensions
    {
        public static UnityEngine.Joint Create(this Joint joint, GameObject gameObject, GameObject parent)
        {
            Rigidbody parentRigidbody = parent.GetComponent<Rigidbody>();
            if (parentRigidbody == null)
                return null;

            gameObject.name = gameObject.name + " (" + joint.type + " Joint: " + joint.name + ")";

            if (joint.type == "fixed")
                return joint.CreateFixedJoint(gameObject, parentRigidbody);
            if (joint.type == "continuous" || joint.type == "revolute")
                return joint.CreateHingeJoint(gameObject, parentRigidbody);
            if (joint.type == "floating")
                return joint.CreateFloatingJoint(gameObject, parentRigidbody);
            if (joint.type == "prismatic")
                return joint.CreatePrismaticJoint(gameObject, parentRigidbody);
            if (joint.type == "planar")
                return joint.CreatePlanarJoint(gameObject, parentRigidbody);
            return null;
        }


        public static FixedJoint CreateFixedJoint(this Joint joint, GameObject gameObject, Rigidbody parentRigidbody)
        {
            FixedJoint fixedJoint = gameObject.AddComponent<FixedJoint>();
            fixedJoint.connectedBody = parentRigidbody;
            return fixedJoint;
        }

        public static HingeJoint CreateHingeJoint(this Joint joint, GameObject gameObject, Rigidbody parentRigidbody)
        {
            HingeJoint hingeJoint = gameObject.AddComponent<HingeJoint>();
            hingeJoint.connectedBody = parentRigidbody;

            // axis:
            hingeJoint.axis = (joint.axis != null) ? joint.axis.GetAxis() : joint.axis.GetDefaultAxis();

            // origin:
            hingeJoint.autoConfigureConnectedAnchor = false;
            hingeJoint.connectedAnchor = joint.GetConnectedAnchor();

            // spring, damper & position:
            if (joint.dynamics != null)
                hingeJoint.spring = joint.dynamics.GetJointSpring();

            // limits:        
            if (joint.type == "revolute" && joint.limit != null)
                hingeJoint.limits = joint.limit.GetJointLimits();

            return hingeJoint;
        }

        public static ConfigurableJoint CreateFloatingJoint(this Joint joint, GameObject gameObject, Rigidbody parentRigidbody)
        {
            ConfigurableJoint floatingJoint = gameObject.AddComponent<ConfigurableJoint>();
            floatingJoint.connectedBody = parentRigidbody;
            // origin:
            floatingJoint.autoConfigureConnectedAnchor = false;
            floatingJoint.connectedAnchor = joint.GetConnectedAnchor();
            return floatingJoint;
        }

        public static ConfigurableJoint CreatePrismaticJoint(this Joint joint, GameObject gameObject, Rigidbody parentRigidbody)
        {
            ConfigurableJoint prismaticJoint = gameObject.AddComponent<ConfigurableJoint>();
            prismaticJoint.connectedBody = parentRigidbody;

            prismaticJoint.axis = (joint.axis != null) ? joint.axis.GetAxis() : joint.axis.GetDefaultAxis();

            // degrees of freedom:
            prismaticJoint.xMotion = ConfigurableJointMotion.Limited;
            prismaticJoint.yMotion = ConfigurableJointMotion.Locked;
            prismaticJoint.zMotion = ConfigurableJointMotion.Locked;
            prismaticJoint.angularXMotion = ConfigurableJointMotion.Locked;
            prismaticJoint.angularYMotion = ConfigurableJointMotion.Locked;
            prismaticJoint.angularZMotion = ConfigurableJointMotion.Locked;

            // origin:
            prismaticJoint.autoConfigureConnectedAnchor = false;
            prismaticJoint.connectedAnchor = joint.GetConnectedAnchor();

            // spring, damper & max. force:
            if (joint.dynamics != null)
                prismaticJoint.xDrive = joint.dynamics.GetJointDrive();

            // limits:
            if (joint.limit != null)
            {
                prismaticJoint.lowAngularXLimit = joint.limit.GetLowSoftJointLimit();
                prismaticJoint.highAngularXLimit = joint.limit.GetHighSoftJointLimit();
            }
            return prismaticJoint;
        }

        public static ConfigurableJoint CreatePlanarJoint(this Joint joint, GameObject gameObject, Rigidbody parentRigidbody)
        {
            ConfigurableJoint planarJoint = gameObject.AddComponent<ConfigurableJoint>();
            planarJoint.connectedBody = parentRigidbody;

            Vector3 normal = (joint.axis != null) ? joint.axis.GetAxis() : joint.axis.GetDefaultAxis();
            Vector3 axisX = Vector3.forward;
            Vector3 axisY = Vector3.left;
            Vector3.OrthoNormalize(ref normal, ref axisX, ref axisY);
            planarJoint.axis = axisX;
            planarJoint.secondaryAxis = axisY;

            // degrees of freedom:
            planarJoint.xMotion = ConfigurableJointMotion.Free;
            planarJoint.yMotion = ConfigurableJointMotion.Free;
            planarJoint.zMotion = ConfigurableJointMotion.Locked;
            planarJoint.angularXMotion = ConfigurableJointMotion.Locked;
            planarJoint.angularYMotion = ConfigurableJointMotion.Locked;
            planarJoint.angularZMotion = ConfigurableJointMotion.Locked;

            // origin:
            planarJoint.autoConfigureConnectedAnchor = false;
            planarJoint.connectedAnchor = joint.GetConnectedAnchor();

            // spring, damper & max. force:
            if (joint.dynamics != null)
            {
                planarJoint.xDrive = joint.dynamics.GetJointDrive();
                planarJoint.yDrive = joint.dynamics.GetJointDrive();
            }
            return planarJoint;
        }

        public static Vector3 GetConnectedAnchor(this Joint joint)
        {
            if (joint.origin != null)
                return joint.origin.GetPosition(); // todo: where to put rotation (if it exists in URDF)?
            else
                return Vector3.zero;
        }
    }

    public static class UrdfJointAxisExtenisions
    {
        public static Vector3 GetAxis(this Joint.Axis axis)
        {
            return new Vector3(
                (float)-axis.xyz[1],
                (float)axis.xyz[2],
                (float)axis.xyz[0]);
        }
        public static Vector3 GetDefaultAxis(this Joint.Axis axis)
        {
            return new Vector3(-1, 0, 0);
        }
    }

    public static class UrdfJointDynamicsExtensions
    {
        public static JointDrive GetJointDrive(this Joint.Dynamics dynamics)
        {
            JointDrive jointDrive = new JointDrive();
            jointDrive.maximumForce = float.MaxValue;
            jointDrive.positionDamper = (float)dynamics.damping;
            jointDrive.positionSpring = (float)dynamics.friction;
            return jointDrive;
        }
        public static JointSpring GetJointSpring(this Joint.Dynamics dynamics)
        {
            JointSpring jointSpring = new JointSpring();
            jointSpring.damper = (float)dynamics.damping;
            jointSpring.spring = (float)dynamics.friction;
            jointSpring.targetPosition = 0;
            return jointSpring;
        }
    }

    public static class UrdfJointLimitsExtensions
    {
        public static JointLimits GetJointLimits(this Joint.Limit limit)
        {
            JointLimits jointLimits = new JointLimits();
            jointLimits.min = (float)limit.lower * Mathf.Rad2Deg;
            jointLimits.max = (float)limit.upper * Mathf.Rad2Deg;
            return jointLimits;
        }

        public static SoftJointLimit GetLowSoftJointLimit(this Joint.Limit limit)
        {
            SoftJointLimit softJointLimit = new SoftJointLimit();
            softJointLimit.limit = (float)limit.lower * Mathf.Rad2Deg;
            return softJointLimit;
        }
        public static SoftJointLimit GetHighSoftJointLimit(this Joint.Limit limit)
        {
            SoftJointLimit softJointLimit = new SoftJointLimit();
            softJointLimit.limit = (float)limit.upper * Mathf.Rad2Deg;
            return softJointLimit;
        }
    }
}