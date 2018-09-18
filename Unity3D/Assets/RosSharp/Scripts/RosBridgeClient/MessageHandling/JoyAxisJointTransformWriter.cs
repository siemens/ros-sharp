﻿/*
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

using RosSharp.Urdf;
using UnityEngine;
using Joint = UnityEngine.Joint;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(Joint)), RequireComponent(typeof(JointStateWriter)), RequireComponent(typeof(UrdfJoint))]
    public class JoyAxisJointTransformWriter : JoyAxisWriter
    {
        public float MaxVelocity;

        private JointStateWriter jointStateWriter;
        private UrdfJoint urdfJoint;

        private bool isMessageReceived;
        private float lastMessageTime;
        private float state;

        private HingeJointLimitsManager hingeJointLimitsManager;
        private PrismaticJointLimitsManager prismaticJointLimitsManager;
        private bool useLimits;
        private float currentAxisValue;

        private void Start()
        {
            jointStateWriter = GetComponent<JointStateWriter>();
            urdfJoint = GetComponent<UrdfJoint>();
            hingeJointLimitsManager = GetComponent<HingeJointLimitsManager>();
            prismaticJointLimitsManager = GetComponent<PrismaticJointLimitsManager>();
            useLimits = (hingeJointLimitsManager != null || prismaticJointLimitsManager != null);
        }

        private void ApplyLimits()
        {
            Vector2 limits = Vector2.zero;
            if (urdfJoint.IsRevoluteOrContinuous)
                limits = new Vector2(hingeJointLimitsManager.LargeAngleLimitMin,hingeJointLimitsManager.LargeAngleLimitMax);
            else if (urdfJoint.JointType == UrdfJoint.JointTypes.Prismatic)
                limits = new Vector2(prismaticJointLimitsManager.PositionLimitMin, prismaticJointLimitsManager.PositionLimitMax);
          
            state = Mathf.Clamp(state, limits.x, limits.y);
        }

        private void Update()
        {
            if (isMessageReceived)
                ProcessMessage();
        }

        private void ProcessMessage()
        {
            float deltaTime = Time.timeSinceLevelLoad - lastMessageTime;
            state += currentAxisValue * MaxVelocity * deltaTime;
            if (useLimits)
                ApplyLimits();

            lastMessageTime = Time.timeSinceLevelLoad;

            jointStateWriter.Write(urdfJoint.IsRevoluteOrContinuous ? state * Mathf.Deg2Rad : state);
            isMessageReceived = false;
        }

        public override void Write(float value)
        {
            currentAxisValue = value;
            isMessageReceived = true;
        }
    }
}