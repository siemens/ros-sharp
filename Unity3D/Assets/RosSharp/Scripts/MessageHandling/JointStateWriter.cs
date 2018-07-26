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

using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(Joint)), RequireComponent(typeof(JointUrdfDataManager))]
    public class JointStateWriter : MonoBehaviour
    {
        private Joint joint;
        private JointUrdfDataManager jointUrdfDataManager;

        private float newState; // rad or m
        private float prevState; // rad or m
        private bool isNewStateReceived;

        private void Start()
        {
            joint = GetComponent<UnityEngine.Joint>();
            jointUrdfDataManager = GetComponent<JointUrdfDataManager>();
        }

        private void Update()
        {
            if (isNewStateReceived)
            {
                WriteUpdate();
                isNewStateReceived = false;
            }
        }
        private void WriteUpdate()
        {
            if (jointUrdfDataManager.IsRevoluteOrContinuous)
                WriteHingeJointUpdate();
            else if (jointUrdfDataManager.IsPrismatic)
                WritePrismaticJointUpdate();
            
        prevState = newState;
        }

        private void WriteHingeJointUpdate()
        {
            Vector3 anchor = transform.TransformPoint(joint.anchor);
            Vector3 axis = transform.TransformDirection(joint.axis);
            transform.RotateAround(anchor, axis, -(newState -prevState) * Mathf.Rad2Deg);
        }
        private void WritePrismaticJointUpdate()
        {
            Vector3 axis = transform.TransformDirection(joint.axis);
            transform.Translate(axis * (newState - prevState));
        }

        public void Write(float state)
        {
            newState = state;
            isNewStateReceived = true;
        }
    }
}