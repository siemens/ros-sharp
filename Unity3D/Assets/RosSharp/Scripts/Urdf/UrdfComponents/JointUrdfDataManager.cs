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

namespace RosSharp
{
    //  [ExecuteInEditMode]
    [RequireComponent(typeof(Joint))]
    public class JointUrdfDataManager : MonoBehaviour
    {
        public enum JointTypes { continuous, revolute, prismatic, undefined };
        public string JointName;         
        public JointTypes JointType;

        public bool IsRevoluteOrContinuous
        {
            get { return JointType == JointTypes.continuous || JointType == JointTypes.revolute; }
        }
        public bool IsPrismatic { get { return JointType == JointTypes.prismatic; } }

        public static JointUrdfDataManager AddComponent(GameObject _gameObject, string jointName, string jointType)
        {
            JointUrdfDataManager jointUrdfDataManager = _gameObject.AddComponent<JointUrdfDataManager>();
            jointUrdfDataManager.JointName = jointName;
            jointUrdfDataManager.JointType = GetJointType(jointType);
            return jointUrdfDataManager;
        }

        private static JointTypes GetJointType(string jointType)
        {
            switch (jointType)
            {
                case "continuous":
                    return JointTypes.continuous;
                case "revolute":
                    return JointTypes.revolute;
                case "prismatic":
                    return JointTypes.prismatic;
                default:
                    return JointTypes.undefined;
            }
        }

    }
}

