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

using RosSharp.Urdf;
using UnityEngine;
using Joint = UnityEngine.Joint;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(Joint)), RequireComponent(typeof(UrdfJoint))]
    public class JointStateReader : MonoBehaviour
    {
        private UrdfJoint urdfJoint;

        private void Start()
        {
            urdfJoint = GetComponent<UrdfJoint>();
        }

        public void Read(out string name, out float position, out float velocity, out float effort)
        {
            name = urdfJoint.JointName;
            position = urdfJoint.GetPosition();
            velocity = urdfJoint.GetVelocity();
            effort = urdfJoint.GetEffort();
        }
    }
}
