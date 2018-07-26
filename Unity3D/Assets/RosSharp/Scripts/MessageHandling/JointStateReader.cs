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

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(Joint)), RequireComponent(typeof(JointUrdfDataManager))]
    public class JointStateReader : MonoBehaviour
    {
        private IUrdfJoint joint;
        private JointUrdfDataManager jointUrdfDataManager;

        private void Start()
        {
            jointUrdfDataManager = GetComponent<JointUrdfDataManager>();

            if (jointUrdfDataManager.IsRevoluteOrContinuous)
                joint = new RevoluteJoint(GetComponent<HingeJoint>());
            else if (jointUrdfDataManager.IsPrismatic)
                joint = new PrismaticJoint(GetComponent<ConfigurableJoint>());
        }

        public void Read(out string name, out float position, out float velocity, out float effort)
        {
            name = jointUrdfDataManager.JointName;
            position = joint.GetPosition();
            velocity = joint.GetVelocity();
            effort = joint.GetEffort();
        }

        public interface IUrdfJoint
        {
            float GetPosition();
            float GetVelocity();
            float GetEffort();
        }
        public class PrismaticJoint : IUrdfJoint
        {
            private ConfigurableJoint configurableJoint;

            public PrismaticJoint(ConfigurableJoint _configurableJoint)
            {
                configurableJoint = _configurableJoint;
            }
            public float GetPosition()
            {
                return Vector3.Dot(configurableJoint.transform.localPosition - configurableJoint.connectedAnchor, configurableJoint.axis);
            }
            public float GetVelocity()
            {
                return 0;
            }
            public float GetEffort()
            {
                return 0;
            }
        }
        public class RevoluteJoint : IUrdfJoint
        {
            private HingeJoint hingeJoint;

            public RevoluteJoint(HingeJoint _hingeJoint)
            {
                hingeJoint = _hingeJoint;
            }
            public float GetPosition()
            {
                return -hingeJoint.angle * Mathf.Deg2Rad;
            }
            public float GetVelocity()
            {
                return -hingeJoint.velocity * Mathf.Deg2Rad;
            }
            public float GetEffort()
            {
                return -hingeJoint.motor.force;
            }
        }
    }
}
