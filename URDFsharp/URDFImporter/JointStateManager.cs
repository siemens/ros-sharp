*/
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

public class JointStateManager : MonoBehaviour
{
    public int jointStateId;
    private HingeJoint _hingeJoint;

    private float newAngle;
    private float prevAngle;
    private bool doUpdate;

    private void Start()
    {
        _hingeJoint = GetComponent<HingeJoint>();
        newAngle = 0;
        prevAngle = 0;
    }

    private void Update()
    {
        if (doUpdate)
        {
            Vector3 anchor = transform.TransformPoint(_hingeJoint.anchor);
            Vector3 axis = transform.TransformDirection(_hingeJoint.axis);
            transform.RotateAround(anchor, axis, (newAngle - prevAngle));
            prevAngle = newAngle;
            doUpdate = false;
        }
    }
    public void updateJointState(float angle)
    {
        newAngle = angle;
        doUpdate = true;
    }
}
