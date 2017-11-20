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

[RequireComponent(typeof(HingeJoint))]
public class JointTransformManager : MonoBehaviour
{
    public float Step;

    public float AngleSetPoint;
    private float prevAngle;
    private float incAngle;

    private HingeJoint joint;

    private void Start()
    {
        joint = GetComponent<HingeJoint>();
    }

    private void Update()
    {
        if (incAngle == 0)
            return;

        AngleSetPoint = AngleSetPoint + incAngle * Step;

        if (joint.useLimits)
        {
            AngleSetPoint = (AngleSetPoint >= joint.limits.min) ? AngleSetPoint : joint.limits.min;
            AngleSetPoint = (AngleSetPoint <= joint.limits.max) ? AngleSetPoint : joint.limits.max;
        }

        Vector3 anchor = transform.TransformPoint(joint.anchor);
        Vector3 axis = transform.TransformDirection(joint.axis);
        transform.RotateAround(anchor, axis, (AngleSetPoint - prevAngle));
        prevAngle = AngleSetPoint;
        incAngle = 0;
    }
    public void SetAngle(float angle)
    {
        incAngle = angle;
    }
}