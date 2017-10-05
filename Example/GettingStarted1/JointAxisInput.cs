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

public class JointAxisInput : MonoBehaviour
{
    public float AxisInput;
    public string AxisName;
    private JointControl jointControl;

    private void Start ()
    {
        jointControl =  GetComponent<JointControl>();
    }

    private void Update ()
    {
        AxisInput = Input.GetAxis(AxisName);
        jointControl.SetAngle(AxisInput);
    }
}