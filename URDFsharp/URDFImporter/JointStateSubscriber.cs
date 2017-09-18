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
using RosBridgeClient;

public class JointStateSubscriber : MonoBehaviour
{
    private JointStateManager[] jointStateManagers;

    private RosSocket rosSocket;
    public int UpdateTime;

    public float[] JointPositions; // deg
    public float[] JointVelocities; // deg/s
    
    private int numberOfJoints;
    
    private void Start()
    {
        rosSocket = transform.GetComponent<RosConnector>().RosSocket;
        rosSocket.Subscribe("/joint_states", "sensor_msgs/JointState", updateJointStates, UpdateTime);
        jointStateManagers = FindObjectsOfType<JointStateManager>();
        numberOfJoints = jointStateManagers.Length;
        JointPositions = new float[numberOfJoints];
        JointVelocities = new float[numberOfJoints];
    }

    private void updateJointStates(Message message)
    {
        SensorJointStates sensorJointStates = (SensorJointStates)message;
        JointPositions = rad2Deg(sensorJointStates.position);
        JointVelocities = rad2Deg(sensorJointStates.velocity);

        foreach (JointStateManager jointStateManager in jointStateManagers)
            jointStateManager.updateJointState(JointPositions[jointStateManager.jointStateId]);
    }

    private static float[] rad2Deg(float[] values)
    {
        for (int i = 0; i < values.Length; i++)
            values[i] = values[i] * Mathf.Rad2Deg;

        return values;
    }
}
