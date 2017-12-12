using RosSharp.RosBridgeClient;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;


using UnityEngine;

namespace RosSharp
{
    class OculusButtonsManager : MonoBehaviour
    {
        SensorJoy sensorJoy;

        private bool hasChanged = false;

        private void Start()
        {
            sensorJoy = new SensorJoy();
        }

        private void Update()
        {
            sensorJoy.buttons = new long[]{
                OculusPoses.poseVR.Buttons.X.state ? 1 : 0,
                OculusPoses.poseVR.Buttons.Y.state ? 1 : 0,
                OculusPoses.poseVR.Buttons.A.state ? 1 : 0,
                OculusPoses.poseVR.Buttons.B.state ? 1 : 0,
                OculusPoses.poseVR.Buttons.LThumbstickButton.state ? 1 : 0,
                OculusPoses.poseVR.Buttons.RThumbstickButton.state ? 1 : 0
            };

            sensorJoy.axes = new float[]
            {
                OculusPoses.poseVR.Buttons.LThumbstick.x,
                OculusPoses.poseVR.Buttons.LThumbstick.y,
                OculusPoses.poseVR.Buttons.RThumbstick.x,
                OculusPoses.poseVR.Buttons.RThumbstick.y,
                OculusPoses.poseVR.Buttons.LIndexTrigger,
                OculusPoses.poseVR.Buttons.RIndexTrigger,
                OculusPoses.poseVR.Buttons.LHandTrigger,
                OculusPoses.poseVR.Buttons.RHandTrigger
            };

            this.hasChanged = true;
        }

        public bool HasChanged()
        {
            return this.hasChanged;
        }

        public SensorJoy GetSensorJoy()
        {
            this.hasChanged = false;
            return this.sensorJoy;
        }

    }
}
 
