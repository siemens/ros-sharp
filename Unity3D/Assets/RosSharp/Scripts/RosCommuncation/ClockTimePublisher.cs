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

using System;
using System.Threading;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class ClockTimePublisher : Publisher
    {
        public float Timestep;
        private int timestep { get { return (int)(Mathf.Round(Timestep * 1000)); } }

        //private System.Diagnostics.Stopwatch stopwatch = new System.Diagnostics.Stopwatch();
        private Thread clockTimeIterate;

        protected override void Start()
        {
            base.Start();

            //stopwatch.Start();
            clockTimeIterate = new Thread(ClockTimeIterate);
            clockTimeIterate.Start();
        }

        private void OnApplicationQuit()
        {
            if (clockTimeIterate != null)
                clockTimeIterate.Abort();
            //if (stopwatch != null)
            //    stopwatch.Stop();
        }

        private void ClockTimeIterate()
        {
            while (true)
            {
                //Debug.Log("Time elapsed: " + stopwatch.Elapsed.ToString());
                StartPublication(EventArgs.Empty);
                Thread.Sleep(timestep);
            }
        }
    }
}