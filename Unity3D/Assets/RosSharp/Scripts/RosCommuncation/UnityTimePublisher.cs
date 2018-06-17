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

namespace RosSharp.RosBridgeClient
{
    public abstract class UnityTimePublisher<T> : Publisher<T> where T : Message
    {
        public enum Timings { UnityFrameTime, UnityFixedTime }
        public Timings Timing;

        public int SkipSteps;
        private int SkipFrames { get { return SkipSteps + 1; } }

        private int stepsSincePublication;

        protected override void Start()
        {
            base.Start();
        }

        private void LateUpdate()
        {
            UpdateStep(Timings.UnityFrameTime);
        }

        // make sure to set Execution Order of this script sufficiently late
        // to publish the recent data from the current FixedUpdate step

        private void FixedUpdate() 
        {
            UpdateStep(Timings.UnityFixedTime);
        }

        private void UpdateStep(Timings UpdateTiming)
        {
            if (Timing == UpdateTiming
                && stepsSincePublication++ % SkipFrames == 0)
            {
                //     Debug.Log("Frame No. " + Time.frameCount);
                Publish();
            }
        }
    }
}