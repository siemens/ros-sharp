/*
© Siemens AG, 2019
Author: Sifan Ye (sifan.ye@siemens.com)

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
    // This is defined according to actionlib_msgs/GoalStatus
    public enum ActionStatus
    {
        NA = -1,    // For internal server use. If status is NA, published status array will have length 0
        PENDING,    //  The goal has yet to be processed by the action server
        ACTIVE,     //  The goal is currently being processed by the action server
        PREEMPTED,  //  The goal received a cancel request after it started executing and has since completed its execution (Terminal State)
        SUCCEEDED,  //  The goal was achieved successfully by the action server (Terminal State)
        ABORTED,    //  The goal was aborted during execution by the action server due to some failure (Terminal State)
        REJECTED,   //  The goal was rejected by the action server without being processed, because the goal was unattainable or invalid (Terminal State)
        PREEMPTING, //  The goal received a cancel request after it started executing and has not yet completed execution
        RECALLING,  //  The goal received a cancel request before it started executing, but the action server has not yet confirmed that the goal is canceled
        RECALLED,   //  The goal received a cancel request before it started executing and was successfully cancelled (Terminal State)
        LOST,       //  An action client can determine that a goal is LOST. This should not be sent over the wire by an action server
    }
}
