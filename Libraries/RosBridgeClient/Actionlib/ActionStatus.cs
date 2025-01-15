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

- Added ROS2 action support:
    - New status enum based on ROS2 action design.
    - This file is moved from (auto generated) MessageTypes/Actionlib to Actionlib.

    © Siemens AG 2025, Mehmet Emre Cakal, emre.cakal@siemens.com/m.emrecakal@gmail.com
*/

namespace RosSharp.RosBridgeClient.Actionlib
{
#if !ROS2
    // This is defined according to actionlib_msgs/GoalStatus
    public enum ActionStatus
    {
        NO_GOAL = -1,    // For internal server use. If status is NA, published status array will have length 0
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

#else
    public enum ActionStatus
    {
        // For internal server use. If status is NA, published status array will have length 0
        STATUS_NO_GOAL = -1,
        //  Indicates status has not been properly set.
        STATUS_UNKNOWN = 0,
        //  The goal has been accepted and is awaiting execution.
        STATUS_ACCEPTED = 1,
        //  The goal is currently being executed by the action server.
        STATUS_EXECUTING = 2,
        //  The client has requested that the goal be canceled and the action server has
        //  accepted the cancel request.
        STATUS_CANCELING = 3,
        //  The goal was achieved successfully by the action server.
        STATUS_SUCCEEDED = 4,
        //  The goal was canceled after an external request from an action client.
        STATUS_CANCELED = 5,
        //  The goal was terminated by the action server without an external request.
        STATUS_ABORTED = 6,
    }

#endif
}