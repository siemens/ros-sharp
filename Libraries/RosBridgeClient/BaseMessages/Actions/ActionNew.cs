///*
//© Siemens AG, 2019
//Author: Sifan Ye (sifan.ye@siemens.com)

//Licensed under the Apache License, Version 2.0 (the "License");
//you may not use this file except in compliance with the License.
//You may obtain a copy of the License at
//<http://www.apache.org/licenses/LICENSE-2.0>.
//Unless required by applicable law or agreed to in writing, software
//distributed under the License is distributed on an "AS IS" BASIS,
//WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//See the License for the specific language governing permissions and
//limitations under the License.
//*/

//namespace RosSharp.RosBridgeClient
//{
//    public abstract class ActionNew<TActionGoal, TActionResult, TActionFeedback, TGoal, TResult, TFeedback> : Message
//        where TActionGoal: ActionGoal<TGoal>
//        where TActionResult: ActionResult<TResult>
//        where TActionFeedback : ActionFeedback<TFeedback>
//        where TGoal : Message
//        where TResult : Message
//        where TFeedback : Message
//    {
//        public TActionGoal action_goal { get; set; }
//        public TActionResult action_result { get; set; }
//        public TActionFeedback action_feedback { get; set; }

//        public ActionNew() { }

//        public ActionNew(TActionGoal action_goal, TActionResult action_result, TActionFeedback action_feedback) {
//            this.action_goal = action_goal;
//            this.action_result = action_result;
//            this.action_feedback = action_feedback;
//        }
//    }
//}
