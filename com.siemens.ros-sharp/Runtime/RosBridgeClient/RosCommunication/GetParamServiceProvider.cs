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

using System.Collections.Generic;
using rosapi = RosSharp.RosBridgeClient.MessageTypes.Rosapi;

namespace RosSharp.RosBridgeClient
{
    public class GetParamServiceProvider : UnityServiceProvider<rosapi.GetParamRequest, rosapi.GetParamResponse>
    {
        public Dictionary<string, string> Parameters = new Dictionary<string, string>
        {
            { "UnityVersion", UnityEngine.Application.version}
        };
        
        protected override bool ServiceCallHandler(rosapi.GetParamRequest request, out rosapi.GetParamResponse response)
        {
            response = new rosapi.GetParamResponse();
            string responseValue;
            if (Parameters.TryGetValue(request.name, out responseValue))
            {
                response.value = responseValue;
                return true;
            }
            response.value = "unknown parameter " + request.name;
            return false;
        }
    }
}