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

using Newtonsoft.Json;

namespace RosSharp.RosBridgeClient.Services.RosApi
{
    public class GetParamRequest : Message
    {
        public string name;

        [JsonProperty("default")]
        public string _default;

        public GetParamRequest(string name, string _default) : this()
        {
        
            this.name = name;
            this._default = _default;
        }
        public GetParamRequest()
        {
            RosMessageName = "rosapi/GetParam";
        }
    }

    public class GetParamResponse : Message
    {
        public string value;
        public GetParamResponse()
        {
            RosMessageName = "rosapi/GetParam";
        }
    }
}