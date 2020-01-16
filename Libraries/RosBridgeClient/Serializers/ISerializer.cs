/*
© University of Kent, 2019
Author: Odysseas Doumas <od79@kent.ac.uk>
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
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RosSharp.RosBridgeClient.Serializers
{
    public interface ISerializer
    {
        byte[] Serialize<T>(T communication);
        ArraySegment<byte> SerializeUnsafe<T>(T communication);

        IReceivedMessage DeserializeReceived(byte[] bytes);
        IReceivedMessage DeserializeReceived(ArraySegment<byte> bytes);

        string GetJsonString(byte[] bytes);
    }
}
