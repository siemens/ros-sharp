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

namespace RosSharp.RosBridgeClient.MessageGeneration
{

    public class MessageToken
    {
        public MessageTokenType type;
        public string content;
        public uint lineNum = 0;

        public MessageToken(MessageTokenType type, string content, uint lineNum) {
            this.type = type;
            this.content = content;
            this.lineNum = lineNum;
        }

        public override string ToString() {
            return type + ": " + content + " (" + lineNum + ")";
        }
    }

    public enum MessageTokenType{
        Undefined,
        FilePath,
        Comment,
        BuiltInType,
        DefinedType,
        Header,
        FixedSizeArray,
        VariableSizeArray,
        Identifier,
        ConstantDeclaration,
        Seperator
    }

}