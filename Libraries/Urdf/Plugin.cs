/*
© Siemens AG, 2018
Author: Suzannah Smith (suzannah.smith@siemens.com)

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

using System.Xml;
using System.Xml.Linq;

namespace RosSharp.Urdf
{
    public class Plugin
    {
        public string text;

        public Plugin(XElement node)
        {
            text = node.ToString();
        }

        public Plugin(string text)
        {
            this.text = text;
        }

        public void WriteToUrdf(XmlWriter writer)
        {
            XmlDocument xDoc = new XmlDocument {PreserveWhitespace = true};
            xDoc.LoadXml(text);
            xDoc.WriteContentTo(writer);
            writer.WriteWhitespace("\n");
        }
    }
}
