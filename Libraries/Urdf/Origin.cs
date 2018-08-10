﻿/*
© Siemens AG, 2017
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

using System.Xml;
using System.Xml.Linq;

namespace RosSharp.Urdf
{
    public class Origin
    {
        public double[] Xyz;
        public double[] Rpy;

        public Origin(XElement node)
        {
            Xyz = node.Attribute("xyz") != null ? node.Attribute("xyz").ReadDoubleArray() : null;
            Rpy = node.Attribute("rpy") != null ? node.Attribute("rpy").ReadDoubleArray() : null;
        }

        public Origin(double[] xyz, double[] rpy)
        {
            Xyz = xyz;
            Rpy = rpy;
        }

        public void WriteToUrdf(XmlWriter writer)
        {
            writer.WriteStartElement("origin");
            if(Rpy != null)
                writer.WriteAttributeString("rpy", Rpy.DoubleArrayToString());
            if(Xyz != null)
                writer.WriteAttributeString("xyz", Xyz.DoubleArrayToString());
            writer.WriteEndElement();
        }
    }
}
