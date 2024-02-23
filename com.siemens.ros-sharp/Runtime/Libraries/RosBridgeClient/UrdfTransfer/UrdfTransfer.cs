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

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using System.Xml.Linq;

namespace RosSharp.RosBridgeClient.UrdfTransfer
{
    public abstract class UrdfTransfer {
        protected RosSocket RosSocket;

        public string RobotName;

        public Dictionary<string, ManualResetEvent> Status;
        public Dictionary<string, bool> FilesBeingProcessed;
        
        public abstract void Transfer();
        
        protected static bool HasValidResourcePath(XElement xElement)
        {
            return (xElement.Name.ToString().Equals("mesh") || xElement.Name.ToString().Equals("texture"))
                   && xElement.Attribute("filename") != null;
        }

        protected static bool IsColladaFile(Uri uri)
        {
            return Path.GetExtension(uri.LocalPath).ToLowerInvariant() == ".dae";
        }

        protected List<Uri> ReadDaeTextureUris(Uri resourceFileUri, XDocument xDocument)
        {
            XNamespace xmlns = "http://www.collada.org/2005/11/COLLADASchema";
            return (from x in xDocument.Descendants()
                where x.Name.LocalName == "library_images"
                let imageElement = x.Element(xmlns + "image")
                where imageElement != null
                let initElement = imageElement.Element(xmlns + "init_from")
                where initElement != null
                select new Uri(resourceFileUri, initElement.Value)).ToList();
        }

        protected static List<Uri> ReadResourceFileUris(XDocument xDocument)
        {
            return (from xElement in xDocument.Descendants()
                where HasValidResourcePath(xElement)
                let filename = xElement.Attribute("filename").Value
                where Uri.IsWellFormedUriString(filename, UriKind.Absolute)
                select new Uri(filename)).ToList();
        }
    }
}