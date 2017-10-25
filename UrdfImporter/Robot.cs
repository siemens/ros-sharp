/*
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

using System.Collections.Generic;
using System.Linq;
using System.Xml.Linq;

namespace RosSharp.UrdfImporter
{
    public class Robot
    {
        public string filename;
        public string name;
        public Link root;
        public List<Link.Visual.Material> materials;
        
        public Robot(string filename)
        {
            this.filename = filename;
            XDocument xdoc = XDocument.Load(filename);
            XElement node = xdoc.Element("robot");
            name = node.Attribute("name").Value;

            materials = readMaterials(node); // multiple
            List<Link> Links = readLinks(node); // multiple
            List<Joint> Joints = readJoints(node); // multiple

            // build tree structure from link and joint lists:
            foreach (Link link in Links)
                link.joints = Joints.FindAll(v => v.parent == link.name);
            foreach (Joint joint in Joints)
                joint.ChildLink = Links.Find(v => v.name == joint.child);

            // save root node only:
            root = findRootLink(Links, Joints);
        }
        private static List<Link.Visual.Material> readMaterials(XElement node)
        {
            var materials =
                from child in node.Elements("material")
                select new Link.Visual.Material(child);
            return materials.ToList();
        }

        private static List<Link> readLinks(XElement node)
        {
            var links =
                from child in node.Elements("link")
                select new Link(child);
            return links.ToList();
        }

        private static List<Joint> readJoints(XElement node)
        {
            var joints =
                from child in node.Elements("joint")
                select new Joint(child);
            return joints.ToList();
        }

        private static Link findRootLink(List<Link> Links, List<Joint> Joints, int startIdx = 0)
        {
            Joint joint = Joints[0];
            string parent;
            do
            {
                parent = joint.parent;
                joint = Joints.Find(v => v.child == parent);
            }
            while (joint != null);
            return Links.Find(v => v.name == parent);
        }
    }
}
