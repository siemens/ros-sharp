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

using System.Collections.Generic;
using System.Linq;
using System.Xml;
using System.Xml.Linq;

namespace RosSharp.UrdfImporter
{
    public class Robot
    {
        public string filename;
        public string name;
        public Link root;
        public List<Link.Visual.Material> materials;

        public List<Link> links;
        public List<Joint> joints;

        public Robot(string filename)
        {
            this.filename = filename;
            XDocument xdoc = XDocument.Load(filename);
            XElement node = xdoc.Element("robot");
            name = node.Attribute("name").Value;

            materials = ReadMaterials(node); // multiple
            links = ReadLinks(node); // multiple
            joints = ReadJoints(node); // multiple

            // build tree structure from link and joint lists:
            foreach (Link link in links)
                link.joints = joints.FindAll(v => v.parent == link.name);
            foreach (Joint joint in joints)
                joint.ChildLink = links.Find(v => v.name == joint.child);

            // save root node only:
            root = FindRootLink(links, joints);
        }

        public Robot(string filename, string name)
        {
            this.filename = filename;
            this.name = name;

            links = new List<Link>();
            joints = new List<Joint>();
            materials = new List<Link.Visual.Material>();
        }

        private static List<Link.Visual.Material> ReadMaterials(XElement node)
        {
            var materials =
                from child in node.Elements("material")
                select new Link.Visual.Material(child);
            return materials.ToList();
        }

        private static List<Link> ReadLinks(XElement node)
        {
            var links =
                from child in node.Elements("link")
                select new Link(child);
            return links.ToList();
        }

        private static List<Joint> ReadJoints(XElement node)
        {
            var joints =
                from child in node.Elements("joint")
                select new Joint(child);
            return joints.ToList();
        }

        private static Link FindRootLink(List<Link> Links, List<Joint> Joints, int startIdx = 0)
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

        public void WriteToUrdf()
        {
            XmlWriterSettings settings = new XmlWriterSettings { Indent = true, NewLineOnAttributes = false };

            using (XmlWriter writer = XmlWriter.Create(filename, settings))
            {
                writer.WriteStartDocument();
                writer.WriteStartElement("robot");
                writer.WriteAttributeString("name", name);

                foreach (var material in materials)
                    material.WriteToUrdf(writer);
                foreach (var link in links)
                    link.WriteToUrdf(writer);
                foreach (var joint in joints)
                    joint.WriteToUrdf(writer);
               
                writer.WriteEndElement();
                writer.WriteEndDocument();
            }
        }
    }
}
