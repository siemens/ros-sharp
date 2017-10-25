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

namespace RosSharp.Urdf
{
    public class Link
    {
        public string name;
        public Inertial inertial;
        public List<Visual> visuals;
        public List<Collision> collisions;
        public List<Joint> joints;

        public Link(XElement node)
        {
            name = (string)node.Attribute("name");  // required
            inertial = (node.Element("inertial") != null) ? new Inertial(node.Element("inertial")) : null;  // optional     
            visuals = readVisuals(node); // multiple
            collisions = readCollisions(node); // optional   
        }
        private static List<Collision> readCollisions(XElement node)
        {
            var collisions =
                from child in node.Elements("collision")
                select new Collision(child);
            return collisions.ToList();

        }
        private static List<Visual> readVisuals(XElement node)
        {
            var visuals =
                from child in node.Elements("visual")
                select new Visual(child);
            return visuals.ToList();
        }

        public class Inertial
        {
            public double mass;
            public Origin origin;
            public Inertia inertia;

            public Inertial(XElement node)
            {
                origin = (node.Element("origin") != null) ? new Origin(node.Element("origin")) : null; // optional  
                mass = (double)node.Element("mass").Attribute("value");// required
                inertia = new Inertia(node.Element("inertia")); // required
            }

            public class Inertia
            {
                public double ixx;
                public double ixy;
                public double ixz;
                public double iyy;
                public double iyz;
                public double izz;

                public Inertia(XElement node)
                {
                    ixx = (double)node.Attribute("ixx");
                    ixy = (double)node.Attribute("ixy");
                    ixz = (double)node.Attribute("ixz");
                    iyy = (double)node.Attribute("iyy");
                    iyz = (double)node.Attribute("iyz");
                    izz = (double)node.Attribute("izz");
                }
            }
        }
      
        public class Collision
        {
            public string name;
            public Origin origin;
            public Geometry geometry;

            public Collision(XElement node)
            {
                name = (string)node.Attribute("name"); // optional
                origin = (node.Element("origin") != null) ? new Origin(node.Element("origin")) : null; // optional  
                geometry = new Geometry(node.Element("geometry")); // required
            }
        }
        public class Visual
        {
            public string name;
            public Origin origin;
            public Geometry geometry;
            public Material material;

            public Visual(XElement node)
            {
                name = (string)node.Attribute("name"); // optional
                origin = (node.Element("origin") != null) ? new Origin(node.Element("origin")) : null; // optional
                geometry = new Geometry(node.Element("geometry")); // required
                material = (node.Element("material") != null) ? new Material(node.Element("material")) : null; // optional
            }

            public class Material
            {
                public string name;
                public Color color;
                public Texture texture;

                public Material(XElement node)
                {
                    name = (string)node.Attribute("name"); // required
                    color = (node.Element("color") != null) ? new Color(node.Element("color")) : null; // optional  
                    texture = (node.Element("texture") != null) ? new Texture(node.Element("texture")) : null;

                }
                public class Texture
                {
                    public string filename;

                    public Texture(XElement node)
                    {
                        filename = (string)node.Attribute("filename"); // required
                    }
                }

                public class Color
                {
                    public double[] rgba;

                    public Color(XElement node)
                    {
                        rgba = node.Attribute("rgba").ReadDoubleArray(); // required
                    }
                }

            }
        }
        
        public class Geometry
        {
            public Box box;
            public Cylinder cylinder;
            public Sphere sphere;
            public Mesh mesh;

            public Geometry(XElement node)
            {
                box = (node.Element("box") != null) ? new Box(node.Element("box")) : null; // optional  
                cylinder = (node.Element("cylinder") != null) ? new Cylinder(node.Element("cylinder")) : null; // optional  
                sphere = (node.Element("sphere") != null) ? new Sphere(node.Element("sphere")) : null; // optional  
                mesh = (node.Element("mesh") != null) ? new Mesh(node.Element("mesh")) : null; // optional           
            }

            public class Box
            {
                public double[] size;

                public Box(XElement node)
                {
                    size = node.Attribute("size") != null ? node.Attribute("size").ReadDoubleArray() : null;
                }
            }


            public class Cylinder
            {
                public double radius;
                public double length;

                public Cylinder(XElement node)
                {
                    radius = (double)node.Attribute("radius");
                    length = (double)node.Attribute("length");
                }
            }


            public class Sphere
            {
                public double radius;

                public Sphere(XElement node)
                {
                    radius = (double)node.Attribute("radius");
                }
            }

            public class Mesh
            {
                public string filename;
                public double[] scale;

                public Mesh(XElement node)
                {
                    filename = (string)node.Attribute("filename");
                    scale = node.Attribute("scale") != null ? node.Attribute("scale").ReadDoubleArray() : null;
                }
            }
        }     
    }    
}
