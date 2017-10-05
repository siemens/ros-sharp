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

using System.Xml.Linq;

namespace Urdf
{
    public class Joint
    {
        public string name;
        public string type;
        public Origin origin;
        public string parent;
        public string child;
        public Axis axis;
        public Calibration calibration;
        public Dynamics dynamics;
        public Limit limit;
        public Mimic mimic;
        public SafetyController safetyController;

        public Link ChildLink;

        public Joint(XElement node)
        {
            name = (string)node.Attribute("name"); // required
            type = (string)node.Attribute("type"); // required
            origin = (node.Element("origin") != null) ? new Origin(node.Element("origin")) : null; // optional  
            parent = (string)node.Element("parent").Attribute("link"); // required
            child = (string)node.Element("child").Attribute("link"); // required
            axis = (node.Element("axis") != null) ? new Axis(node.Element("axis")) : null;  // optional 
            calibration = (node.Element("calibration") != null) ? new Calibration(node.Element("calibration")) : null;  // optional 
            dynamics = (node.Element("dynamics") != null) ? new Dynamics(node.Element("dynamics")) : null;  // optional 
            limit = (node.Element("limit") != null) ? new Limit(node.Element("limit")) : null;  // required only for revolute and prismatic joints
            mimic = (node.Element("mimic") != null) ? new Mimic(node.Element("mimic")) : null;  // optional
            safetyController = (node.Element("safety_controller") != null) ? new SafetyController(node.Element("safety_controller")) : null;  // optional
        }

        public class Axis
        {
            public double[] xyz;

            public Axis(XElement node)
            {
                xyz = node.Attribute("xyz") != null ? node.Attribute("xyz").ReadDoubleArray() : null;
            }
        }

        public class Calibration
        {
            public double rising;
            public double falling;

            public Calibration(XElement node)
            {
                rising = node.Attribute("rising").ReadOptionalDouble();  // optional
                falling = node.Attribute("falling").ReadOptionalDouble();  // optional
            }
        }

        public class Dynamics
        {
            public double damping;
            public double friction;

            public Dynamics(XElement node)
            {
                damping = node.Attribute("damping").ReadOptionalDouble(); // optional
                friction = node.Attribute("friction").ReadOptionalDouble(); // optional
            }
        }

        public class Limit
        {
            public double lower;
            public double upper;
            public double effort;
            public double velocity;

            public Limit(XElement node)
            {
                lower = node.Attribute("lower").ReadOptionalDouble(); // optional
                upper = node.Attribute("upper").ReadOptionalDouble(); // optional
                effort = (double)node.Attribute("effort"); // required
                velocity = (double)node.Attribute("velocity"); // required
            }
        }

        public class Mimic
        {
            public string joint;
            public double multiplier;
            public double offset;

            public Mimic(XElement node)
            {
                joint = (string)node.Attribute("joint"); // required
                multiplier = node.Attribute("multiplier").ReadOptionalDouble(); // optional
                offset = node.Attribute("offset").ReadOptionalDouble(); // optional   
            }
        }

        public class SafetyController
        {
            public double softLowerLimit;
            public double softUpperLimit;
            public double kPosition;
            public double kVelocity;

            public SafetyController(XElement node)
            {
                softLowerLimit = node.Attribute("soft_lower_limit").ReadOptionalDouble(); // optional
                softUpperLimit = node.Attribute("soft_upper_limit").ReadOptionalDouble(); // optional
                kPosition = node.Attribute("k_position").ReadOptionalDouble(); // optional
                kVelocity = node.Attribute("k_velocity").ReadOptionalDouble(); // required   
            }
        }
    }
    
}
