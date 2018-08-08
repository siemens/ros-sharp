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

using RosSharp;
using RosSharp.UrdfImporter;
using System.IO;
using UnityEngine;
using Joint = RosSharp.UrdfImporter.Joint;

namespace UrdfExport.UrdfComponents
{
    public class UrdfRobot : MonoBehaviour
    {
        public void ExportRobotToUrdf()
        {
            string newRobotAssetPath = Application.dataPath + "\\Urdf\\" + name;
            Directory.CreateDirectory(newRobotAssetPath);
            Debug.Log("exporting " + gameObject.name + " to URDF,  at" + newRobotAssetPath);

            Robot robot = new Robot(newRobotAssetPath + "\\TESTING_EXPORT.urdf", gameObject.name);

            //Test material library
            Link.Visual.Material material = new Link.Visual.Material("blue", new Link.Visual.Material.Color(new double[] {0, 0, 1, 1}));
            robot.materials.Add(material);

            //Test link
            Link link = new Link("test_link_1", new Link.Inertial(
                111.0, 
                new Origin(new double[] {0, 1, 2}, 
                            new double[] { 0, 1, 2 }),
                new Link.Inertial.Inertia(0.01, 0.01, 0.01, 0.01, 0.01, 0.01)
                ));

            link.visuals.Add(new Link.Visual(
                new Link.Geometry(new Link.Geometry.Box(new double[] {4, 1, 2})),
                null,
                new Origin(new double[] { 0, 0, 0 },
                           new double[] { 0, 1, 1 }),
                new Link.Visual.Material("blue"))
            );

            link.collisions.Add(new Link.Collision(
                new Link.Geometry(new Link.Geometry.Box(new double[] {4, 1, 2})))
            );

            robot.links.Add(link);

            //Test second link
            robot.links.Add(new Link("test_link_2"));
            
            //Test joint
            Joint joint = new Joint(
                "test_joint_1", 
                "fixed", 
                "test_link_1",
                "test_link_2", 
                new Origin(new double[] { 0, 1, 2 },
                           new double[] { 0, 1, 2 }),
                new Joint.Axis(new[] {0.1, 0.2, 0.3}),
                new Joint.Calibration(0.1),
                new Joint.Dynamics(0.4, 0.000001),
                new Joint.Limit(0, 0, 3, 10),
                new Joint.Mimic("test_joint_1"),
                new Joint.SafetyController(-10, 10, 0, 2)
            );
            
            robot.joints.Add(joint);
            
            //Test export to URDF
            robot.WriteToUrdf();
        }

        public void InitializeRobot()
        {
            GameObject baseLink = new GameObject("base_link");
            baseLink.transform.SetParentAndAlign(gameObject.transform);

            baseLink.AddComponent<UrdfLink>();
        }
        
    }
}
