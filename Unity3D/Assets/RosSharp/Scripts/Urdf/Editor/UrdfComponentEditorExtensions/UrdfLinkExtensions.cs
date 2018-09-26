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

using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    public static class UrdfLinkExtensions
    { 
        public static UrdfLink Create(Transform parent, Link link = null, Joint joint = null)
        {
            GameObject linkObject = new GameObject("link");
            linkObject.transform.SetParentAndAlign(parent);
            UrdfLink urdfLink = linkObject.AddComponent<UrdfLink>();

            UrdfVisualsExtensions.Create(linkObject.transform, link?.visuals);
            UrdfCollisionsExtensions.Create(linkObject.transform, link?.collisions);
            
            if (link != null)
                urdfLink.ImportLinkData(link, joint);
            else
            {
                UrdfInertialExtensions.Create(linkObject);
                UnityEditor.EditorGUIUtility.PingObject(linkObject);
            }

            return urdfLink;
        }

        private static void ImportLinkData(this UrdfLink urdfLink, Link link, Joint joint)
        {
            if (link.inertial == null && joint == null)
                urdfLink.IsBaseLink = true;

            urdfLink.gameObject.name = link.name;

            if (joint?.origin != null)
                UrdfOrigin.ImportOriginData(urdfLink.transform, joint.origin);

            if (link.inertial != null)
            {
                UrdfInertialExtensions.Create(urdfLink.gameObject, link.inertial);

                if (joint != null)
                    UrdfJointExtensions.Create(urdfLink.gameObject, UrdfJointExtensions.GetJointType(joint.type), joint);
            }
            else if (joint != null)
                Debug.LogWarning("No Joint Component will be created in GameObject \"" + urdfLink.gameObject.name + "\" as it has no Rigidbody Component.\n"
                                 + "Please define an Inertial for Link \"" + link.name + "\" in the URDF file to create a Rigidbody Component.\n", urdfLink.gameObject);

            foreach (Joint childJoint in link.joints)
            {
                Link child = childJoint.ChildLink;
                UrdfLinkExtensions.Create(urdfLink.transform, child, childJoint);
            }
        } 
        
        public static Link ExportLinkData(this UrdfLink urdfLink)
        {
            if(urdfLink.transform.localScale != Vector3.one)
                Debug.LogWarning("Only visuals should be scaled. Scale on link \"" + urdfLink.gameObject.name + "\" cannot be saved to the URDF file.", urdfLink.gameObject);

            UrdfInertial urdfInertial = urdfLink.gameObject.GetComponent<UrdfInertial>();
            Link link = new Link(urdfLink.gameObject.name)
            {
                visuals = urdfLink.GetComponentInChildren<UrdfVisuals>().ExportVisualsData(),
                collisions = urdfLink.GetComponentInChildren<UrdfCollisions>().ExportCollisionsData(),
                inertial = urdfInertial == null ? null : urdfInertial.ExportInertialData()
            };
            
            return link;
        }
    }
}