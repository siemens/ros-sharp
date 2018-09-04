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

using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf
{
    public class UrdfLink : MonoBehaviour
    {
        public static UrdfLink Create(Transform parent, Link link = null, Joint joint = null)
        {
            GameObject linkObject = new GameObject("link");
            linkObject.transform.SetParentAndAlign(parent);
            UrdfLink urdfLink = linkObject.AddComponent<UrdfLink>();

            UrdfVisuals.Create(linkObject.transform, link?.visuals);
            UrdfCollisions.Create(linkObject.transform, link?.collisions);
            
            if (link != null)
            {
                linkObject.name = link.name;

                if (joint?.origin != null)
                    UrdfOrigin.SetTransform(urdfLink.transform, joint.origin);

                if (link.inertial != null)
                {
                    UrdfInertial.Create(linkObject, link.inertial);

                    if(joint != null)
                        UrdfJoint.Create(linkObject, joint);
                }
                else if (joint != null)
                    Debug.LogWarning("No Joint Component will be created in GameObject \"" + urdfLink.name + "\" as it has no Rigidbody Component.\n"
                                     + "Please define an Inertial for Link \"" + link.name + "\" in the URDF file to create a Rigidbody Component.\n", linkObject);

                foreach (Joint childJoint in link.joints)
                {
                    Link child = childJoint.ChildLink;
                    UrdfLink.Create(urdfLink.transform, child, childJoint);
                }
            }
            else
                UrdfInertial.Create(linkObject);
            
            EditorGUIUtility.PingObject(linkObject);

            return urdfLink;
        }
        
        public Link GetLinkData()
        {
            if(transform.localScale != Vector3.one)
                Debug.LogWarning("Only visuals should be scaled. Scale on link \"" + gameObject.name + "\" cannot be saved to the URDF file.", gameObject);

            Link link = new Link(gameObject.name)
            {
                visuals = gameObject.GetComponentInChildren<UrdfVisuals>().GetVisualsData(),
                collisions = gameObject.GetComponentInChildren<UrdfCollisions>().GetCollisionsData(),
                inertial = gameObject.GetComponent<UrdfInertial>().GetInertialData()
            };
            
            return link;
        }
    }
}