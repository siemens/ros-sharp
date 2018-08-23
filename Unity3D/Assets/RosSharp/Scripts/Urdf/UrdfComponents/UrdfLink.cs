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

namespace RosSharp.Urdf.Export
{
    public class UrdfLink : MonoBehaviour
    {
        public UrdfLink AddChildLink()
        {
            GameObject childObject = new GameObject("link");
            childObject.transform.SetParentAndAlign(gameObject.transform);

            UrdfLink childUrdfLink = childObject.AddComponent<UrdfLink>();

            return childUrdfLink;
        }

        public void AddChildLink(UrdfJoint.JointTypes jointType)
        {
            UrdfLink childLink = AddChildLink();
            childLink.gameObject.AddComponent<UrdfJoint>().Initialize(gameObject.name + "_joint", jointType.ToString().ToLower());
        }

        public void Reset()
        {
            transform.DestroyChildrenImmediate();

            AddVisualsObject();
            AddCollisionsObject();

            transform.DestroyImmediateIfExists<Rigidbody>();
            gameObject.AddComponent<Rigidbody>();
            
            EditorGUIUtility.PingObject(gameObject);
        }

        //TODO: Add ability to reset and/or change type of joint

        private void AddVisualsObject()
        {
            GameObject visualsObject = new GameObject("Visuals");
            visualsObject.transform.SetParentAndAlign(gameObject.transform);
            visualsObject.AddComponent<UrdfVisuals>();
        }

        private void AddCollisionsObject()
        {
            GameObject collisionsObject = new GameObject("Collisions");
            collisionsObject.transform.SetParentAndAlign(gameObject.transform);
            collisionsObject.AddComponent<UrdfCollisions>();
        }

        public Link GetLinkData()
        {
            if(transform.localScale != Vector3.one)
                Debug.LogWarning("Only visuals should be scaled. Scale on link \"" + gameObject.name + "\" cannot be saved to the URDF file.", gameObject);

            Link link = new Link(gameObject.name)
            {
                visuals = gameObject.GetComponentInChildren<UrdfVisuals>().GetVisualsData(),
                collisions = gameObject.GetComponentInChildren<UrdfCollisions>().GetCollisionsData(),
                inertial = UrdfInertial.GetInertialData(GetComponent<Rigidbody>())
            };
            
            return link;
        }
    }
}