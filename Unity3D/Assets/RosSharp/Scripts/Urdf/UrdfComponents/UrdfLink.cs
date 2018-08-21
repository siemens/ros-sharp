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
using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Export
{
    public class UrdfLink : MonoBehaviour
    {
        public UrdfLink AddChildLink()
        {
            //TODO: Check that link has a unique name
            GameObject childObject = new GameObject("Link");
            childObject.transform.SetParentAndAlign(gameObject.transform);

            UrdfLink childUrdfLink = childObject.AddComponent<UrdfLink>();

            return childUrdfLink;
        }

        public void AddChildLink(UrdfJoint.JointTypes jointType)
        {
            UrdfLink childLink = AddChildLink();

            //Todo: add appropriate type of joint as well
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
                Debug.LogWarning("Only visuals should be scaled. Scale on link \"" + gameObject.name + "\" cannot be saved to the URDF file.");

            Link link = new Link(gameObject.name)
            {
                visuals = gameObject.GetComponentInChildren<UrdfVisuals>().GetVisualsData(),
                collisions = gameObject.GetComponentInChildren<UrdfCollisions>().GetCollisionsData(),
                inertial = GetInertialData()
            };
            
            return link;
        }

        private Link.Inertial GetInertialData()
        {
            Rigidbody _rigidbody = GetComponent<Rigidbody>();
            if (_rigidbody == null)
                return null;

            //TODO: what to do for rotation values here?
            Origin inertialOrigin = new Origin(_rigidbody.centerOfMass.Unity2Ros().ToRoundedDoubleArray(), new double[] {0,0,0});
            
            //TODO: get actual values for inertial matrix
            return new Link.Inertial(_rigidbody.mass, inertialOrigin, new Link.Inertial.Inertia(0,0,0,0,0,0));
            
        }
    }
}