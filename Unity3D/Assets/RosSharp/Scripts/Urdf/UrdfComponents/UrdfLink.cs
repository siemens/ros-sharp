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
        public enum UrdfJointTypes { Revolute, Continuous, Prismatic, Fixed, Floating, Planar }

        public UrdfLink AddChildLink()
        {
            //TODO: Check that link has a unique name
            GameObject childObject = new GameObject("Link");
            childObject.transform.SetParentAndAlign(gameObject.transform);

            UrdfLink childUrdfLink = childObject.AddComponent<UrdfLink>();
            childUrdfLink.Initialize();

            return childUrdfLink;
        }

        public void AddChildLink(UrdfJointTypes jointType)
        {
            UrdfLink childLink = AddChildLink();

            //Todo: add appropriate type of joint as well
            childLink.gameObject.AddComponent<UrdfJoint>().Initialize(gameObject.name + "_joint", jointType.ToString().ToLower());
        }

        public void Initialize()
        {
            AddVisuals();
            AddCollisions();

            gameObject.AddComponent<Rigidbody>();
            
            EditorGUIUtility.PingObject(gameObject);
        }

        private void AddVisuals()
        {
            GameObject visuals = new GameObject("Visuals");
            visuals.transform.SetParentAndAlign(gameObject.transform);
            visuals.AddComponent<UrdfVisuals>();
        }

        private void AddCollisions()
        {
            GameObject collisions = new GameObject("Collisions");
            collisions.transform.SetParentAndAlign(gameObject.transform);
            collisions.AddComponent<UrdfCollisions>();
        }

        public Link GetLinkData()
        {
            Link link = new Link(gameObject.name);

            UrdfVisual[] visuals = gameObject.GetComponentInChildren<UrdfVisuals>().GetComponentsInChildren<UrdfVisual>();
            foreach (UrdfVisual urdfVisual in visuals)
            {
                link.visuals.Add(urdfVisual.GetVisualData());
            }

            UrdfCollision[] collisions = gameObject.GetComponentInChildren<UrdfCollisions>().GetComponentsInChildren<UrdfCollision>();
            foreach (UrdfCollision urdfCollision in collisions)
            {
                link.collisions.Add(urdfCollision.GetCollisionData());
            }

            link.inertial = GetInertialData();
            return link;
        }

        private Link.Inertial GetInertialData()
        {
            Rigidbody rigidbody = GetComponent<Rigidbody>();
            if (rigidbody == null)
                return null;

            //TODO: get actual values for inertial matrix
            return new Link.Inertial(rigidbody.mass, transform.GetOriginData(), new Link.Inertial.Inertia(0,0,0,0,0,0));
            
        }
    }
}