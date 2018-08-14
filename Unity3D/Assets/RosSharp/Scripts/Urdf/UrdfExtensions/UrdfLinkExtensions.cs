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

using UnityEngine;

namespace RosSharp.Urdf.Import
{
    public static class UrdfLinkExtensions
    {
        public static GameObject Create(this Link link, GameObject parent, Joint joint = null)
        {
            GameObject gameObject = new GameObject(link.name);
            gameObject.transform.SetParentAndAlign(parent.transform);

            joint?.origin?.SetTransform(gameObject);

            if (link.inertial != null)
            {
                link.inertial.Create(gameObject);
                joint?.Create(gameObject, parent);
            }
            else if (joint != null)
                Debug.LogWarning("No Joint Component will be created in GameObject \"" + gameObject.name + "\" as it has no Rigidbody Component.\n"
                    + "Please define an Inertial for Link \"" + link.name + "\" in the URDF file to create a Rigidbody Component.\n");

            GameObject visualGameObject = new GameObject("Visuals");
            visualGameObject.transform.SetParentAndAlign(gameObject.transform);
            foreach (Link.Visual visual in link.visuals)
                visual.Create(visualGameObject);

            GameObject collisionGameObject = new GameObject("Collisions");
            collisionGameObject.transform.SetParentAndAlign(gameObject.transform);
            foreach (Link.Collision collision in link.collisions)
                collision.Create(collisionGameObject);

            foreach (Joint childJoint in link.joints)
            {
                Link child = childJoint.ChildLink;
                child.Create(gameObject, childJoint);
            }
            return gameObject;
        }
    }

    public static class UrdfLinkInertialExtensions
    {
        public static Rigidbody Create(this Link.Inertial inertial, GameObject gameObject)
        {
            Rigidbody rigidbody = gameObject.AddComponent<Rigidbody>();
            rigidbody.mass = (float)inertial.mass;

            if (inertial.origin != null)
                rigidbody.centerOfMass = inertial.origin.GetPosition();

            inertial.inertia.SetInertia(rigidbody);

            RigidbodyUrdfDataManager rigidbodyUrdfDataManager
                = gameObject.AddComponent<RigidbodyUrdfDataManager>();

            rigidbodyUrdfDataManager.GetValuesFromUrdf(
                rigidbody.centerOfMass,
                rigidbody.inertiaTensor,
                rigidbody.inertiaTensorRotation);
            rigidbodyUrdfDataManager.UseUrdfData = true;

            return rigidbody;
        }
    }

    public static class UrdfLinkCollisionExtensions
    {
        public static GameObject Create(this Link.Collision collision, GameObject parent)
        {
            GameObject gameObject = new GameObject(collision.name ?? "unnamed");
            gameObject.transform.SetParentAndAlign(parent.transform);

            collision.origin?.SetTransform(gameObject);

            collision.geometry.CreateCollider(gameObject);

            return gameObject;
        }
    }

    public static class UrdfLinkVisualExtensions
    {
        public static GameObject Create(this Link.Visual visual, GameObject parent)
        {
            GameObject gameObject = new GameObject(visual.name ?? "unnamed");
            gameObject.transform.SetParentAndAlign(parent.transform);

            visual.origin?.SetTransform(gameObject);

            visual.geometry.CreateVisual(gameObject);

            UrdfMaterialHandler.SetUrdfMaterial(gameObject, visual.material);

            return gameObject;
        }
    }
}
