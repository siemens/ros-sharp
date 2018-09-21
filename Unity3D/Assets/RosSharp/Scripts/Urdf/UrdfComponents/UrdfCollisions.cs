﻿/*
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

using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace RosSharp.Urdf
{ 
    public class UrdfCollisions : MonoBehaviour
    {
        public static void Create(Transform parent, List<Link.Collision> collisions = null)
        {
            GameObject collisionsObject = new GameObject("Collisions");
            collisionsObject.transform.SetParentAndAlign(parent);
            UrdfCollisions urdfCollisions = collisionsObject.AddComponent<UrdfCollisions>();

            collisionsObject.hideFlags = HideFlags.NotEditable;
            urdfCollisions.hideFlags = HideFlags.None;

            if (collisions != null)
            {
                foreach (Link.Collision collision in collisions)
                    UrdfCollision.Create(urdfCollisions.transform, collision);
            }
        }
        
        public List<Link.Collision> ExportCollisionsData()
        {
            UrdfCollision[] urdfCollisions = gameObject.GetComponentsInChildren<UrdfCollision>();
            return urdfCollisions.Select(urdfCollision => urdfCollision.ExportCollisionData()).ToList();
        }
    }
}