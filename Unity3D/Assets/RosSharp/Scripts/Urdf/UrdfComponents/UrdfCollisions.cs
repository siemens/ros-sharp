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

using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace RosSharp.Urdf.Export
{
    public class UrdfCollisions : MonoBehaviour
    {
        public void Reset()
        {
            transform.DestroyChildrenImmediate();

            gameObject.hideFlags = HideFlags.NotEditable;
            hideFlags = HideFlags.None;
        }

        public void AddColision(UrdfGeometry.GeometryTypes type, Transform visualTransform = null)
        {
            GameObject collision = new GameObject("unnamed");
            collision.transform.SetParentAndAlign(gameObject.transform);

            collision.AddComponent<UrdfCollision>().Initialize(type, visualTransform);
        }

        public List<Link.Collision> GetCollisionsData()
        {
            UrdfCollision[] urdfCollisions = gameObject.GetComponentsInChildren<UrdfCollision>();

            return urdfCollisions.Select(urdfCollision => urdfCollision.GetCollisionData()).ToList();
        }
    }
}