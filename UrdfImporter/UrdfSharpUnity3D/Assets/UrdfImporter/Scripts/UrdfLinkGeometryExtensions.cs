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

namespace Urdf
{
    public static class UrdfLinkGeometryExtensions
    {
        public static GameObject CreateVisual(this Link.Geometry geometry, GameObject gameObject)
        {
            if (geometry.box != null)
                return geometry.box.CreateVisual(gameObject);
            else if (geometry.cylinder != null)
                return geometry.cylinder.CreateVisual(gameObject);
            else if (geometry.sphere != null)
                return geometry.sphere.CreateVisual(gameObject);
            else if (geometry.mesh != null)
                return geometry.mesh.CreateVisual(gameObject);
            else
                return null;
        }

        public static GameObject CreateCollider(this Link.Geometry geometry, GameObject gameObject)
        {
            if (geometry.box != null)
                return geometry.box.CreateCollider(gameObject);
            else if (geometry.cylinder != null)
                return geometry.cylinder.CreateCollider(gameObject);
            else if (geometry.sphere != null)
                return geometry.sphere.CreateCollider(gameObject);
            else if (geometry.mesh != null)
                return geometry.mesh.CreateCollider(gameObject);
            else
                return null;
        }
    }
}