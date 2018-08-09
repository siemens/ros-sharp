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
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
using MathNet.Numerics.LinearAlgebra.Factorization;

namespace RosSharp.Urdf.Import
{

    public static class UrdfLinkInertialInertiaExtensions
    {
        public static void SetInertia(this Link.Inertial.Inertia inertia, Rigidbody rigidbody)
        {
            Evd<float> Evd = inertia.Unity3DCoordTrafo().ToMatrix().Evd(Symmetricity.Symmetric);
            rigidbody.inertiaTensor = Evd.EigenValues.Real().ToSingle().ToVector3().FixMinInertia(); // optionally check vector for imaginary part = 0
            rigidbody.inertiaTensorRotation = Evd.EigenVectors.ToQuaternion(); // optionally check matrix for determinant = 1
        }

        private static Link.Inertial.Inertia Unity3DCoordTrafo(this Link.Inertial.Inertia inertia)
        {
            Link.Inertial.Inertia unity3DInertia = inertia;
            unity3DInertia.ixx = inertia.iyy;
            unity3DInertia.iyy = inertia.izz;
            unity3DInertia.izz = inertia.ixx;

            unity3DInertia.ixy = inertia.iyz;
            unity3DInertia.ixz = inertia.ixy;
            unity3DInertia.iyz = inertia.ixz;
            return unity3DInertia;
        }
        private static Matrix ToMatrix(this Link.Inertial.Inertia inertia)
        {
            return DenseMatrix.OfArray(
                new float[,]{
                    {(float)inertia.ixx,(float)inertia.ixy,(float)inertia.ixz },
                    {(float)inertia.ixy,(float)inertia.iyy,(float)inertia.iyz },
                    {(float)inertia.ixz,(float)inertia.iyz,(float)inertia.izz } }
            );
        }

        private const float minInertia = 1e-6f;

        private static Vector3 FixMinInertia(this Vector3 vector3)
        {
            for (int i = 0; i < 3; i++)
                if (vector3[i] < minInertia)
                    vector3[i] = minInertia;
            return vector3;
        }

        public static Vector3 ToVector3(this Vector<float> vector)
        {
            if (vector.Count != 3)
                throw new System.ArgumentException("Vector length must be 3.", "vector");

            return new Vector3(vector[0], vector[1], vector[2]);
        }

        public static Quaternion ToQuaternion(this Matrix<float> matrix)
        {
            if (matrix.RowCount != 3 || matrix.ColumnCount != 3)
                throw new System.ArgumentException("Matrix must be 3x3.", "matrix");

            float w = Mathf.Sqrt(1f + matrix[0, 0] + matrix[1, 1] + matrix[2, 2]) / 2f;
            float w4 = 4 * w;

            return new Quaternion(
                matrix[2, 1] - matrix[1, 2] / w4,
                matrix[0, 2] - matrix[2, 0] / w4,
                matrix[1, 0] - matrix[0, 1] / w4,
                w);
        }

    }
}

