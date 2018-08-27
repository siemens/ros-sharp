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
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;
using MathNet.Numerics.LinearAlgebra.Single;
using UnityEngine;

namespace RosSharp.Urdf
{
    class UrdfInertial
    {
        private static readonly int RoundDigits = 6;

        public static void Create(GameObject linkObject, Link.Inertial inertial = null)
        {
            Rigidbody rigidbody = linkObject.AddComponent<Rigidbody>();

            if (inertial == null) return;

            rigidbody.mass = (float)inertial.mass;

            if (inertial.origin != null)
                rigidbody.centerOfMass = UrdfOrigin.GetPosition(inertial.origin);
                
            SetInertiaData(rigidbody, inertial.inertia);

            RigidbodyUrdfDataManager rigidbodyUrdfDataManager
                = linkObject.AddComponent<RigidbodyUrdfDataManager>();

            rigidbodyUrdfDataManager.GetValuesFromUrdf(
                rigidbody.centerOfMass,
                rigidbody.inertiaTensor,
                rigidbody.inertiaTensorRotation);
            rigidbodyUrdfDataManager.UseUrdfData = true;
        }
        
        #region SetInertiaData

        public static void SetInertiaData(Rigidbody rigidbody, Link.Inertial.Inertia inertia)
        {
            Evd<float> Evd = ToMatrix(Unity3DCoordTrafo(inertia)).Evd(Symmetricity.Symmetric);
            rigidbody.inertiaTensor = FixMinInertia(ToVector3(Evd.EigenValues.Real().ToSingle())); // optionally check vector for imaginary part = 0
            rigidbody.inertiaTensorRotation = ToQuaternion(Evd.EigenVectors); // optionally check matrix for determinant = 1
        }

        private static Link.Inertial.Inertia Unity3DCoordTrafo(Link.Inertial.Inertia inertia)
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

        private static Matrix ToMatrix(Link.Inertial.Inertia inertia)
        {
            return DenseMatrix.OfArray(
                new float[,]{
                    {(float)inertia.ixx,(float)inertia.ixy,(float)inertia.ixz },
                    {(float)inertia.ixy,(float)inertia.iyy,(float)inertia.iyz },
                    {(float)inertia.ixz,(float)inertia.iyz,(float)inertia.izz } }
            );
        }

        private const float minInertia = 1e-6f;

        private static Vector3 FixMinInertia(Vector3 vector3)
        {
            for (int i = 0; i < 3; i++)
                if (vector3[i] < minInertia)
                    vector3[i] = minInertia;
            return vector3;
        }

        private static Vector3 ToVector3(Vector<float> vector)
        {
            if (vector.Count != 3)
                throw new System.ArgumentException("Vector length must be 3.", "vector");

            return new Vector3(vector[0], vector[1], vector[2]);
            //return new Vector3(vector[0], vector[1], vector[2]).Ros2Unity();
        }

        private static Quaternion ToQuaternion(Matrix<float> matrix)
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

        #endregion
        
        #region GetInertiaData
        public static Link.Inertial GetInertialData(Rigidbody _rigidbody)
        {
            if (_rigidbody == null)
                return null;

            Origin inertialOrigin = new Origin(_rigidbody.centerOfMass.Unity2Ros().ToRoundedDoubleArray(), new double[] { 0, 0, 0 });
            Link.Inertial.Inertia inertia = GetInertiaData(_rigidbody);

            //TODO: test inertia export more thoroughly. Check to see if values are the same when re-importing
            return new Link.Inertial(_rigidbody.mass, inertialOrigin, inertia);
        }

        private static Link.Inertial.Inertia GetInertiaData(Rigidbody _rigidbody)
        {
            double[][] lamdaMatrix =
                { new double[] { _rigidbody.inertiaTensor[0], 0, 0 },
                  new double[] { 0, _rigidbody.inertiaTensor[1], 0 },
                  new double[] { 0, 0, _rigidbody.inertiaTensor[2] } };

            double[][] qMatrix = Quaternion2Matrix(_rigidbody.inertiaTensorRotation);

            double[][] qMatrixTransposed = new double[3][];
            for (int i = 0; i < 3; i++)
            {
                qMatrixTransposed[i] = new double[3];
                for (int j = 0; j < 3; j++)
                    qMatrixTransposed[i][j] = qMatrix[j][i];
            }

            //matrix multiplication (inertia matrix = qMatrix X lambdaMatrix X qMatrixTransposed)
            double[][] tempMatrix = new double[3][];
            for (int i = 0; i < 3; i++)
            {
                tempMatrix[i] = new double[3];
                for (int j = 0; j < 3; j++)
                    tempMatrix[i][j] = DotProduct(qMatrix[i], lamdaMatrix[j]);
            }

            double[][] inertiaMatrix = new double[3][];
            for (int i = 0; i < 3; i++)
            {
                inertiaMatrix[i] = new double[3];
                for (int j = 0; j < 3; j++)
                    inertiaMatrix[i][j] = Math.Round(DotProduct(tempMatrix[i], qMatrixTransposed[j]), RoundDigits);
            }

            return MatrixToRosInertia(inertiaMatrix);
        }

        private static double[][] Quaternion2Matrix(Quaternion quaternion)
        {
            //Quaternion rosQuaternion = quaternion.Unity2Ros();
            Quaternion rosQuaternion = Quaternion.Normalize(quaternion);

            double[][] rotationMatrix = { new double[3], new double[3], new double[3] };

            rotationMatrix[0][0] = 1 - 2 * Math.Pow(rosQuaternion.y, 2) - 2 * Math.Pow(rosQuaternion.z, 2);
            rotationMatrix[0][1] = 2 * rosQuaternion.x * rosQuaternion.y - 2 * rosQuaternion.z * rosQuaternion.w;
            rotationMatrix[0][2] = 2 * rosQuaternion.x * rosQuaternion.z + 2 * rosQuaternion.y * rosQuaternion.w;

            rotationMatrix[1][0] = 2 * rosQuaternion.x * rosQuaternion.y + 2 * rosQuaternion.z * rosQuaternion.w;
            rotationMatrix[1][1] = 1 - 2 * Math.Pow(rosQuaternion.x, 2) - 2 * Math.Pow(rosQuaternion.z, 2);
            rotationMatrix[1][2] = 2 * rosQuaternion.y * rosQuaternion.z - 2 * rosQuaternion.x * rosQuaternion.w;

            rotationMatrix[2][0] = 2 * rosQuaternion.x * rosQuaternion.z - 2 * rosQuaternion.y * rosQuaternion.w;
            rotationMatrix[2][0] = 2 * rosQuaternion.y * rosQuaternion.z + 2 * rosQuaternion.x * rosQuaternion.w;
            rotationMatrix[2][2] = 1 - 2 * Math.Pow(rosQuaternion.x, 2) - 2 * Math.Pow(rosQuaternion.y, 2);

            return rotationMatrix;
        }

        private static double DotProduct(double[] a, double[] b)
        {
            return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        }

        private static Link.Inertial.Inertia MatrixToRosInertia(double[][] inertiaMatrix)
        {
            Link.Inertial.Inertia inertia = new Link.Inertial.Inertia(0, 0, 0, 0, 0, 0)
            {
                ixx = inertiaMatrix[2][2],
                iyy = inertiaMatrix[0][0],
                izz = inertiaMatrix[1][1],
                ixy = inertiaMatrix[0][2],
                ixz = inertiaMatrix[1][2],
                iyz = inertiaMatrix[0][1]
            };

            return inertia;
        }
        #endregion
    }
}
