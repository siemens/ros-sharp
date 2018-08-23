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
using UnityEngine;

namespace RosSharp.Urdf.Export
{
    class UrdfInertial
    {
        private static readonly int RoundDigits = 6;
        private Rigidbody _rigidbody;

        public static Link.Inertial GetInertialData(Rigidbody _rigidbody)
        {
            if (_rigidbody == null)
                return null;

            Origin inertialOrigin = new Origin(_rigidbody.centerOfMass.Unity2Ros().ToRoundedDoubleArray(), new double[] { 0, 0, 0 });
            Link.Inertial.Inertia inertia = GetInertiaData(_rigidbody);

            //TODO: test import and export
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
    }
}
