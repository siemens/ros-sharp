/*
© Siemens AG, 2019
Author: Sifan Ye (sifan.ye@siemens.com)

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
using UnityEditor;
using UnityEngine;

namespace RosSharp.RosBridgeClient.MessageGeneration
{
    public class PackageSrvAutoGenEditorWindow : PackageAutoGenEditorWindow
    {
        protected override string GenerationType
        {
            get { return "service"; }
        }

        protected override string FileExtension
        {
            get { return "srv"; }
        }

        [MenuItem("RosBridgeClient/Auto Generate Services/Package Services...", false, 11)]
        private static void OpenWindow()
        {
            PackageSrvAutoGenEditorWindow window = GetWindow<PackageSrvAutoGenEditorWindow>(false, "Service Auto Generation", true);
            window.minSize = new Vector2(750, 100);
            window.maxSize = new Vector2(750, 100);
            window.Show();
        }

        protected override List<string> Generate(string inPath, string outPath, bool isRos2, string rosPackageName = "")
        {
            //todo: Do I need this?
            return ServiceAutoGen.GenerateSingleService(inPath, outPath, rosPackageName);
        }
    }
}