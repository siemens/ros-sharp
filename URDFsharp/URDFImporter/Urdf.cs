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


using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Xml.Linq;

using UnityEngine;
using UnityEditor;
using System;

namespace Urdf
{
    public class Model
    {
        public string Name;
        public Link Root;
        public List<UrdfMaterial> MaterialList;

        private string assetPath;

        public Model(string urdfFile)
        {
            assetPath = Path.GetDirectoryName(urdfFile).getAssetPath();
            XDocument xdoc = XDocument.Load(urdfFile);
            XElement node = xdoc.Element("robot");
            Name = node.Attribute("name").Value;

            MaterialList = readMaterials(node); // multiple
            List<Link> Links = readLinks(node); // multiple
            List<Joint> Joints = readJoints(node); // multiple

            // build tree structure from link and joint lists:
            foreach (Link link in Links)
                link.ChildJoints = Joints.FindAll(v => v.Parent == link.Name);
            foreach (Joint joint in Joints)
                joint.ChildLink = Links.Find(v => v.Name == joint.Child);

            // save root node only:
            Root = findRootLink(Links, Joints);
        }

        public string getAssetPath(string PackagePath)
        {
            return Path.Combine(assetPath, PackagePath.Substring(10).Replace(Path.AltDirectorySeparatorChar, Path.DirectorySeparatorChar));
        }
        public string getAssetPath()
        {
            return assetPath;
        }

        public static GameObject CreateModel(string urdfFile)
        {
            Model model = new Model(urdfFile);
            return model.Create();
        }

        public GameObject Create()
        {
            if (!AssetDatabase.IsValidFolder(Path.Combine(assetPath, "Materials")))
                AssetDatabase.CreateFolder(assetPath, "Materials");

            UrdfMaterial.CreateDefaultMaterialAsset(assetPath);
            foreach (UrdfMaterial material in MaterialList)
                material.CreateMaterialAsset(assetPath);

            GameObject gameObject = new GameObject(Name);
            Root.Create(this, gameObject);

            GameObjectUtility.SetParentAndAlign(gameObject, Selection.activeObject as GameObject);
            Undo.RegisterCreatedObjectUndo(gameObject, "Create " + gameObject.name);
            Selection.activeObject = gameObject;

            Rigidbody[] rb = Rigidbody.FindObjectsOfType(typeof(Rigidbody)) as Rigidbody[];
            foreach (Rigidbody body in rb)
            {
                body.isKinematic = true;
            }

            return gameObject;
        }

        private static List<UrdfMaterial> readMaterials(XElement node)
        {
            var materials =
                from child in node.Elements("material")
                select new UrdfMaterial(child);
            return materials.ToList();
        }

        private static List<Link> readLinks(XElement node)
        {
            var links =
                from child in node.Elements("link")
                select new Link(child);
            return links.ToList();
        }

        private static List<Joint> readJoints(XElement node)
        {
            var joints =
                from child in node.Elements("joint")
                select new Joint(child);
            return joints.ToList();
        }

        private static Link findRootLink(List<Link> Links, List<Joint> Joints, int startIdx = 0)
        {
            Joint joint = Joints[0];
            string parent;
            do
            {
                parent = joint.Parent;
                joint = Joints.Find(v => v.Child == parent);
            }
            while (joint != null);
            return Links.Find(v => v.Name == parent);
        }
    }

    public class Origin
    {
        public double[] Xyz;
        public double[] Rpy;


        public Origin(XElement node)
        {
            Xyz = node.Attribute("xyz") != null ? node.Attribute("xyz").ReadDoubleArray() : null;

            Rpy = node.Attribute("rpy") != null ? node.Attribute("rpy").ReadDoubleArray() : null;
        }

        public Vector3 translation()
        {
            return new Vector3(
                (float)-Xyz[1],
                (float)Xyz[2],
                (float)Xyz[0]);
        }
        public Vector3 rotation()
        {
            return new Vector3(
                ((float)Rpy[1]) * Mathf.Rad2Deg,
                (-(float)Rpy[2]) * Mathf.Rad2Deg,
                (-(float)Rpy[0]) * Mathf.Rad2Deg);
        }
        public void SetOrigin(GameObject gameObject)
        {
            if (Xyz != null)
                gameObject.transform.Translate(translation());
            if (Rpy != null)
                gameObject.transform.Rotate(rotation());
        }
    }

    #region Joint Classes

    public class Joint
    {
        public string Name;
        public string Type;
        public Origin Origin;
        public string Parent;
        public string Child;
        public Axis Axis;
        public Calibration Calibration;
        public Dynamics Dynamics;
        public Limit Limit;
        public Mimic Mimic;
        public SafetyController SafetyController;

        public Link ChildLink;

        public Joint(XElement node)
        {
            Name = (string)node.Attribute("name"); // required
            Type = (string)node.Attribute("type"); // required
            Origin = (node.Element("origin") != null) ? new Origin(node.Element("origin")) : null; // optional  
            Parent = (string)node.Element("parent").Attribute("link"); // required
            Child = (string)node.Element("child").Attribute("link"); // required
            Axis = (node.Element("axis") != null) ? new Axis(node.Element("axis")) : null;  // optional 
            Calibration = (node.Element("calibration") != null) ? new Calibration(node.Element("calibration")) : null;  // optional 
            Dynamics = (node.Element("dynamics") != null) ? new Dynamics(node.Element("dynamics")) : null;  // optional 
            Limit = (node.Element("limit") != null) ? new Limit(node.Element("limit")) : null;  // required only for revolute and prismatic joints
            Mimic = (node.Element("mimic") != null) ? new Mimic(node.Element("mimic")) : null;  // optional
            SafetyController = (node.Element("safety_controller") != null) ? new SafetyController(node.Element("safety_controller")) : null;  // optional
        }

        public Vector3 GetConnectedAnchor()
        {
            if (Origin != null)
                return Origin.translation(); // wo muss Origin.rotation() hin?
            else
                return Vector3.zero;
        }

        public FixedJoint CreateFixedJoint(GameObject gameObject, Rigidbody parentRigidbody)
        {
            FixedJoint joint = gameObject.AddComponent<FixedJoint>();
            joint.connectedBody = parentRigidbody;
            return joint;
        }

        public HingeJoint CreateHingeJoint(GameObject gameObject, Rigidbody parentRigidbody)
        {
            HingeJoint joint = gameObject.AddComponent<HingeJoint>();
            joint.connectedBody = parentRigidbody;

            // axis:
            joint.axis = (Axis != null) ? Axis.GetAxis() : Axis.GetDefaultAxis();

            // origin:
            joint.autoConfigureConnectedAnchor = false;
            joint.connectedAnchor = GetConnectedAnchor();

            // spring, damper & position:
            if (Dynamics != null)
                joint.spring = Dynamics.GetJointSpring();

            // limits:        
            if (Type == "revolute" && Limit != null)
                joint.limits = Limit.GetJointLimits();

            return joint;
        }

        public ConfigurableJoint CreateFloatingJoint(GameObject gameObject, Rigidbody parentRigidbody)
        {
            ConfigurableJoint joint = gameObject.AddComponent<ConfigurableJoint>();
            joint.connectedBody = parentRigidbody;
            // origin:
            joint.autoConfigureConnectedAnchor = false;
            joint.connectedAnchor = GetConnectedAnchor();
            return joint;
        }

        public ConfigurableJoint CreatePrismaticJoint(GameObject gameObject, Rigidbody parentRigidbody)
        {
            ConfigurableJoint joint = gameObject.AddComponent<ConfigurableJoint>();
            joint.connectedBody = parentRigidbody;

            joint.axis = (Axis != null) ? Axis.GetAxis() : Axis.GetDefaultAxis();

            // degrees of freedom:
            joint.xMotion = ConfigurableJointMotion.Limited;
            joint.yMotion = ConfigurableJointMotion.Locked;
            joint.zMotion = ConfigurableJointMotion.Locked;
            joint.angularXMotion = ConfigurableJointMotion.Locked;
            joint.angularYMotion = ConfigurableJointMotion.Locked;
            joint.angularZMotion = ConfigurableJointMotion.Locked;

            // origin:
            joint.autoConfigureConnectedAnchor = false;
            joint.connectedAnchor = GetConnectedAnchor();

            // spring, damper & max. force:
            if (Dynamics != null)
                joint.xDrive = Dynamics.GetJointDrive();

            // limits:
            if (Limit != null)
            {
                joint.lowAngularXLimit = Limit.GetLowSoftJointLimit();
                joint.highAngularXLimit = Limit.GetHighSoftJointLimit();
            }
            return joint;
        }

        public ConfigurableJoint CreatePlanarJoint(GameObject gameObject, Rigidbody parentRigidbody)
        {
            ConfigurableJoint joint = gameObject.AddComponent<ConfigurableJoint>();
            joint.connectedBody = parentRigidbody;

            Vector3 normal = (Axis != null) ? Axis.GetAxis() : Axis.GetDefaultAxis();
            Vector3 axisX = Vector3.forward;
            Vector3 axisY = Vector3.left;
            Vector3.OrthoNormalize(ref normal, ref axisX, ref axisY);
            joint.axis = axisX;
            joint.secondaryAxis = axisY;

            // degrees of freedom:
            joint.xMotion = ConfigurableJointMotion.Free;
            joint.yMotion = ConfigurableJointMotion.Free;
            joint.zMotion = ConfigurableJointMotion.Locked;
            joint.angularXMotion = ConfigurableJointMotion.Locked;
            joint.angularYMotion = ConfigurableJointMotion.Locked;
            joint.angularZMotion = ConfigurableJointMotion.Locked;

            // origin:
            joint.autoConfigureConnectedAnchor = false;
            joint.connectedAnchor = GetConnectedAnchor();

            // spring, damper & max. force:
            if (Dynamics != null)
            {
                joint.xDrive = Dynamics.GetJointDrive();
                joint.yDrive = Dynamics.GetJointDrive();
            }
            return joint;
        }

        public UnityEngine.Joint Create(Model model, GameObject gameObject, GameObject parent)
        {
            Rigidbody parentRigidbody = parent.GetComponent<Rigidbody>();
            if (parentRigidbody == null)
                return null;

            gameObject.name = gameObject.name + " (" + Type + " Joint: " + Name + ")";

            if (Type == "fixed")
                return CreateFixedJoint(gameObject, parentRigidbody);
            if (Type == "continuous" || Type == "revolute")
                return CreateHingeJoint(gameObject, parentRigidbody);
            if (Type == "floating")
                return CreateFloatingJoint(gameObject, parentRigidbody);
            if (Type == "prismatic")
                return CreatePrismaticJoint(gameObject, parentRigidbody);
            if (Type == "planar")
                return CreatePlanarJoint(gameObject, parentRigidbody);
            return null;

        }
    }

    public class Axis
    {
        public double[] Xyz;

        public Axis(XElement node)
        {
            Xyz = node.Attribute("xyz") != null ? node.Attribute("xyz").ReadDoubleArray() : null;

        }
        public Vector3 GetAxis()
        {
            return new Vector3(
                (float)-Xyz[1],
                (float)Xyz[2],
                (float)Xyz[0]);
        }
        public static Vector3 GetDefaultAxis()
        {
            return new Vector3(-1, 0, 0);
        }
    }

    public class Calibration
    {
        public double Rising;
        public double Falling;

        public Calibration(XElement node)
        {
            Rising = node.Attribute("rising").ReadOptionalDouble();  // optional
            Falling = node.Attribute("falling").ReadOptionalDouble();  // optional
        }
    }

    public class Dynamics
    {
        public double Damping;
        public double Friction;

        public Dynamics(XElement node)
        {
            Damping = node.Attribute("damping").ReadOptionalDouble(); // optional
            Friction = node.Attribute("friction").ReadOptionalDouble(); // optional
        }
        public JointDrive GetJointDrive()
        {
            JointDrive jointDrive = new JointDrive();
            jointDrive.maximumForce = float.MaxValue;
            jointDrive.positionDamper = (float)Damping;
            jointDrive.positionSpring = (float)Friction;
            return jointDrive;
        }
        public JointSpring GetJointSpring()
        {
            JointSpring jointSpring = new JointSpring();
            jointSpring.damper = (float)Damping;
            jointSpring.spring = (float)Friction;
            jointSpring.targetPosition = 0;
            return jointSpring;
        }
    }

    public class Limit
    {
        public double Lower;
        public double Upper;
        public double Effort;
        public double Velocity;

        public Limit(XElement node)
        {
            Lower = node.Attribute("lower").ReadOptionalDouble(); // optional
            Upper = node.Attribute("upper").ReadOptionalDouble(); // optional
            Effort = (double)node.Attribute("effort"); // required
            Velocity = (double)node.Attribute("velocity"); // required
        }

        public JointLimits GetJointLimits()
        {
            JointLimits jointLimits = new JointLimits();
            jointLimits.min = (float)Lower;
            jointLimits.max = (float)Upper;
            return jointLimits;
        }

        public SoftJointLimit GetLowSoftJointLimit()
        {
            SoftJointLimit softJointLimit = new SoftJointLimit();
            softJointLimit.limit = (float)Lower * Mathf.Rad2Deg;
            return softJointLimit;
        }
        public SoftJointLimit GetHighSoftJointLimit()
        {
            SoftJointLimit softJointLimit = new SoftJointLimit();
            softJointLimit.limit = (float)Upper * Mathf.Rad2Deg;
            return softJointLimit;
        }
    }

    public class Mimic
    {
        public string Joint;
        public double Multiplier;
        public double Offset;

        public Mimic(XElement node)
        {
            Joint = (string)node.Attribute("joint"); // required
            Multiplier = node.Attribute("multiplier").ReadOptionalDouble(); // optional
            Offset = node.Attribute("offset").ReadOptionalDouble(); // optional   
        }
    }

    public class SafetyController
    {
        public double SoftLowerLimit;
        public double SoftUpperLimit;
        public double KPosition;
        public double KVelocity;

        public SafetyController(XElement node)
        {
            SoftLowerLimit = node.Attribute("soft_lower_limit").ReadOptionalDouble(); // optional
            SoftUpperLimit = node.Attribute("soft_upper_limit").ReadOptionalDouble(); // optional
            KPosition = node.Attribute("k_position").ReadOptionalDouble(); // optional
            KVelocity = node.Attribute("k_velocity").ReadOptionalDouble(); // required   
        }
    }

    #endregion

    #region Link Classes

    public class Link
    {
        public string Name;
        public Inertial Inertial;
        public List<Visual> Visuals;
        public List<Collision> Collisions;
        public List<Joint> ChildJoints;

        public Link(XElement node)
        {
            Name = (string)node.Attribute("name");  // required
            Inertial = (node.Element("inertial") != null) ? new Inertial(node.Element("inertial")) : null;  // optional     
            Visuals = readVisuals(node); // multiple
            Collisions = readCollisions(node);// (node.Element("collision") != null) ? new Collision(node.Element("collision")) : null;  // optional   
        }

        public GameObject Create(Model model, GameObject parent, Joint joint = null)
        {
            GameObject gameObject = new GameObject(Name);
            gameObject.transform.ResetToParent(parent.transform);

            if (joint != null && joint.Origin != null)
                joint.Origin.SetOrigin(gameObject);

            if (Inertial != null)
            {
                Inertial.Create(gameObject);
                if (joint != null)
                    joint.Create(model, gameObject, parent);
            }

            GameObject visualParent = new GameObject("Visuals");
            visualParent.transform.ResetToParent(gameObject.transform);
            foreach (Visual visual in Visuals)
                visual.Create(model, visualParent);

            GameObject collisionParent = new GameObject("Collisions");
            collisionParent.transform.ResetToParent(gameObject.transform);
            foreach (Collision collision in Collisions)
                collision.Create(model, collisionParent);

            foreach (Joint childJoint in ChildJoints)
            {
                Link child = childJoint.ChildLink;
                child.Create(model, gameObject, childJoint);
            }

            return gameObject;

        }

        private static List<Collision> readCollisions(XElement node)
        {
            var collisions =
                from child in node.Elements("collision")
                select new Collision(child);
            return collisions.ToList();

        }
        private static List<Visual> readVisuals(XElement node)
        {
            var visuals =
                from child in node.Elements("visual")
                select new Visual(child);
            return visuals.ToList();
        }
    }

    public class Inertial
    {
        public double Mass;
        public Origin Origin;
        public Inertia Inertia;

        public Inertial(XElement node)
        {
            Origin = (node.Element("origin") != null) ? new Origin(node.Element("origin")) : null; // optional  
            Mass = (double)node.Element("mass").Attribute("value");// required
            Inertia = new Inertia(node.Element("inertia")); // required
        }
        public Rigidbody Create(GameObject gameObject)
        {
            Rigidbody rigidbody = gameObject.AddComponent<Rigidbody>();

            if (Origin != null)
                rigidbody.centerOfMass = Origin.translation();

            // todo: rotation von Inertial bei Inertia anwenden.
            rigidbody.mass = (float)Mass;
            Inertia.setInertia(rigidbody);
            return rigidbody;
        }
    }

    [CustomEditor(typeof(Rigidbody))]
    public class RigidbodyEditor : Editor
    {

        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            Rigidbody _rigidbody = (Rigidbody)target;
            _rigidbody.centerOfMass = EditorGUILayout.Vector3Field("Center Of Mass", _rigidbody.centerOfMass);
            _rigidbody.inertiaTensor = EditorGUILayout.Vector3Field("Inertia Tensor", _rigidbody.inertiaTensor);

            Quaternion inertiaTensorRotation = new Quaternion();
            inertiaTensorRotation.eulerAngles = EditorGUILayout.Vector3Field("Inertia Tensor Rotation", _rigidbody.inertiaTensorRotation.eulerAngles);
            _rigidbody.inertiaTensorRotation = inertiaTensorRotation;
        }
    }

    public class Inertia
    {
        public double Ixx;
        public double Ixy;
        public double Ixz;
        public double Iyy;
        public double Iyz;
        public double Izz;

        private double imin = 1e-6;

        public Inertia(XElement node)
        {
            Ixx = (double)node.Attribute("ixx");
            Ixy = (double)node.Attribute("ixy");
            Ixz = (double)node.Attribute("ixz");
            Iyy = (double)node.Attribute("iyy");
            Iyz = (double)node.Attribute("iyz");
            Izz = (double)node.Attribute("izz");
        }

        public void setInertia(Rigidbody rigidbody)
        {

            // todo: matrix diagonalisieren und rotationsmatrizen in quaternionen verwandeln
            float ixx = (float)((Ixx > imin) ? Ixx : imin);
            float iyy = (float)((Iyy > imin) ? Iyy : imin);
            float izz = (float)((Izz > imin) ? Izz : imin);

            rigidbody.inertiaTensor = new Vector3(ixx, iyy, izz);
        }
    }

    public class Collision
    {
        public string Name;
        public Origin Origin;
        public Geometry Geometry;

        public Collision(XElement node)
        {
            Name = (string)node.Attribute("name"); // optional
            Origin = (node.Element("origin") != null) ? new Origin(node.Element("origin")) : null; // optional  
            Geometry = new Geometry(node.Element("geometry")); // required
        }

        public GameObject Create(Model model, GameObject parent)
        {
            GameObject gameObject = new GameObject((Name == null) ? "unnamed" : Name);
            gameObject.transform.ResetToParent(parent.transform);

            if (Origin != null)
                Origin.SetOrigin(gameObject);

            Geometry.CreateCollider(model, gameObject);

            return gameObject;
        }
    }

    public class Visual
    {
        public string Name;
        public Origin Origin;
        public Geometry Geometry;
        public UrdfMaterial Material;

        public Visual(XElement node)
        {
            Name = (string)node.Attribute("name"); // optional
            Origin = (node.Element("origin") != null) ? new Origin(node.Element("origin")) : null; // optional
            Geometry = new Geometry(node.Element("geometry")); // required
            Material = (node.Element("material") != null) ? new UrdfMaterial(node.Element("material")) : null; // optional
        }

        public GameObject Create(Model model, GameObject parent)
        {
            GameObject gameObject = new GameObject((Name == null) ? "unnamed" : Name);
            gameObject.transform.ResetToParent(parent.transform);

            if (Origin != null)
                Origin.SetOrigin(gameObject);

            Geometry.CreateVisual(model, gameObject);

            if (Material != null)
                Material.SetMaterial(model, gameObject);

            if (gameObject.GetComponentInChildren<Renderer>().sharedMaterial == null)
                UrdfMaterial.SetDefaultMaterial(model, gameObject);

            return gameObject;
        }
    }

    public class UrdfMaterial
    {
        public string Name;
        public UrdfColor Color;
        public UrdfTexture Texture;
        public string Filename;

        public UrdfMaterial()
        {
            Name = null;
            Color = null;
            Texture = null;
            Filename = "Default";
        }

        public UrdfMaterial(XElement node)
        {
            Name = (string)node.Attribute("name"); // required
            Color = (node.Element("color") != null) ? new UrdfColor(node.Element("color")) : null; // optional  
            Texture = (node.Element("texture") != null) ? new UrdfTexture(node.Element("texture")) : null;
            Filename = Path.GetFileNameWithoutExtension(Name);
        }

        public static void SetDefaultMaterial(Model model, GameObject gameObject)
        {
            Material material = GetDefaultMaterialAsset(model.getAssetPath());
            SetMaterial(gameObject, material);
        }

        public void SetMaterial(Model model, GameObject gameObject)
        {
            Material material = GetMaterialAsset(model.getAssetPath());
            SetMaterial(gameObject, material);
        }


        private static void SetMaterial(GameObject gameObject, Material material)
        {
            Renderer[] renderers = gameObject.GetComponentsInChildren<Renderer>();
            foreach (Renderer renderer in renderers)
                renderer.sharedMaterial = material;
        }

        public static Material CreateDefaultMaterialAsset(string assetPath)
        {
            Material material = new Material(Shader.Find("Standard"));
            material.SetFloat("_Metallic", 0.75f);
            material.SetFloat("_Glossiness", 0.75f);
            material.color = new Color(0.33f, 0.33f, 0.33f, 0.0f);
            AssetDatabase.CreateAsset(material, getMaterialAssetPath(assetPath, "Default.mat"));
            return material;
        }

        public Material CreateMaterialAsset(string assetPath)
        {
            Material material = new Material(Shader.Find("Standard"));
            material.SetFloat("_Metallic", 0.75f);
            material.SetFloat("_Glossiness", 0.75f);

            if (Color != null)
            {
                material.color = new Color(
                    (float)Color.Rgba[0],
                    (float)Color.Rgba[1],
                    (float)Color.Rgba[2],
                    (float)Color.Rgba[3]);
            }
            else if (Texture != null)
            {
                string path = Path.Combine(assetPath, Texture.Filename);

                if (path == null)
                    return null;
                Texture tex = AssetDatabase.LoadAssetAtPath<Texture>(path);
                material.mainTexture = tex;
            }

            AssetDatabase.CreateAsset(material, getMaterialAssetPath(assetPath, Filename + ".mat"));
            return material;
        }

        public static Material GetDefaultMaterialAsset(string assetPath)
        {
            return AssetDatabase.LoadAssetAtPath<Material>(getMaterialAssetPath(assetPath, "Default.mat"));
        }

        public Material GetMaterialAsset(string assetPath)
        {
            return AssetDatabase.LoadAssetAtPath<Material>(getMaterialAssetPath(assetPath, Filename + ".mat"));
        }

        public static string getMaterialAssetPath(string assetPath, string filename)
        {
            return Path.Combine(Path.Combine(assetPath, "Materials"), filename + ".mat");
        }
    }

    public class UrdfTexture
    {
        public string Filename;

        public UrdfTexture(XElement node)
        {
            Filename = (string)node.Attribute("filename"); // required
        }
    }

    public class UrdfColor
    {
        public double[] Rgba;

        public UrdfColor(XElement node)
        {
            Rgba = node.Attribute("rgba").ReadDoubleArray(); // required
        }
    }

    public class Geometry
    {
        public Box Box;
        public Cylinder Cylinder;
        public Sphere Sphere;
        public UrdfMesh Mesh;

        public Geometry(XElement node)
        {
            Box = (node.Element("box") != null) ? new Box(node.Element("box")) : null; // optional  
            Cylinder = (node.Element("cylinder") != null) ? new Cylinder(node.Element("cylinder")) : null; // optional  
            Sphere = (node.Element("sphere") != null) ? new Sphere(node.Element("sphere")) : null; // optional  
            Mesh = (node.Element("mesh") != null) ? new UrdfMesh(node.Element("mesh")) : null; // optional           
        }

        public GameObject CreateVisual(Model model, GameObject gameObject)
        {
            if (Box != null)
                return Box.CreateVisual(gameObject);
            else if (Cylinder != null)
                return Cylinder.CreateVisual(gameObject);
            else if (Sphere != null)
                return Sphere.CreateVisual(gameObject);
            else if (Mesh != null)
                return Mesh.CreateVisual(model, gameObject);
            else return null;
        }

        public GameObject CreateCollider(Model model, GameObject gameObject)
        {
            if (Box != null)
                return Box.CreateCollider(gameObject);
            else if (Cylinder != null)
                return Cylinder.CreateCollider(gameObject);
            else if (Sphere != null)
                return Sphere.CreateCollider(gameObject);
            else if (Mesh != null)
                return Mesh.CreateCollider(model, gameObject);
            else return null;
        }
    }

    public class Box
    {
        public double[] Size;

        public Box(XElement node)
        {
            Size = node.Attribute("size") != null ? node.Attribute("size").ReadDoubleArray() : null;
        }

        public GameObject CreateVisual(GameObject parent)
        {
            GameObject gameObject = new GameObject("Box");
            gameObject.transform.ResetToParent(parent.transform);

            MeshFilter meshFilter = gameObject.AddComponent<MeshFilter>();
            meshFilter.mesh = GetMesh();
            gameObject.AddComponent<MeshRenderer>();

            return gameObject;
        }

        public GameObject CreateCollider(GameObject parent)
        {
            GameObject gameObject = new GameObject("Box");
            gameObject.transform.ResetToParent(parent.transform);

            BoxCollider boxCollider = gameObject.AddComponent<BoxCollider>();
            boxCollider.size = new Vector3((float)Size[1], (float)Size[2], (float)Size[0]);

            return gameObject;
        }

        private Mesh GetMesh()
        {
            float length = (float)Size[1];
            float width = (float)Size[2];
            float height = (float)Size[0];

            #region Vertices
            Vector3 p0 = new Vector3(-length * .5f, -width * .5f, height * .5f);
            Vector3 p1 = new Vector3(length * .5f, -width * .5f, height * .5f);
            Vector3 p2 = new Vector3(length * .5f, -width * .5f, -height * .5f);
            Vector3 p3 = new Vector3(-length * .5f, -width * .5f, -height * .5f);

            Vector3 p4 = new Vector3(-length * .5f, width * .5f, height * .5f);
            Vector3 p5 = new Vector3(length * .5f, width * .5f, height * .5f);
            Vector3 p6 = new Vector3(length * .5f, width * .5f, -height * .5f);
            Vector3 p7 = new Vector3(-length * .5f, width * .5f, -height * .5f);

            Vector3[] vertices = new Vector3[]
            {
                p0, p1, p2, p3, // Bottom                
                p7, p4, p0, p3, // Left             
                p4, p5, p1, p0, // Front                
                p6, p7, p3, p2, // Back
                p5, p6, p2, p1, // Right
                p7, p6, p5, p4 // Top
            };
            #endregion

            #region Normales
            Vector3 up = Vector3.up;
            Vector3 down = Vector3.down;
            Vector3 front = Vector3.forward;
            Vector3 back = Vector3.back;
            Vector3 left = Vector3.left;
            Vector3 right = Vector3.right;

            Vector3[] normales = new Vector3[]
            {
                down, down, down, down, // Bottom
                left, left, left, left, // Left
                front, front, front, front, // Front
                back, back, back, back, // Back
                right, right, right, right, // Right
                up, up, up, up // Top
            };
            #endregion

            #region UVs
            Vector2 _00 = new Vector2(0f, 0f);
            Vector2 _10 = new Vector2(1f, 0f);
            Vector2 _01 = new Vector2(0f, 1f);
            Vector2 _11 = new Vector2(1f, 1f);

            Vector2[] uvs = new Vector2[]
            {
                _11, _01, _00, _10, // Bottom 
                _11, _01, _00, _10, // Left
                _11, _01, _00, _10, // Front
                _11, _01, _00, _10, // Back
                _11, _01, _00, _10, // Right
                _11, _01, _00, _10, // Top
            };
            #endregion

            #region Triangles
            int[] triangles = new int[]
            {
                3, 1, 0, // Bottom
                3, 2, 1,
                3 + 4 * 1, 1 + 4 * 1, 0 + 4 * 1, // Left
                3 + 4 * 1, 2 + 4 * 1, 1 + 4 * 1,
                3 + 4 * 2, 1 + 4 * 2, 0 + 4 * 2, // Front
                3 + 4 * 2, 2 + 4 * 2, 1 + 4 * 2,
                3 + 4 * 3, 1 + 4 * 3, 0 + 4 * 3, // Back
                3 + 4 * 3, 2 + 4 * 3, 1 + 4 * 3,
                3 + 4 * 4, 1 + 4 * 4, 0 + 4 * 4, // Right
                3 + 4 * 4, 2 + 4 * 4, 1 + 4 * 4,
                3 + 4 * 5, 1 + 4 * 5, 0 + 4 * 5, // Top
                3 + 4 * 5, 2 + 4 * 5, 1 + 4 * 5,
            };
            #endregion

            Mesh mesh = new Mesh();

            mesh.vertices = vertices;
            mesh.normals = normales;
            mesh.uv = uvs;
            mesh.triangles = triangles;

            mesh.RecalculateBounds();
            return mesh;
        }
    }

    public class Cylinder
    {
        public double Radius;
        public double Length;

        public Cylinder(XElement node)
        {
            Radius = (double)node.Attribute("radius");
            Length = (double)node.Attribute("length");
        }

        public GameObject CreateVisual(GameObject parent)
        {
            GameObject gameObject = new GameObject("Cylinder");
            gameObject.transform.ResetToParent(parent.transform);

            MeshFilter meshFilter = gameObject.AddComponent<MeshFilter>();
            meshFilter.mesh = GetMesh();
            gameObject.AddComponent<MeshRenderer>();

            return gameObject;
        }

        public GameObject CreateCollider(GameObject parent)
        {
            GameObject gameObject = new GameObject("Cylinder");
            gameObject.transform.ResetToParent(parent.transform);

            MeshCollider meshCollider = gameObject.AddComponent<MeshCollider>();
            meshCollider.sharedMesh = GetMesh();
            meshCollider.convex = true;

            return gameObject;
        }

        private Mesh GetMesh()
        {
            float height = (float)Length;
            float bottomRadius = (float)Radius;
            float topRadius = (float)Radius;
            int nbSides = 30;
            int nbHeightSeg = 30;

            int nbVerticesCap = nbSides + 1;
            #region Vertices

            // bottom + top + sides
            Vector3[] vertices = new Vector3[nbVerticesCap + nbVerticesCap + nbSides * nbHeightSeg * 2 + 2];
            int vert = 0;
            float _2pi = Mathf.PI * 2f;

            // Bottom cap
            float top = 0.5f * height;
            float bottom = -0.5f * height;
            vertices[vert++] = new Vector3(0f, bottom, 0f);
            while (vert <= nbSides)
            {
                float rad = (float)vert / nbSides * _2pi;
                vertices[vert] = new Vector3(Mathf.Cos(rad) * bottomRadius, bottom, Mathf.Sin(rad) * bottomRadius);
                vert++;
            }

            // Top cap
            vertices[vert++] = new Vector3(0f, top, 0f);
            while (vert <= nbSides * 2 + 1)
            {
                float rad = (float)(vert - nbSides - 1) / nbSides * _2pi;
                vertices[vert] = new Vector3(Mathf.Cos(rad) * topRadius, top, Mathf.Sin(rad) * topRadius);
                vert++;
            }

            // Sides
            int v = 0;
            while (vert <= vertices.Length - 4)
            {
                float rad = (float)v / nbSides * _2pi;
                vertices[vert] = new Vector3(Mathf.Cos(rad) * topRadius, top, Mathf.Sin(rad) * topRadius);
                vertices[vert + 1] = new Vector3(Mathf.Cos(rad) * bottomRadius, bottom, Mathf.Sin(rad) * bottomRadius);
                vert += 2;
                v++;
            }
            vertices[vert] = vertices[nbSides * 2 + 2];
            vertices[vert + 1] = vertices[nbSides * 2 + 3];
            #endregion

            #region Normales

            // bottom + top + sides
            Vector3[] normales = new Vector3[vertices.Length];
            vert = 0;

            // Bottom cap
            while (vert <= nbSides)
            {
                normales[vert++] = Vector3.down;
            }

            // Top cap
            while (vert <= nbSides * 2 + 1)
            {
                normales[vert++] = Vector3.up;
            }

            // Sides
            v = 0;
            while (vert <= vertices.Length - 4)
            {
                float rad = (float)v / nbSides * _2pi;
                float cos = Mathf.Cos(rad);
                float sin = Mathf.Sin(rad);

                normales[vert] = new Vector3(cos, 0f, sin);
                normales[vert + 1] = normales[vert];

                vert += 2;
                v++;
            }
            normales[vert] = normales[nbSides * 2 + 2];
            normales[vert + 1] = normales[nbSides * 2 + 3];
            #endregion

            #region UVs
            Vector2[] uvs = new Vector2[vertices.Length];

            // Bottom cap
            int u = 0;
            uvs[u++] = new Vector2(0.5f, 0.5f);
            while (u <= nbSides)
            {
                float rad = (float)u / nbSides * _2pi;
                uvs[u] = new Vector2(Mathf.Cos(rad) * .5f + .5f, Mathf.Sin(rad) * .5f + .5f);
                u++;
            }

            // Top cap
            uvs[u++] = new Vector2(0.5f, 0.5f);
            while (u <= nbSides * 2 + 1)
            {
                float rad = (float)u / nbSides * _2pi;
                uvs[u] = new Vector2(Mathf.Cos(rad) * .5f + .5f, Mathf.Sin(rad) * .5f + .5f);
                u++;
            }

            // Sides
            int u_sides = 0;
            while (u <= uvs.Length - 4)
            {
                float t = (float)u_sides / nbSides;
                uvs[u] = new Vector3(t, 1f);
                uvs[u + 1] = new Vector3(t, 0f);
                u += 2;
                u_sides++;
            }
            uvs[u] = new Vector2(1f, 1f);
            uvs[u + 1] = new Vector2(1f, 0f);
            #endregion

            #region Triangles
            int nbTriangles = nbSides + nbSides + nbSides * 2;
            int[] triangles = new int[nbTriangles * 3 + 3];

            // Bottom cap
            int tri = 0;
            int i = 0;
            while (tri < nbSides - 1)
            {
                triangles[i] = 0;
                triangles[i + 1] = tri + 1;
                triangles[i + 2] = tri + 2;
                tri++;
                i += 3;
            }
            triangles[i] = 0;
            triangles[i + 1] = tri + 1;
            triangles[i + 2] = 1;
            tri++;
            i += 3;

            // Top cap
            //tri++;
            while (tri < nbSides * 2)
            {
                triangles[i] = tri + 2;
                triangles[i + 1] = tri + 1;
                triangles[i + 2] = nbVerticesCap;
                tri++;
                i += 3;
            }

            triangles[i] = nbVerticesCap + 1;
            triangles[i + 1] = tri + 1;
            triangles[i + 2] = nbVerticesCap;
            tri++;
            i += 3;
            tri++;

            // Sides
            while (tri <= nbTriangles)
            {
                triangles[i] = tri + 2;
                triangles[i + 1] = tri + 1;
                triangles[i + 2] = tri + 0;
                tri++;
                i += 3;

                triangles[i] = tri + 1;
                triangles[i + 1] = tri + 2;
                triangles[i + 2] = tri + 0;
                tri++;
                i += 3;
            }
            #endregion

            Mesh mesh = new Mesh();

            mesh.vertices = vertices;
            mesh.normals = normales;
            mesh.uv = uvs;
            mesh.triangles = triangles;

            mesh.RecalculateBounds();

            return mesh;
        }
    }

    public class Sphere
    {
        public double Radius;

        public Sphere(XElement node)
        {
            Radius = (double)node.Attribute("radius");
        }

        public GameObject CreateVisual(GameObject parent)
        {
            GameObject gameObject = new GameObject("Sphere");
            gameObject.transform.ResetToParent(parent.transform);

            MeshFilter meshFilter = gameObject.AddComponent<MeshFilter>();
            meshFilter.mesh = GetMesh();
            gameObject.AddComponent<MeshRenderer>();

            return gameObject;
        }

        public GameObject CreateCollider(GameObject parent)
        {
            GameObject gameObject = new GameObject("Sphere");
            gameObject.transform.ResetToParent(parent.transform);

            SphereCollider sphereCollider = gameObject.AddComponent<SphereCollider>();
            sphereCollider.radius = (float)Radius;

            return gameObject;
        }

        private Mesh GetMesh()
        {
            float radius = (float)Radius;
            int nbLong = 30;
            int nbLat = 30;

            #region Vertices
            Vector3[] vertices = new Vector3[(nbLong + 1) * nbLat + 2];
            float _pi = Mathf.PI;
            float _2pi = _pi * 2f;

            vertices[0] = Vector3.up * radius;
            for (int lat = 0; lat < nbLat; lat++)
            {
                float a1 = _pi * (float)(lat + 1) / (nbLat + 1);
                float sin1 = Mathf.Sin(a1);
                float cos1 = Mathf.Cos(a1);

                for (int lon = 0; lon <= nbLong; lon++)
                {
                    float a2 = _2pi * (float)(lon == nbLong ? 0 : lon) / nbLong;
                    float sin2 = Mathf.Sin(a2);
                    float cos2 = Mathf.Cos(a2);

                    vertices[lon + lat * (nbLong + 1) + 1] = new Vector3(sin1 * cos2, cos1, sin1 * sin2) * radius;
                }
            }
            vertices[vertices.Length - 1] = Vector3.up * -radius;
            #endregion

            #region Normales		
            Vector3[] normales = new Vector3[vertices.Length];
            for (int n = 0; n < vertices.Length; n++)
                normales[n] = vertices[n].normalized;
            #endregion

            #region UVs
            Vector2[] uvs = new Vector2[vertices.Length];
            uvs[0] = Vector2.up;
            uvs[uvs.Length - 1] = Vector2.zero;
            for (int lat = 0; lat < nbLat; lat++)
                for (int lon = 0; lon <= nbLong; lon++)
                    uvs[lon + lat * (nbLong + 1) + 1] = new Vector2((float)lon / nbLong, 1f - (float)(lat + 1) / (nbLat + 1));
            #endregion

            #region Triangles
            int nbFaces = vertices.Length;
            int nbTriangles = nbFaces * 2;
            int nbIndexes = nbTriangles * 3;
            int[] triangles = new int[nbIndexes];

            //Top Cap
            int i = 0;
            for (int lon = 0; lon < nbLong; lon++)
            {
                triangles[i++] = lon + 2;
                triangles[i++] = lon + 1;
                triangles[i++] = 0;
            }

            //Middle
            for (int lat = 0; lat < nbLat - 1; lat++)
            {
                for (int lon = 0; lon < nbLong; lon++)
                {
                    int current = lon + lat * (nbLong + 1) + 1;
                    int next = current + nbLong + 1;

                    triangles[i++] = current;
                    triangles[i++] = current + 1;
                    triangles[i++] = next + 1;

                    triangles[i++] = current;
                    triangles[i++] = next + 1;
                    triangles[i++] = next;
                }
            }

            //Bottom Cap
            for (int lon = 0; lon < nbLong; lon++)
            {
                triangles[i++] = vertices.Length - 1;
                triangles[i++] = vertices.Length - (lon + 2) - 1;
                triangles[i++] = vertices.Length - (lon + 1) - 1;
            }
            #endregion

            Mesh mesh = new Mesh();
            mesh.vertices = vertices;
            mesh.normals = normales;
            mesh.uv = uvs;
            mesh.triangles = triangles;

            mesh.RecalculateBounds();

            return mesh;
        }
    }

    public class UrdfMesh
    {
        public string Filename;
        public double[] Scale;

        public UrdfMesh(XElement node)
        {
            Filename = (string)node.Attribute("filename");
            Scale = node.Attribute("scale") != null ? node.Attribute("scale").ReadDoubleArray() : null;

        }

        public GameObject CreateVisual(Model model, GameObject parent)
        {
            GameObject gameObject = UnityEngine.Object.Instantiate(AssetDatabase.LoadAssetAtPath<GameObject>(model.getAssetPath(Filename)));
            gameObject.transform.ResetToParent(parent.transform);
            setScale(gameObject);
            return gameObject;
        }

        public GameObject CreateCollider(Model model, GameObject parent)
        {
            GameObject gameObject = new GameObject(Filename + "(MeshCollider)");
            Transform reference = AssetDatabase.LoadAssetAtPath<Transform>(model.getAssetPath(Filename));
            gameObject.transform.position = reference.position;
            gameObject.transform.rotation = reference.rotation;
            gameObject.transform.localScale = reference.localScale;

            MeshCollider meshCollider = gameObject.AddComponent<MeshCollider>();
            meshCollider.sharedMesh = AssetDatabase.LoadAssetAtPath<Mesh>(model.getAssetPath(Filename));
            meshCollider.convex = true;

            gameObject.transform.ResetToParent(parent.transform);
            setScale(gameObject);

            return gameObject;
        }

        private void setScale(GameObject gameObject)
        {
            if (Scale != null)
            {
                Vector3 scale = new Vector3((float)Scale[0], (float)Scale[1], (float)Scale[2]);
                gameObject.transform.localScale = Vector3.Scale(gameObject.transform.localScale, scale);
            }

        }
    }

    #endregion
}

/*
public ConfigurableJoint CreateHingeJoint(GameObject gameObject, Rigidbody parentRigidbody)
{
    ConfigurableJoint joint = gameObject.AddComponent<ConfigurableJoint>();

    joint.connectedBody = parentRigidbody;

    joint.axis = Axis.GetAxis();

    joint.xMotion = ConfigurableJointMotion.Locked;
    joint.yMotion = ConfigurableJointMotion.Locked;
    joint.zMotion = ConfigurableJointMotion.Locked;
    joint.angularXMotion = ConfigurableJointMotion.Free;
    joint.angularYMotion = ConfigurableJointMotion.Locked;
    joint.angularZMotion = ConfigurableJointMotion.Locked;

    // origin:
    joint.autoConfigureConnectedAnchor = false;
    if (Origin != null)
        joint.connectedAnchor = Origin.translation(); // wo muss Origin.rotation() hin?
    else
        joint.connectedAnchor = Vector3.zero;

    // joint drive:
    if (Dynamics != null)
        joint.angularXDrive = Dynamics.GetJointDrive();

    if (Type == "revolute" && Limit != null)
    {
        joint.lowAngularXLimit = Limit.GetLowSoftJointLimit();
        joint.highAngularXLimit = Limit.GetHighSoftJointLimit();
    }

    // todo:
    // calibration
    // effort: limit
    // velocity: limit
    // safety controller
    // mimic

    return joint;
}*/
