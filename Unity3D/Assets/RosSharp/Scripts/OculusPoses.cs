using System;
using UnityEngine;
using UnityEngine.VR;

public class OculusPoses : MonoBehaviour//ScriptableObject
{
    //> in OculusPoses <
    //PoseLeft: position {x,y,z}, orientation {x,y,z,w}
    //PoseRight: position {x,y,z}, orientation {x,y,z,w}
    //PoseHead: position {x,y,z}, orientation {x,y,z,w}
    //PoseEyeLeft: position {x,y,z}, orientation {x,y,z,w}
    //PoseEyeRight: position {x,y,z}, orientation {x,y,z,w}

    public static PoseVR poseVR { get; set; }


    public static void Update()
    {
        OVRInput.Update();
        PoseVR newPose = new PoseVR();

        newPose.Head.Position = InputTracking.GetLocalPosition(VRNode.Head);
        newPose.LeftEye.Position = InputTracking.GetLocalPosition(VRNode.LeftEye);
        newPose.RightEye.Position = InputTracking.GetLocalPosition(VRNode.RightEye);
        newPose.CenterEye.Position = InputTracking.GetLocalPosition(VRNode.CenterEye);
        newPose.LeftHand.Position = InputTracking.GetLocalPosition(VRNode.LeftHand);
        newPose.RightHand.Position = InputTracking.GetLocalPosition(VRNode.RightHand);

        newPose.Head.Orientation = InputTracking.GetLocalRotation(VRNode.Head);
        newPose.LeftEye.Orientation = InputTracking.GetLocalRotation(VRNode.LeftEye);
        newPose.RightEye.Orientation = InputTracking.GetLocalRotation(VRNode.RightEye);
        newPose.CenterEye.Orientation = InputTracking.GetLocalRotation(VRNode.CenterEye);
        newPose.LeftHand.Orientation = InputTracking.GetLocalRotation(VRNode.LeftHand);
        newPose.RightHand.Orientation = InputTracking.GetLocalRotation(VRNode.RightHand);

        OculusButtons.Update();
        newPose.Buttons = OculusButtons.touchController;

        poseVR = newPose;
    }

    public static string toJSON()
    {
        return JsonUtility.ToJson(poseVR);
    }

    [Serializable]
    public struct Pose
    {
        public Vector3 Position;
        public Quaternion Orientation;
    }

    [Serializable]
    public struct PoseVR
    {
        public Pose LeftHand;
        public Pose RightHand;
        public Pose LeftEye;
        public Pose RightEye;
        public Pose CenterEye;
        public Pose Head;
        public OculusButtons.TouchController Buttons;
    }
}
