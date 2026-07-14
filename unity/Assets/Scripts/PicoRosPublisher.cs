// SPDX-License-Identifier: BSD-3-Clause
// Windows PCVR input app: publishes locally connected PICO OpenXR tracking to
// the ROS-TCP endpoint on the simulation server using standard ROS 2 messages.

using System;
using UnityEngine;
using UnityEngine.XR;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;

public class PicoRosPublisher : MonoBehaviour
{
    [Header("ROS topics")]
    public string leftPoseTopic = "/pico/left_controller/pose";
    public string rightPoseTopic = "/pico/right_controller/pose";
    public string headPoseTopic = "/pico/head/pose";
    public string joyTopic = "/pico/controllers/joy";
    public float publishHz = 60.0f;

    private ROSConnection ros;
    private float nextPublishTime;

    void Start()
    {
        // Keep controller publication alive while the operator is focused on
        // the separate Isaac Sim WebRTC client window.
        Application.runInBackground = true;
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(leftPoseTopic);
        ros.RegisterPublisher<PoseStampedMsg>(rightPoseTopic);
        ros.RegisterPublisher<PoseStampedMsg>(headPoseTopic);
        ros.RegisterPublisher<JoyMsg>(joyTopic);
    }

    void Update()
    {
        if (Time.unscaledTime < nextPublishTime) return;
        nextPublishTime = Time.unscaledTime + 1.0f / Mathf.Max(publishHz, 1.0f);

        InputDevice left = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
        InputDevice right = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
        InputDevice head = InputDevices.GetDeviceAtXRNode(XRNode.CenterEye);

        PublishPose(left, leftPoseTopic);
        PublishPose(right, rightPoseTopic);
        PublishPose(head, headPoseTopic);
        PublishJoy(left, right);
    }

    private void PublishPose(InputDevice device, string topic)
    {
        if (!device.isValid) return;
        if (!device.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 position)) return;
        if (!device.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rotation)) return;

        Vector3 rosPosition = UnityPositionToRosFlu(position);
        Quaternion rosRotation = UnityRotationToRosFlu(rotation);
        PoseStampedMsg message = new PoseStampedMsg();
        message.header.frame_id = "pico_tracking";
        message.pose.position = new PointMsg(rosPosition.x, rosPosition.y, rosPosition.z);
        message.pose.orientation = new QuaternionMsg(rosRotation.x, rosRotation.y, rosRotation.z, rosRotation.w);
        ros.Publish(topic, message);
    }

    private void PublishJoy(InputDevice left, InputDevice right)
    {
        float leftTrigger = Axis(left, CommonUsages.trigger);
        float leftGrip = Axis(left, CommonUsages.grip);
        float rightTrigger = Axis(right, CommonUsages.trigger);
        float rightGrip = Axis(right, CommonUsages.grip);

        // Stable ABI consumed by the ROS 2 bridge:
        // axes=[left trigger, left grip, right trigger, right grip]
        // buttons=[A, B, X, Y, menu]
        JoyMsg message = new JoyMsg();
        message.axes = new float[] { leftTrigger, leftGrip, rightTrigger, rightGrip };
        message.buttons = new int[] {
            Button(right, CommonUsages.primaryButton),
            Button(right, CommonUsages.secondaryButton),
            Button(left, CommonUsages.primaryButton),
            Button(left, CommonUsages.secondaryButton),
            Math.Max(Button(left, CommonUsages.menuButton), Button(right, CommonUsages.menuButton))
        };
        ros.Publish(joyTopic, message);
    }

    private static float Axis(InputDevice device, InputFeatureUsage<float> usage)
    {
        return device.isValid && device.TryGetFeatureValue(usage, out float value) ? Mathf.Clamp01(value) : 0.0f;
    }

    private static int Button(InputDevice device, InputFeatureUsage<bool> usage)
    {
        return device.isValid && device.TryGetFeatureValue(usage, out bool value) && value ? 1 : 0;
    }

    // Unity is left-handed RUF (right/up/forward); ROS is right-handed FLU
    // (forward/left/up). Position mapping is [z, -x, y].
    private static Vector3 UnityPositionToRosFlu(Vector3 value)
    {
        return new Vector3(value.z, -value.x, value.y);
    }

    // R_ros = C * R_unity * C^-1. C is a reflection because Unity and ROS
    // have opposite handedness, so component swapping alone is not correct.
    private static Quaternion UnityRotationToRosFlu(Quaternion value)
    {
        Matrix4x4 c = Matrix4x4.identity;
        c.SetRow(0, new Vector4(0, 0, 1, 0));
        c.SetRow(1, new Vector4(-1, 0, 0, 0));
        c.SetRow(2, new Vector4(0, 1, 0, 0));
        Matrix4x4 rotation = c * Matrix4x4.Rotate(value) * c.transpose;
        return QuaternionFromMatrix(rotation);
    }

    private static Quaternion QuaternionFromMatrix(Matrix4x4 m)
    {
        float trace = m.m00 + m.m11 + m.m22;
        float x, y, z, w;
        if (trace > 0.0f)
        {
            float s = Mathf.Sqrt(trace + 1.0f) * 2.0f;
            w = 0.25f * s;
            x = (m.m21 - m.m12) / s;
            y = (m.m02 - m.m20) / s;
            z = (m.m10 - m.m01) / s;
        }
        else if (m.m00 > m.m11 && m.m00 > m.m22)
        {
            float s = Mathf.Sqrt(1.0f + m.m00 - m.m11 - m.m22) * 2.0f;
            w = (m.m21 - m.m12) / s;
            x = 0.25f * s;
            y = (m.m01 + m.m10) / s;
            z = (m.m02 + m.m20) / s;
        }
        else if (m.m11 > m.m22)
        {
            float s = Mathf.Sqrt(1.0f + m.m11 - m.m00 - m.m22) * 2.0f;
            w = (m.m02 - m.m20) / s;
            x = (m.m01 + m.m10) / s;
            y = 0.25f * s;
            z = (m.m12 + m.m21) / s;
        }
        else
        {
            float s = Mathf.Sqrt(1.0f + m.m22 - m.m00 - m.m11) * 2.0f;
            w = (m.m10 - m.m01) / s;
            x = (m.m02 + m.m20) / s;
            y = (m.m12 + m.m21) / s;
            z = 0.25f * s;
        }
        Quaternion result = new Quaternion(x, y, z, w);
        result.Normalize();
        return result;
    }
}
