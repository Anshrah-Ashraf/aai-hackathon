# Unity Examples for Robotics Simulation

Unity provides powerful tools and frameworks for creating digital twin environments for humanoid robots. This section explores practical examples of how to implement various robotics simulation features in Unity, demonstrating best practices and common patterns.

## Setting Up Unity for Robotics

### Initial Project Configuration
Configuring Unity for robotics applications:

```csharp
using UnityEngine;
using System.Collections;

public class RobotSimulationSetup : MonoBehaviour
{
    [Header("Simulation Parameters")]
    public float simulationTimeStep = 0.01f;  // 100 Hz update rate
    public bool useFixedTimestep = true;
    public float maxSimulationSteps = 10;

    [Header("Physics Parameters")]
    public float gravity = -9.81f;
    public int solverIterations = 8;
    public float bounceThreshold = 2.0f;

    void Start()
    {
        // Configure physics settings
        Physics.gravity = new Vector3(0, gravity, 0);
        Physics.defaultSolverIterations = solverIterations;
        Physics.bounceThreshold = bounceThreshold;

        // Configure time settings for consistent simulation
        if (useFixedTimestep)
        {
            Time.fixedDeltaTime = simulationTimeStep;
            Time.maximumDeltaTime = simulationTimeStep * maxSimulationSteps;
        }
    }
}
```

### Robotics-Specific Project Settings
Key Unity settings for robotics simulation:
- **Physics settings**: Configure gravity, solver iterations, and collision detection
- **Layer management**: Set up collision layers for different robot parts
- **Tag system**: Use tags for different robot components and environment objects
- **Quality settings**: Balance visual quality with performance requirements
- **Build settings**: Configure for appropriate deployment targets

## Robot Model Import and Setup

### URDF Import Workflow
Using Unity's robotics tools to import robot models:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class URDFRobotSetup : MonoBehaviour
{
    [Header("Joint Configuration")]
    public List<ConfigurableJoint> joints;
    public List<JointDrive> jointDrives;

    [Header("Link Properties")]
    public List<Rigidbody> links;
    public List<ArticulationBody> articulationBodies; // Unity 2020.1+

    void Start()
    {
        ConfigureJoints();
        SetupJointLimits();
        InitializeRobotState();
    }

    void ConfigureJoints()
    {
        for (int i = 0; i < joints.Count; i++)
        {
            var joint = joints[i];
            var drive = jointDrives[i];

            // Configure joint drive for position control
            joint.xDrive = drive;
            joint.yDrive = drive;
            joint.zDrive = drive;

            // Set joint projection to prevent drift
            joint.projectionMode = JointProjectionMode.PositionAndRotation;
            joint.projectionDistance = 0.1f;
            joint.projectionAngle = 5f;
        }
    }

    void SetupJointLimits()
    {
        // Example: Configure revolute joint limits
        foreach (var joint in joints)
        {
            if (joint is ConfigurableJoint configJoint)
            {
                SoftJointLimit lowLimit = configJoint.lowAngularXLimit;
                SoftJointLimit highLimit = configJoint.highAngularXLimit;

                // Configure limits based on robot specifications
                lowLimit.limit = -Mathf.PI / 2;  // -90 degrees
                highLimit.limit = Mathf.PI / 2; // 90 degrees

                configJoint.lowAngularXLimit = lowLimit;
                configJoint.highAngularXLimit = highLimit;
            }
        }
    }

    void InitializeRobotState()
    {
        // Initialize robot to home position
        // This would typically read from robot configuration
    }
}
```

### Articulation Body Setup (Modern Unity)
Using Unity's newer ArticulationBody system:

```csharp
using UnityEngine;

public class ArticulationRobotSetup : MonoBehaviour
{
    [Header("Articulation Chain")]
    public ArticulationBody[] robotChain;

    void Start()
    {
        ConfigureArticulationChain();
    }

    void ConfigureArticulationChain()
    {
        for (int i = 0; i < robotChain.Length; i++)
        {
            var body = robotChain[i];

            // Configure joint type (Fixed, Prismatic, Revolute, etc.)
            body.jointType = ArticulationJointType.RevoluteJoint;

            // Configure joint limits
            var drive = body.xDrive;
            drive.lowerLimit = -90f;  // degrees
            drive.upperLimit = 90f;   // degrees
            drive.forceLimit = 1000f;
            drive.damping = 10f;
            drive.stiffness = 100f;

            body.xDrive = drive;

            // Enable drive for position control
            body.linearDamping = 0.05f;
            body.angularDamping = 0.05f;
        }
    }
}
```

## Sensor Simulation in Unity

### Camera Sensor Simulation
Creating realistic camera sensors:

```csharp
using UnityEngine;

public class CameraSensor : MonoBehaviour
{
    [Header("Camera Parameters")]
    public float fieldOfView = 60f;
    public int resolutionWidth = 640;
    public int resolutionHeight = 480;
    public float nearClip = 0.1f;
    public float farClip = 100f;

    [Header("Noise Parameters")]
    public float noiseIntensity = 0.01f;
    public float gaussianNoise = 0.005f;

    private Camera robotCamera;
    private RenderTexture sensorTexture;

    void Start()
    {
        SetupCamera();
        CreateRenderTexture();
    }

    void SetupCamera()
    {
        robotCamera = GetComponent<Camera>();
        if (robotCamera == null)
        {
            robotCamera = gameObject.AddComponent<Camera>();
        }

        robotCamera.fieldOfView = fieldOfView;
        robotCamera.nearClipPlane = nearClip;
        robotCamera.farClipPlane = farClip;
        robotCamera.clearFlags = CameraClearFlags.SolidColor;
        robotCamera.backgroundColor = Color.black;
    }

    void CreateRenderTexture()
    {
        sensorTexture = new RenderTexture(resolutionWidth, resolutionHeight, 24);
        robotCamera.targetTexture = sensorTexture;
    }

    // Method to get camera data for robotics framework
    public Texture2D GetSensorImage()
    {
        RenderTexture.active = sensorTexture;
        Texture2D image = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RGB24, false);
        image.ReadPixels(new Rect(0, 0, resolutionWidth, resolutionHeight), 0, 0);
        image.Apply();

        // Add noise if required
        AddNoiseToImage(image);

        return image;
    }

    void AddNoiseToImage(Texture2D image)
    {
        Color[] pixels = image.GetPixels();
        for (int i = 0; i < pixels.Length; i++)
        {
            // Add Gaussian noise
            float noise = Random.Range(-gaussianNoise, gaussianNoise);
            pixels[i] += new Color(noise, noise, noise);

            // Clamp values to valid range
            pixels[i] = new Color(
                Mathf.Clamp01(pixels[i].r),
                Mathf.Clamp01(pixels[i].g),
                Mathf.Clamp01(pixels[i].b)
            );
        }
        image.SetPixels(pixels);
        image.Apply();
    }
}
```

### LiDAR Sensor Simulation
Implementing LiDAR simulation using raycasting:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class LiDARSensor : MonoBehaviour
{
    [Header("LiDAR Configuration")]
    public int horizontalResolution = 360;
    public int verticalResolution = 1;
    public float horizontalFOV = 360f;
    public float verticalFOV = 20f;
    public float maxRange = 10f;
    public float minRange = 0.1f;

    [Header("Performance")]
    public float updateRate = 10f; // Hz
    public LayerMask detectionLayers = -1;

    private float updateInterval;
    private float lastUpdate;
    private List<float> pointCloud;

    void Start()
    {
        updateInterval = 1f / updateRate;
        pointCloud = new List<float>();
    }

    void Update()
    {
        if (Time.time - lastUpdate >= updateInterval)
        {
            ScanEnvironment();
            lastUpdate = Time.time;
        }
    }

    void ScanEnvironment()
    {
        pointCloud.Clear();

        float hAngleStep = horizontalFOV / horizontalResolution;
        float vAngleStep = verticalFOV / verticalResolution;

        for (int v = 0; v < verticalResolution; v++)
        {
            float vAngle = (v - verticalResolution / 2) * vAngleStep;

            for (int h = 0; h < horizontalResolution; h++)
            {
                float hAngle = (h - horizontalResolution / 2) * hAngleStep;

                Vector3 scanDirection = CalculateScanDirection(hAngle, vAngle);

                if (Physics.Raycast(transform.position, scanDirection, out RaycastHit hit, maxRange, detectionLayers))
                {
                    if (hit.distance >= minRange)
                    {
                        pointCloud.Add(hit.distance);
                    }
                    else
                    {
                        pointCloud.Add(0f); // Invalid measurement
                    }
                }
                else
                {
                    pointCloud.Add(maxRange + 1f); // Maximum range
                }
            }
        }
    }

    Vector3 CalculateScanDirection(float hAngle, float vAngle)
    {
        // Convert angles to world space direction
        Vector3 direction = transform.forward;
        direction = Quaternion.Euler(0, hAngle, 0) * direction;
        direction = Quaternion.Euler(-vAngle, 0, 0) * direction;
        return direction.normalized;
    }

    // Method to get point cloud data
    public List<float> GetPointCloud()
    {
        return new List<float>(pointCloud);
    }

    // Visualization method for debugging
    void OnDrawGizmosSelected()
    {
        if (pointCloud == null || pointCloud.Count == 0) return;

        Gizmos.color = Color.red;

        float hAngleStep = horizontalFOV / horizontalResolution;
        float vAngleStep = verticalFOV / verticalResolution;

        int index = 0;
        for (int v = 0; v < verticalResolution; v++)
        {
            float vAngle = (v - verticalResolution / 2) * vAngleStep;

            for (int h = 0; h < horizontalResolution; h++)
            {
                float hAngle = (h - horizontalResolution / 2) * hAngleStep;

                Vector3 scanDirection = CalculateScanDirection(hAngle, vAngle);
                float distance = pointCloud[index];

                if (distance <= maxRange && distance >= minRange)
                {
                    Vector3 point = transform.position + scanDirection * distance;
                    Gizmos.DrawSphere(point, 0.02f);
                }

                index++;
            }
        }
    }
}
```

## Control Interface Implementation

### ROS Integration Example
Connecting Unity to ROS for real-time control:

```csharp
using UnityEngine;
using System.Collections;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class ROSRobotController : MonoBehaviour
{
    [Header("ROS Configuration")]
    public string robotName = "humanoid_robot";
    public float controlFrequency = 100f; // Hz

    private ROSConnection ros;
    private float controlInterval;
    private float lastControlUpdate;

    // Robot joint states
    private float[] jointPositions;
    private float[] jointVelocities;
    private float[] jointEfforts;

    void Start()
    {
        // Connect to ROS
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>($"{robotName}/joint_states");

        controlInterval = 1f / controlFrequency;
        InitializeJointArrays();
    }

    void Update()
    {
        if (Time.time - lastControlUpdate >= controlInterval)
        {
            PublishJointStates();
            lastControlUpdate = Time.time;
        }
    }

    void InitializeJointArrays()
    {
        // This would typically be configured based on actual robot joints
        jointPositions = new float[20]; // Example: 20 DOF humanoid
        jointVelocities = new float[20];
        jointEfforts = new float[20];
    }

    void PublishJointStates()
    {
        JointStateMsg jointState = new JointStateMsg();

        // Set timestamps
        jointState.header = new HeaderMsg();
        jointState.header.stamp = new TimeMsg(0, (uint)(Time.time * 1e9));
        jointState.header.frame_id = robotName;

        // Set joint names (would be configured based on robot)
        jointState.name = new string[jointPositions.Length];
        for (int i = 0; i < jointPositions.Length; i++)
        {
            jointState.name[i] = $"joint_{i}";
        }

        // Set joint positions, velocities, and efforts
        jointState.position = jointPositions;
        jointState.velocity = jointVelocities;
        jointState.effort = jointEfforts;

        // Publish to ROS
        ros.Publish($"{robotName}/joint_states", jointState);
    }

    // Method to receive control commands from ROS
    public void SetJointPositions(float[] positions)
    {
        if (positions.Length == jointPositions.Length)
        {
            for (int i = 0; i < positions.Length; i++)
            {
                jointPositions[i] = positions[i];
            }
        }
    }
}
```

## Human-Robot Interaction Examples

### Gesture Recognition System
Implementing gesture recognition for HRI:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class GestureRecognition : MonoBehaviour
{
    [Header("Gesture Detection")]
    public float gestureThreshold = 0.1f;
    public float gestureTimeout = 2.0f;

    private Vector3 lastHandPosition;
    private List<Vector3> gesturePath;
    private float gestureStartTime;

    void Start()
    {
        gesturePath = new List<Vector3>();
    }

    void Update()
    {
        // Detect hand movement (would typically come from hand tracking system)
        Vector3 currentHandPosition = GetHandPosition(); // Implementation dependent

        if (Vector3.Distance(currentHandPosition, lastHandPosition) > gestureThreshold)
        {
            gesturePath.Add(currentHandPosition);
            lastHandPosition = currentHandPosition;

            if (gesturePath.Count == 1)
            {
                gestureStartTime = Time.time;
            }
        }

        // Check for gesture completion
        if (gesturePath.Count > 5 && Time.time - gestureStartTime > gestureTimeout)
        {
            RecognizeGesture();
        }
    }

    Vector3 GetHandPosition()
    {
        // This would interface with hand tracking system
        // For example, from VR controllers or computer vision
        return Vector3.zero;
    }

    void RecognizeGesture()
    {
        // Analyze gesture path and recognize pattern
        string gesture = AnalyzeGesturePath(gesturePath);

        if (!string.IsNullOrEmpty(gesture))
        {
            ExecuteGestureCommand(gesture);
        }

        gesturePath.Clear();
    }

    string AnalyzeGesturePath(List<Vector3> path)
    {
        // Implement gesture recognition algorithm
        // This is a simplified example
        if (path.Count < 10) return null;

        Vector3 start = path[0];
        Vector3 end = path[path.Count - 1];
        Vector3 direction = (end - start).normalized;

        // Example: Recognize horizontal swipe
        if (Mathf.Abs(direction.y) < 0.3f && Mathf.Abs(direction.x) > 0.7f)
        {
            return direction.x > 0 ? "swipe_right" : "swipe_left";
        }

        return null;
    }

    void ExecuteGestureCommand(string gesture)
    {
        switch (gesture)
        {
            case "swipe_right":
                Debug.Log("Gesture: Swipe Right - Robot turning right");
                break;
            case "swipe_left":
                Debug.Log("Gesture: Swipe Left - Robot turning left");
                break;
            default:
                Debug.Log($"Unrecognized gesture: {gesture}");
                break;
        }
    }
}
```

## Performance Optimization Examples

### Level of Detail (LOD) System
Implementing performance optimization:

```csharp
using UnityEngine;

public class RobotLODSystem : MonoBehaviour
{
    [Header("LOD Configuration")]
    public Transform[] lodLevels;
    public float[] lodDistances;
    public Renderer[] robotRenderers;

    [Header("Performance Settings")]
    public float lodUpdateInterval = 0.1f;
    public Transform cameraTransform;

    private float lastUpdate;
    private int currentLODLevel = 0;

    void Start()
    {
        if (cameraTransform == null)
        {
            cameraTransform = Camera.main.transform;
        }
    }

    void Update()
    {
        if (Time.time - lastUpdate >= lodUpdateInterval)
        {
            UpdateLOD();
            lastUpdate = Time.time;
        }
    }

    void UpdateLOD()
    {
        if (cameraTransform == null) return;

        float distance = Vector3.Distance(transform.position, cameraTransform.position);

        int newLODLevel = 0;
        for (int i = 0; i < lodDistances.Length; i++)
        {
            if (distance > lodDistances[i])
            {
                newLODLevel = i + 1;
            }
            else
            {
                break;
            }
        }

        // Clamp to valid range
        newLODLevel = Mathf.Clamp(newLODLevel, 0, lodLevels.Length - 1);

        if (newLODLevel != currentLODLevel)
        {
            SwitchLODLevel(newLODLevel);
            currentLODLevel = newLODLevel;
        }
    }

    void SwitchLODLevel(int level)
    {
        // Activate the appropriate LOD level
        for (int i = 0; i < lodLevels.Length; i++)
        {
            lodLevels[i].gameObject.SetActive(i == level);
        }
    }
}
```

## Scene Management and Environment Setup

### Dynamic Environment Loading
Managing complex environments efficiently:

```csharp
using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class EnvironmentManager : MonoBehaviour
{
    [Header("Environment Zones")]
    public List<EnvironmentZone> zones;
    public float activationDistance = 50f;
    public float deactivationDistance = 70f;

    [Header("Performance")]
    public Transform playerReference;

    private Dictionary<EnvironmentZone, bool> zoneStates;

    [System.Serializable]
    public class EnvironmentZone
    {
        public string zoneName;
        public GameObject zoneObject;
        public Vector3 centerPosition;
        public float radius;
    }

    void Start()
    {
        if (playerReference == null)
        {
            playerReference = Camera.main.transform;
        }

        zoneStates = new Dictionary<EnvironmentZone, bool>();
        foreach (var zone in zones)
        {
            zoneStates[zone] = false;
            zone.zoneObject.SetActive(false); // Start inactive
        }
    }

    void Update()
    {
        if (playerReference == null) return;

        foreach (var zone in zones)
        {
            float distance = Vector3.Distance(playerReference.position, zone.centerPosition);
            bool shouldBeActive = distance <= activationDistance + zone.radius;

            if (shouldBeActive != zoneStates[zone])
            {
                zone.zoneObject.SetActive(shouldBeActive);
                zoneStates[zone] = shouldBeActive;
            }
        }
    }
}
```

These examples demonstrate practical implementations of various robotics simulation features in Unity, showing how to balance visual quality with physical accuracy while maintaining good performance. Each example can be adapted and extended based on specific requirements for digital twin applications.