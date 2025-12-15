ROBOT = "go2" # Robot name, "go2", "b2", "b2w", "h1", "go2w", "g1" 
ROBOT_SCENE = "../unitree_robots/" + ROBOT + "/scene.xml" # Robot scene
DOMAIN_ID = 1 # Domain id
INTERFACE = "lo" # Interface 

USE_JOYSTICK = 1 # Simulate Unitree WirelessController using a gamepad
JOYSTICK_TYPE = "xbox" # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0 # Joystick number

PRINT_SCENE_INFORMATION = True # Print link, joint and sensors information of robot
ENABLE_ELASTIC_BAND = False # Virtual spring band, used for lifting h1

SIMULATE_DT = 0.002  # Need to be larger than the runtime of viewer.sync()
VIEWER_DT = 0.02  # 50 fps for viewer

# Depth camera settings
ENABLE_DEPTH_CAMERA = True
CAMERA_SENSOR_NAME = "depth_camera"
CAMERA_ORIGINAL_WIDTH = 640
CAMERA_ORIGINAL_HEIGHT = 480
CAMERA_DOWNSAMPLED_WIDTH = 80
CAMERA_DOWNSAMPLED_HEIGHT = 60
DEPTH_PUBLISH_DT = 0.1  # Publish depth image at 10 Hz
NEAR_CLIP = 0.175  # Near clip plane for depth camera
FAR_CLIP = 3.0     # Far clip plane for depth camera

