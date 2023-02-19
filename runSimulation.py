# 已知bug：在启动仿真环境后，需要手动修改Viewport:
# Viewport1 -> Overall_camera
# Viewport2 -> Camera_left
# Viewport3 -> Camera_right

# 小车的控制、位姿真值的发布由GUI Actin Graph完成(真值Topic名称为/odom)
# 代码主要实现对于双目数据的帧率设置与发布、IMU的帧率设置与发布

# 一些参数
scene_path = "/home/xuhui/Omniverse-Projects/our_carV6.usd" # 场景+小车所在的usd文件路径
imu_outpath = "/home/xuhui/omni-record/imu_data.txt"        # IMU数据输出文件路径

prim_imu_path = "/our_car/camera_body/Imu_Sensor"   # IMU的Prim路径
prim_cam_left_path = "/our_car/body/Camera_left"    # 左相机的Prim路径
prim_cam_right_path = "/our_car/body/Camera_right"  # 右相机的Prim路径

frequency_imu = 500                     # IMU的观测频率
rgb_interval = 10                       # 每隔N个渲染帧发布一帧
topic_name_left_image = "/rgb_left"     # 发布的左影像Topic名称
topic_name_right_image = "/rgb_right"   # 发布的右影像Topic名称

# ------------------------------------------------------------

# standalone模式规定代码
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

# 启用ROS Bridge扩展，否则Isaac Sim无法发布ROS Topic
from omni.isaac.core.utils import extensions
extensions.enable_extension("omni.isaac.ros_bridge")

# 加载环境与机器人(要在新建世界之前，不然会报错)
from omni.isaac.core.utils.stage import open_stage
open_stage(usd_path=scene_path)

# 新建世界
from omni.isaac.core import World
world = World()

# 根据参数设置IMU频率
from omni.isaac.core.utils.prims import get_prim_at_path
prim_imu = get_prim_at_path(prim_imu_path)
prim_imu.GetAttribute("sensorPeriod").Set(1/frequency_imu)

# 构造Action Graph，发布相机数据
import omni.graph.core as og
keys = og.Controller.Keys
(ros_camera_graph, _, _, _) = og.Controller.edit(
    {
        "graph_path": "/publish_camera",    # 注意Graph的名称必须以/开头
        "evaluator_name": "push",
        "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
    },
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),

            ("createViewportLeft", "omni.isaac.core_nodes.IsaacCreateViewport"),
            ("createViewportRight", "omni.isaac.core_nodes.IsaacCreateViewport"),

            ("setActiveCameraLeft", "omni.graph.ui.SetActiveViewportCamera"),
            ("setActiveCameraRight", "omni.graph.ui.SetActiveViewportCamera"),

            ("cameraHelperLeft", "omni.isaac.ros_bridge.ROS1CameraHelper"),
            ("cameraHelperRight", "omni.isaac.ros_bridge.ROS1CameraHelper"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "createViewportLeft.inputs:execIn"),
            ("createViewportLeft.outputs:viewport", "setActiveCameraLeft.inputs:execIn"),
            ("createViewportLeft.outputs:viewport", "cameraHelperLeft.inputs:viewport"),
            ("setActiveCameraLeft.outputs:execOut", "cameraHelperLeft.inputs:execIn"),

            ("OnPlaybackTick.outputs:tick", "createViewportRight.inputs:execIn"),
            ("createViewportRight.outputs:viewport", "setActiveCameraRight.inputs:viewport"),
            ("createViewportRight.outputs:viewport", "cameraHelperRight.inputs:viewport"),
            ("setActiveCameraRight.outputs:execOut", "cameraHelperRight.inputs:execIn"),
        ],
        keys.SET_VALUES: [
            ("createViewportLeft.inputs:viewportId", 1),
            ("createViewportRight.inputs:viewportId", 2),
            ("setActiveCameraLeft.inputs:primPath", prim_cam_left_path),
            ("setActiveCameraRight.inputs:primPath", prim_cam_right_path),

            ("cameraHelperLeft.inputs:topicName", topic_name_left_image),
            ("cameraHelperLeft.inputs:type", "rgb"),

            ("cameraHelperRight.inputs:topicName", topic_name_right_image),
            ("cameraHelperRight.inputs:type", "rgb"),
        ],
    },
)

# 运行一次构造的Graph，生成SDGPipeline
og.Controller.evaluate_sync(ros_camera_graph)
simulation_app.update()

# 修改SDGPipeline对应的节点，调整帧率
left_camera_path = "/Render/PostProcess/SDGPipeline/RenderProduct_Viewport_2_LdrColorSDIsaacSimulationGate"
right_camera_path = "/Render/PostProcess/SDGPipeline/RenderProduct_Viewport_3_LdrColorSDIsaacSimulationGate"
import omni.graph.core as og
og.Controller.attribute(left_camera_path + ".inputs:step").set(rgb_interval)
og.Controller.attribute(right_camera_path + ".inputs:step").set(rgb_interval)

# 获取IMU传感器对象
from omni.isaac.isaac_sensor import _isaac_sensor
_is = _isaac_sensor.acquire_imu_sensor_interface()

print("start to simulate")
frame_counter = 0
imu_out = open(imu_outpath,"w")
world.reset()

# 开始循环，在循环过程中采集IMU数据并输出至指定文件
while True:
    # 读取IMU传感器数据
    imu_reading = _is.get_sensor_readings(prim_imu_path)

    # 解析IMU传感器数据
    if(len(imu_reading) != 0):
        for i in range(len(imu_reading)):
            tmp_imu_reading = imu_reading[i]
            data_timestamp = tmp_imu_reading[0]
            
            data_acc_x = tmp_imu_reading[1]
            data_acc_y = tmp_imu_reading[2]
            data_acc_z = tmp_imu_reading[3]

            data_ang_x = tmp_imu_reading[4]
            data_ang_y = tmp_imu_reading[5]
            data_ang_z = tmp_imu_reading[6]

            data_ori_x = tmp_imu_reading[7][0]
            data_ori_y = tmp_imu_reading[7][1]
            data_ori_z = tmp_imu_reading[7][2]
            data_ori_w = tmp_imu_reading[7][3]

            imu_out.write(str(data_timestamp)+"\t"+
                str(data_acc_x)+"\t"+str(data_acc_y)+"\t"+str(data_acc_z)+"\t"+
                str(data_ang_x)+"\t"+str(data_ang_y)+"\t"+str(data_ang_z)+"\t"+
                str(data_ori_x)+"\t"+str(data_ori_y)+"\t"+str(data_ori_z)+"\t"+str(data_ori_w)+"\n")
            # print(data_timestamp, data_acc_x, data_acc_y, data_acc_z, data_ang_x, data_ang_y, data_ang_z, data_ori_x, data_ori_y, data_ori_z, data_ori_w)

    world.step(render=True)
    
    if frame_counter % 5000 == 0:
        print("IMU Recoding in progress")
    frame_counter += 1
