启动imu节点
cd ~/lightning_ws
source ./install/setup.bash
ros2 launch wit_imu_ros2 wit_imu.launch.py

启动雷达节点
cd ~/lightning_ws
source ./install/setup.bash
sudo ifconfig eno1 192.168.1.102
密码：1234
ros2 launch lslidar_driver lslidar_c16_launch.py

启动建图导航相机
source /home/agx/rtabmap_ws/install/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  serial_no:=_338122302709 \
  camera_name:=nav camera_namespace:=camera \
  align_depth.enable:=true

启动hydra
source /home/agx/hydra_ws/install/setup.bash
ros2 launch hydra_ros wheel_hydra.launch.py

启动hydra mcp服务
bash /home/agx/hydra_mcp_servers/start_http_daemon.sh

验证
curl http://127.0.0.1:9000/health 
停止
pkill -9 -f 'hydra_object_summary_server.py --no-stdio'

锁频率
sudo jetson_clocks

启动建图
cd ~/lightning_ws
source ./install/setup.bash
ros2 run lightning run_slam_online --config /home/agx/lightning_ws/src/hoslam_lightning_lm/config/default_c16.yaml

启动重定位
雷达点云范围改为180度
cd ~/lightning_ws
source ./install/setup.bash
ros2 run lightning run_loc_online --config /home/agx/lightning_ws/src/hoslam_lightning_lm/config/default_c16.yaml

/home/agx/ros2_ws/src/david_nav/navigate_semantic_pose_with_debug_bag.sh

抓取任务一键启动导航
~/ros2_ws/src/david_nav/start_waypoint_nav_headless.sh

一键启动语音导航
/home/agx/ros2_ws/src/david_nav/start_navigation_headless.sh 

保存地图
source install/setup.bash
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: my_map}"

启动标点
cd /home/agx/mcp_server
source /opt/ros/humble/setup.bash
source .venv/bin/activate
python server.py --host 0.0.0.0 --port 8000 --odom-topic /lightning/odometry
另一终端
cd /home/agx/mcp_server
source /opt/ros/humble/setup.bash
source .venv/bin/activate
curl http://127.0.0.1:8000/health
python client_example.py  tools
python client_example.py pose
python client_example.py save pose


python client_example.py get  pose
python client_example.py --server http://127.0.0.1:8000 save apple
python client_example.py --server http://127.0.0.1:8000 get apple


启动底盘驱动
cd ~/wheelhumanoidrobot/workspace_wheeltec/wheeltec_robot_ros2/
source ./install/setup.bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

键盘控制
cd ~/wheelhumanoidrobot/workspace_wheeltec/wheeltec_robot_ros2/
source ./install/setup.bash
ros2 run turn_on_wheeltec_robot keyboard_teleop_ros2.py 


启动导航
cd ~/ros2_ws
source ./install/setup.bash
ros2 launch david_nav 2d_david_nav2_bringup.launch.py use_rviz:=false enable_scan_bridge:=false

~/ros2_ws/src/david_nav/scripts/launch_waypoint_nav.sh

启动hydra
包的语义
bash ~/run_hydra_uhumans2.sh
模型的语义
bash ~/run_hydra_uhumans2.sh model
换其它模型
MODEL_NAME=ade20k-efficientvit_seg_l2_fp16  bash ~/run_hydra_uhumans2.sh model

lerobot录制数据集指令：

lerobot录制数据集指令：

conda activate lerobot

export HF_USER=ronzhou
python -m lerobot.record \
  --robot.type=bi_so100_follower \
  --robot.left_arm_port=/dev/ttyACM1 \
  --robot.right_arm_port=/dev/ttyACM0 \
  --robot.id=bimanual_follower \
  --robot.cameras='{
   main: {"type": "opencv", "index_or_path": 4, "width": 1280, "height": 720, "fps": 30},
  }' \
  --teleop.type=bi_so100_leader \
  --teleop.left_arm_port=/dev/ttyACM3 \
  --teleop.right_arm_port=/dev/ttyACM2 \
  --teleop.id=bimanual_leader \
  --display_data=true \
  --dataset.repo_id=${HF_USER}/bimanual-so100-stack-red-cube \
  --dataset.num_episodes=100 \
  --dataset.single_task="Grab and stack the red cube " \
  --dataset.push_to_hub=false \
  --resume=true

可以通过重新运行相同命令并加上 --resume=true 来继续。在继续录制时， --dataset.num_episodes 必须设置为要额外录制的集数，而不是数据集中目标的总集数！
To start recording from scratch, manually delete the dataset directory.
要从头开始录制，请手动删除数据集目录。


realsense（RGBD）录制数据集指令（抖动严重，像素高时相机超时退出）

export HF_USER=ronzhou
python -m lerobot.record \
  --robot.type=bi_so100_follower \
  --robot.left_arm_port=/dev/ttyACM1 \
  --robot.right_arm_port=/dev/ttyACM0 \
  --robot.id=bimanual_follower \
  --robot.cameras='{
   main: {"type": "intelrealsense", "serial_number_or_name": "338122302709", "width": 640, "height": 480, "fps": 30},
  }' \
  --teleop.type=bi_so100_leader \
  --teleop.left_arm_port=/dev/ttyACM3 \
  --teleop.right_arm_port=/dev/ttyACM2 \
  --teleop.id=bimanual_leader \
  --display_data=true \
  --dataset.repo_id=${HF_USER}/bimanual-so100-handover-cube \
  --dataset.num_episodes=50 \
  --dataset.single_task="Grab and handover the banana to the other arm" \
  --dataset.push_to_hub=false


可视化本地数据集

python -m lerobot.scripts.visualize_dataset \
  --repo-id dummy/local-dataset \
  --root ./data/bi-so100-banana/bimanual-so100-handover-cube-realsense-depth-25 \
  --episode-index 0

电机校准 
python -m lerobot.setup_motors \
    --teleop.type=so100_leader \
    --teleop.port=/dev/ttyACM2
    
python -m lerobot.setup_motors \
    --robot.type=so100_follower \
    --robot.port=/dev/ttyACM0
    
遥操作：
python -m lerobot.teleoperate \
  --robot.type=bi_so100_follower \
  --robot.left_arm_port=/dev/ttyACM1 \
  --robot.right_arm_port=/dev/ttyACM0 \
  --robot.id=bimanual_follower \
  --robot.cameras='{
    left: {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30},
    top: {"type": "opencv", "index_or_path": 6, "width": 640, "height": 480, "fps": 30},
    right: {"type": "opencv", "index_or_path": 8, "width": 640, "height": 480, "fps": 30}
  }' \
  --teleop.type=bi_so100_leader \
  --teleop.left_arm_port=/dev/ttyACM3 \
  --teleop.right_arm_port=/dev/ttyACM2 \
  --teleop.id=bimanual_leader \
  --display_data=true
  
  
 liuzhenlubao
 终端1
 ros2 launch semantic_inference_ros closed_set.launch.py     model_name:=ade20k-efficientvit_seg_l2_fp16     image_topic:=/tesse/left_cam/rgb/image_raw     use_prerecorded:=false     compressed_rgb:=false     labelspace_name:=ade20k_full
 终端2
 ros2 bag play /home/agx/hydra_ws/datasets/uhumans2/apartment_scene/uHumans2_apartment_s1_00h_ros2 --clock
终端3
rviz2
终端4
 ros2 bag record -o hydra_bag /tesse/depth_cam/mono/image_raw /tesse/left_cam/rgb/image_raw /semantic_color/image_raw /tf

