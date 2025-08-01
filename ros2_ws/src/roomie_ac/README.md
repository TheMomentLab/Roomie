
폴더 구조
'''
roomie_ac/
├── package.xml
├── CMakeLists.txt
├── [setup.py](http://setup.py/)
├── resource/
│   └── roomie_ac
├── roomie_ac/
│   ├── **init**.py
│   ├── ac_node.py       # ArmControlNode (메인 ROS 2 노드)
│   ├── ac_action_server.py      # ArmActionServer
│   ├── vision_service_client.py  # VisionServiceClient
│   ├── kinematics_solver.py      # KinematicsSolver (basic2.py의 KinematicsSolver 클래스)
│   ├── esp32_serial_commander.py # ESP32SerialCommander (basic2.py의 SerialManager 클래스)
│   └── ros_joint_publisher.py    # ROSJointPublisher 클래스
│   
├── urdf/
│   └── roomie2.urdf              # 로봇 URDF 파일
└── launch/
└── arm_control_launch.py
'''
