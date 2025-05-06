Проект для управления движением робота AR600.

Humanoid robot control experiments based on Kajita et al. experience.

This repository contains walking control algorithms based on LIPM (Linear Inverted Pendulum Model).

Опирается на данные, полученные в https://github.com/tarasradio/humanoid_control

Для запуска движения:

 1 - запустить ROS:

 ~/ roscore

 2 - запустить Gazebo:

 ~/ rosrun gazebo_ros gazebo

 3 - поставить модель робота в Gazebo:

 ~/ roslaunch ar600_control start_simulation.launch

 4 - запустить файл для движения:

 ~/ python walker.py (для запуска нужно перейти в каталог: catkin_ws/src/humanoid_vstu/ar600_control/scripts/)

Код работает под ROS1.noetic

Для работы с ROS в VS.Code требуется установить следующие расширения:

- ROS (v 0.9.2 на момент написания документа)
- Python

Для работы VS Code с ROS:
-  добавить в файл c_cpp_properties.json (.vscode):

{
  "configurations": [
    {
      "browse": {
        "databaseFilename": "",
        "limitSymbolsToIncludedHeaders": true
      },
      "includePath": [
        "/home/{username}/catkin_ws/devel/include/**",
        "/opt/ros/noetic/include/**",
        "/home/{username}/catkin_ws/src/humanoid_vstu/ar600_control/include/**",
        "/usr/include/**"
      ],
      "name": "ROS"
    }
  ],
  "version": 4
}

- добавить в settings.json (.vscode):

{
  "python.autoComplete.extraPaths": [
    "/home/{username}/catkin_ws/devel/lib/python3/dist-packages",
    "/opt/ros/noetic/lib/python3/dist-packages"
  ],
  "python.analysis.extraPaths": [
    "/home/{username}/catkin_ws/devel/lib/python3/dist-packages",
    "/opt/ros/noetic/lib/python3/dist-packages"
  ]
}

Структура каталогов:

~/catkin_ws:
 - src/
  - humanoid_vstu/
    - ar600_control/ (этот репозиторий)
      - config/
        - joints.yaml
      - include/
        - ar600_control/
      - launch/
        - gazebo.launch
        - init_ros_control.launch
        - init_ros_control_legs.launch
        - start_simulation.launch
      - scripts/
        - footsteps_generator.py
        - foot_trajectories.py
        - leg_ik.py
        - leg_joints.py
        - legs_controller.py
        - lipm.py
        - plotter.py
        - position_transition.py
        - robot_description.py
        - robot_gazebo.py
        - walker.py
      - src/
      - CMakeLists.txt
      - package.xml
      - setup.py
    - ar600_description/ (репозиторий https://github.com/tarasradio/ar600_description)
      - config/
        - config_view.rviz
      - launch/
        - gazebo.launch
        - rviz.launch
        - spawn.launch
      - meshes/
        - collision/
          - foot_left.stl
          - foot_right.stl
          - head.stl
          - hip_fork-left.stl
          - hip-fork-right.stl
          - hip-left.stl
          - hip_right.stl
          - pelvis.stl
          - shin_left.stl
          - shin_right.stl
          - shoulder_fork_left.stl
          - shoulder_fork_right.stl
          - shoulder_left.stl
          - shoulder_right.stl
          - torso.stl
        - visual/
          - foot_left.stl
          - foot_right.stl
          - head.stl
          - hip_cross_left.stl
          - hip_cross_right.stl
          - hip_fork-left.stl
          - hip-fork-right.stl
          - hip-left.stl
          - hip_right.stl
          - neck.stl
          - neck_cross.stl
          - neck_fork.stl
          - pelvis.stl
          - shin_left.stl
          - shin_right.stl
          - shoulder_fork_left.stl
          - shoulder_fork_right.stl
          - shoulder_left.stl
          - shoulder_right.stl
          - torso.stl
      - xacro/
        - ar600.xacro
        - arm.xacro
        - head_and_neck.xacro
        - left_right.xacro
        - leg.xacro
        - lr_limit.xacro
        - materials.xacro
        - meshes.xacro
        - pelvis.xacro
        - stairs.xacro
        - torso.xacro
        - transmission_block.xacro
      - axis.png
      - CMakeLists.txt
      - kinamatic.png
      - package.xml
      - README.md
    - joint_states_listener/ (репозиторий https://github.com/tarasradio/joint_states_listener)
      - nodes/
        - listener.py
      - scripts/
        - joints_printer.py
      - src/
      - srv/
        - ReturnJointStates.srv
      - CMakeLists.txt
      - package.xml
      - setup.py