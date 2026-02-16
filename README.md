Comandi per terminali

1 | 2
-----
3 | 5
4

6

TERMINALE 1:
ros2 launch franka_fr3_moveit_config MIO_moveit.launch.py robot_ip:=192.168.9.12
ros2 launch franka_bringup franka.launch.py robot_ip:=192.168.9.12

Se vuoi lanciare con fake-hardware:
ros2 launch franka_fr3_moveit_config MIO_moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true

Se vuoi lanciare ad elevata priorita:
sudo bash -c "source /opt/ros/humble/setup.bash && source /home/hargalaten/franka_ros2_ws/install/setup.bash && chrt -f 85 ros2 launch franka_fr3_moveit_config MIO_moveit.launch.py robot_ip:=192.168.9.12"

-----------------------------------------------------

TERMINALE 2:
ros2 launch realsense2_camera rs_launch.py clip_distance:=1.0 config_file:=/home/hargalaten/Desktop/MIO_realsense_pickplace.yaml

Per calibrazione:
ros2 launch realsense2_camera rs_launch.py clip_distance:=1.0 config_file:=/home/hargalaten/Desktop/MIO_realsense_calib.yaml

-----------------------------------------------------

TERMINALE 3:
ros2 launch real_franka_robot MIO_static_transform_publisher.launch.py

-----------------------------------------------------

TERMINALE 4:
PER PASSARE A GRAVITY_COMPENSATION: 
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{start_controllers: ['gravity_compensation_example_controller'], stop_controllers: ['fr3_arm_controller'], strictness: 2, start_asap: 2}"

PER RITORNARE A FR3_CONTROLLER: 
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{start_controllers: ['fr3_arm_controller'], stop_controllers: ['gravity_compensation_example_controller'], strictness: 2, start_asap: true}"

PER RESETTARE UN ERRORE REFLEX:
ros2 action send_goal /action_server/error_recovery franka_msgs/action/ErrorRecovery "{}"

PER RESETTARE LA OCTOMAP:
ros2 service call /clear_octomap std_srvs/srv/Empty {}

-----------------------------------------------------------

Terminale 5:
export GEMINI_API_KEY=...
ros2 run real_franka_robot inner_monologue

-----------------------------------------------------------

Terminale 6:
ros2 launch perception_server get_scene_objects.launch.py

-----------------------------------------------------

Verificare la qualità della comunicazione:
ros2 topic echo /franka_robot_state_broadcaster/franka_states --field control_command_success_rate

Per leggere il QoS dei topic:
ros2 topic info /camera/camera/depth/color/points --verbose

ChArUco:
Squares,X,Y: 5x5
Marker size: 50 px
Square size: 80 px
margin size: 10 px
marker border: 1 bits 
DICT 4x4
0.133 m (board length)
0.0167 m (measured marker size)

Chiudi gripper:
ros2 action send_goal -f /franka_gripper/grasp franka_msgs/action/Grasp "{width: 0.00, speed: 0.03, force: 0.1, epsilon: {inner: 0.01, outer: 0.01}}"
ros2 action send_goal -f /franka_gripper/move franka_msgs/action/Move "{width: 0.08, speed: 0.03}"

PROCEDURA CALIBRAZIONE
Preparare il robot:
1. Spegni tutto
2. Smonta e-e
3. Accendi
4. IMPORTANTE: Cambia profilo e-e: Desk>Settings>End Effector>Attiva "Migrated Profile"
5. Avvia moveit_launch mettendo nel launchfile MIO_calib.rviz come configurazione
6. Settare i parametri sul plugin moveit
7. Calibrare
8. Generare il file launch da cui prendere i dati
9. Copiare i dati nello script camera_transform (in RealRobotFranka>calibration) ed eseguire.
10. Copiare i dati nel MIO_static_transform (in RealRobotFranka>launch)
11. Spegni tutto
12. Rimonta e-e
13. IMPORTANTE: Ricorda di cambiare profilo in quello del Franka Hand

Avviare launch moveit_config cambiando la configurazione


PROMPT DEMO: 
The kids have forgotten the toys on the table, clean them
