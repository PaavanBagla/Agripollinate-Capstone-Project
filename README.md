# Agripollinate-Capstone-Project

Steps To Run:

# 1. Run the Farm world Simulation
In terminal 1 run:
```bash
cd Agripollinate-Capstone-Project/capstone_ws/
colcon build --symlink-install
source install/setup.bash 
ros2 launch bee_farm_sim launch_sim.launch.py
```

# 2. Run the Rviz to see the scan msgs
In terminal 2 run:
```bash
rviz2
```

# 3. Run the Heatmap Generator
In terminal 3 run:
```bash
cd Agripollinate-Capstone-Project/capstone_ws/
source install/setup.bash 
ros2 run bee_farm_sim bee_heatmap_generator
```
