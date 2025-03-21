import sys

num_pursuers = int(sys.argv[1])
num_evaders = int(sys.argv[2]) 
type_name = sys.argv[3]

pursuer_nodes = ""
for i in range(1, num_pursuers+1):
  pursuer_nodes += f"""
  <node pkg="multi_target_tracking" type="{type_name}" name="coordination_{i}">
    <param name="drone_name" value="Drone{i}"/>
    <param name="robot_index_ID" type="int" value="{i}"/>
    <param name="number_of_pursuers" value="{num_pursuers}"/>
    <param name="number_of_evaders" value="{num_evaders}"/>  
    <param name="algorithm_to_run" value="$(arg algorithm_to_run)"/>

    <param name="camera_name" value="front_center"/>
    <param name="image_topic" value="/camera_{i}/image"/>
    <param name="communication_range" type="double" value="50"/>
  </node>
"""

launch_file_contents = f"""
<launch>
  <arg name="algorithm_to_run" default="RBG"/>

  {pursuer_nodes}

</launch>
"""

with open('generated.launch', 'w') as f:
  f.write(launch_file_contents)