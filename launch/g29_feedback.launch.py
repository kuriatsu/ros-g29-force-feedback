<launch>
<rosparam command="load" file="$(find ros_g29_force_feedback)/config/g29.yaml"/>

 <node name="ros_g29_force_feedback_node" pkg="ros_g29_force_feedback" type="ros_g29_force_feedback" output="screen"/>
</launch>
