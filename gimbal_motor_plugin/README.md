# Gimbal Motor Plugin

Pure gazebo plugin for a gimbal motor. A gimbal 'motor' is tied to a specific gazebo
joint. It provides a topic for current joint angle (similar to an encoder) and allows
you to set a desired joint angle or desired torque in Nm

Add built plugin to gazebo plugin path:

export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/code/pure_gazebo_playground/gimbal_motor_plugin/build
