## *********************************************************
##
## File autogenerated for the qrotor_ground package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'name': 'Default', 'type': '', 'state': True, 'cstate': 'true', 'id': 0, 'parent': 0, 'parameters': [{'name': 'mode', 'type': 'int', 'default': 0, 'level': 0, 'description': 'A drone command mode', 'min': 0, 'max': 1, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': "{'enum': [{'name': 'Move', 'type': 'int', 'value': 0, 'srcline': 7, 'srcfile': '/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/cfg/DroneCommands.cfg', 'description': 'Move drone by', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'Setpoint', 'type': 'int', 'value': 1, 'srcline': 8, 'srcfile': '/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/cfg/DroneCommands.cfg', 'description': 'Move drone to setpoint', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'Command setpoint to drone'}", 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'x', 'type': 'double', 'default': 0.0, 'level': 0, 'description': 'x in meters', 'min': -2.0, 'max': 2.0, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'y', 'type': 'double', 'default': 0.0, 'level': 0, 'description': 'y in meters', 'min': -2.0, 'max': 2.0, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'z', 'type': 'double', 'default': 0.0, 'level': 0, 'description': 'z in meters', 'min': -2.0, 'max': 3.0, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'yaw', 'type': 'double', 'default': 0.0, 'level': 0, 'description': 'yaw in degrees', 'min': -180.0, 'max': 180.0, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'send', 'type': 'bool', 'default': False, 'level': 0, 'description': 'Send Command', 'min': False, 'max': True, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'bool', 'cconsttype': 'const bool'}, {'name': 'activate_mission', 'type': 'bool', 'default': False, 'level': 0, 'description': 'Send activate mission Command', 'min': False, 'max': True, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'bool', 'cconsttype': 'const bool'}, {'name': 'kill_mission', 'type': 'bool', 'default': False, 'level': 0, 'description': 'Send kill mission Command', 'min': False, 'max': True, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'bool', 'cconsttype': 'const bool'}], 'groups': [{'name': 'CMDs', 'type': 'tab', 'state': True, 'cstate': 'true', 'id': 1, 'parent': 0, 'parameters': [{'name': 'kill', 'type': 'bool', 'default': False, 'level': 0, 'description': 'Send Kill Command', 'min': False, 'max': True, 'srcline': 18, 'srcfile': '/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/cfg/DroneCommands.cfg', 'edit_method': '', 'ctype': 'bool', 'cconsttype': 'const bool'}, {'name': 'arm', 'type': 'bool', 'default': False, 'level': 0, 'description': 'Send Arm Command', 'min': False, 'max': True, 'srcline': 19, 'srcfile': '/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/cfg/DroneCommands.cfg', 'edit_method': '', 'ctype': 'bool', 'cconsttype': 'const bool'}, {'name': 'disarm', 'type': 'bool', 'default': False, 'level': 0, 'description': 'Send Disrm Command', 'min': False, 'max': True, 'srcline': 20, 'srcfile': '/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/cfg/DroneCommands.cfg', 'edit_method': '', 'ctype': 'bool', 'cconsttype': 'const bool'}, {'name': 'takeoff', 'type': 'bool', 'default': False, 'level': 0, 'description': 'Send takeoff Command', 'min': False, 'max': True, 'srcline': 21, 'srcfile': '/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/cfg/DroneCommands.cfg', 'edit_method': '', 'ctype': 'bool', 'cconsttype': 'const bool'}, {'name': 'land', 'type': 'bool', 'default': False, 'level': 0, 'description': 'Send land Command', 'min': False, 'max': True, 'srcline': 22, 'srcfile': '/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/cfg/DroneCommands.cfg', 'edit_method': '', 'ctype': 'bool', 'cconsttype': 'const bool'}, {'name': 'OFFBOARD', 'type': 'bool', 'default': False, 'level': 0, 'description': 'Set OFFBOARD CONTROL MODE', 'min': False, 'max': True, 'srcline': 23, 'srcfile': '/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/cfg/DroneCommands.cfg', 'edit_method': '', 'ctype': 'bool', 'cconsttype': 'const bool'}], 'groups': [], 'srcline': 124, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'class': 'DEFAULT::CMDS', 'parentclass': 'DEFAULT', 'parentname': 'Default', 'field': 'DEFAULT::cmds', 'upper': 'CMDS', 'lower': 'cmds'}], 'srcline': 246, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'class': 'DEFAULT', 'parentclass': '', 'parentname': 'Default', 'field': 'default', 'upper': 'DEFAULT', 'lower': 'groups'}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

DroneCommands_Move = 0
DroneCommands_Setpoint = 1
