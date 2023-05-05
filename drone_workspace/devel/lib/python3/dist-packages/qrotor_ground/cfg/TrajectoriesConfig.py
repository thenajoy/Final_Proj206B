## *********************************************************
##
## File autogenerated for the qrotor_ground package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'name': 'Default', 'type': '', 'state': True, 'cstate': 'true', 'id': 0, 'parent': 0, 'parameters': [{'name': 'trajectory', 'type': 'int', 'default': 1, 'level': 0, 'description': 'A trajectory mode usign enum', 'min': 0, 'max': 3, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': "{'enum': [{'name': 'StraightLine', 'type': 'int', 'value': 0, 'srcline': 6, 'srcfile': '/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/cfg/Trajectories.cfg', 'description': 'a straight line', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'Circle2D', 'type': 'int', 'value': 1, 'srcline': 7, 'srcfile': '/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/cfg/Trajectories.cfg', 'description': '2d circular trajectory', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'Circle3D', 'type': 'int', 'value': 2, 'srcline': 8, 'srcfile': '/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/cfg/Trajectories.cfg', 'description': '3d circular trajectory', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'Ellipse2D', 'type': 'int', 'value': 3, 'srcline': 9, 'srcfile': '/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/cfg/Trajectories.cfg', 'description': '2d elliptical trajectory', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'Enum to set trajectory'}", 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'send', 'type': 'bool', 'default': False, 'level': 0, 'description': 'call rosservice', 'min': False, 'max': True, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': '', 'ctype': 'bool', 'cconsttype': 'const bool'}], 'groups': [{'name': 'Circle2D', 'type': 'tab', 'state': True, 'cstate': 'true', 'id': 1, 'parent': 0, 'parameters': [{'name': 'cx', 'type': 'double', 'default': 0.0, 'level': 0, 'description': 'circle center x', 'min': -2.0, 'max': 2.0, 'srcline': 17, 'srcfile': '/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/cfg/Trajectories.cfg', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'cy', 'type': 'double', 'default': 0.0, 'level': 0, 'description': 'circle center y', 'min': -2.0, 'max': 2.0, 'srcline': 18, 'srcfile': '/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/cfg/Trajectories.cfg', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'cz', 'type': 'double', 'default': 0.0, 'level': 0, 'description': 'circle center z', 'min': -2.0, 'max': 2.0, 'srcline': 19, 'srcfile': '/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/cfg/Trajectories.cfg', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'radius', 'type': 'double', 'default': 1.0, 'level': 0, 'description': 'circle radius', 'min': 0.0, 'max': 2.0, 'srcline': 20, 'srcfile': '/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/cfg/Trajectories.cfg', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'phase', 'type': 'double', 'default': 0.0, 'level': 0, 'description': 'circle phase', 'min': -180.0, 'max': 180.0, 'srcline': 21, 'srcfile': '/home/cc/ee106b/sp23/class/ee106b-abr/Desktop/drone_workspace/src/qrotor_firmware-devel/qrotor_ground/cfg/Trajectories.cfg', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}], 'groups': [], 'srcline': 124, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'class': 'DEFAULT::CIRCLE2D', 'parentclass': 'DEFAULT', 'parentname': 'Default', 'field': 'DEFAULT::circle2d', 'upper': 'CIRCLE2D', 'lower': 'circle2d'}], 'srcline': 246, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'class': 'DEFAULT', 'parentclass': '', 'parentname': 'Default', 'field': 'default', 'upper': 'DEFAULT', 'lower': 'groups'}

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

Trajectories_StraightLine = 0
Trajectories_Circle2D = 1
Trajectories_Circle3D = 2
Trajectories_Ellipse2D = 3
