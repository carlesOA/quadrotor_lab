## *********************************************************
## 
## File autogenerated for the quadrotor_lab package 
## by the dynamic_reconfigure package.
## Please do not edit.
## 
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 233, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [{'srcline': 259, 'description': 'gravity_compensation', 'max': 20.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'gravity_compensation', 'edit_method': '', 'default': 15.0, 'level': 0, 'min': 10.0, 'type': 'double'}, {'srcline': 259, 'description': 'integral_window_enable', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'integral_window_enable', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 259, 'description': 'x_kp', 'max': 5.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'x_kp', 'edit_method': '', 'default': 0.2, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'x_ki', 'max': 5.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'x_ki', 'edit_method': '', 'default': 0.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'x_kd', 'max': 5.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'x_kd', 'edit_method': '', 'default': 0.4, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'x_integral_limit', 'max': 0.5, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'x_integral_limit', 'edit_method': '', 'default': 0.1, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'x_integral_window_size', 'max': 10000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'x_integral_window_size', 'edit_method': '', 'default': 500, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'y_kp', 'max': 5.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'y_kp', 'edit_method': '', 'default': 0.2, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'y_ki', 'max': 5.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'y_ki', 'edit_method': '', 'default': 0.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'y_kd', 'max': 5.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'y_kd', 'edit_method': '', 'default': 0.4, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'y_integral_limit', 'max': 0.5, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'y_integral_limit', 'edit_method': '', 'default': 0.1, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'y_integral_window_size', 'max': 10000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'y_integral_window_size', 'edit_method': '', 'default': 1000, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'z_kp', 'max': 5.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'z_kp', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'z_ki', 'max': 5.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'z_ki', 'edit_method': '', 'default': 0.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'z_kd', 'max': 5.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'z_kd', 'edit_method': '', 'default': 0.2, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'z_integral_limit', 'max': 0.5, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'z_integral_limit', 'edit_method': '', 'default': 0.04, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'z_integral_window_size', 'max': 10000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'z_integral_window_size', 'edit_method': '', 'default': 1000, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'yaw_kp', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'yaw_kp', 'edit_method': '', 'default': 0.4, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'yaw_ki', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'yaw_ki', 'edit_method': '', 'default': 0.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'yaw_kd', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'yaw_kd', 'edit_method': '', 'default': 0.2, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'yaw_integral_limit', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'yaw_integral_limit', 'edit_method': '', 'default': 0.5, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'yaw_integral_window_size', 'max': 10000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'yaw_integral_window_size', 'edit_method': '', 'default': 500, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'roll_limit', 'max': 2.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'roll_limit', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'pitch_limit', 'max': 2.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'pitch_limit', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'thrust_limit', 'max': 2.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'thrust_limit', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'yaw_limit', 'max': 2.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'yaw_limit', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': 0.0, 'type': 'double'}], 'type': '', 'id': 0}

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

Controller_Off = 'Off'
Controller_On = 'On'