import os
import yaml


def create_xacro_file(xacro_target,
                      yaml_file=None,
                      requested_macros=None,
                      boiler_plate_top='',
                      boiler_plate_bot='',
                      num_test=lambda name, num: True,
                      param_test=lambda name, params={}: True,
                      ):
    """
    Purpose: Create a .xacro file to create a custom WAM-V .urdf

    Args:
        xacro_target (str): Target file for writing the xacro to
                            NOTE: will overwrite an existing file
        yaml_file (str): .yaml file with requested macros
        requested_macros (dict): if dict is passed directly => ignore yaml file
        boiler_plate_top (str): String to start the xacro file
        boiler_plate_bot (str): String to end the xacro file
        num_test (function): test if the number of macro types is allowed
        param_test (function): test if a macro call parameters are sensible

    Creates a xacro file at 'xacro_target'

    Returns test_fail (bool): Indicator if the wamv passed compliance tests
    """
    test_fail = False
    # Initialize xacro file
    xacro_file = open(xacro_target, 'wb')
    xacro_file.write(boiler_plate_top)

    # If requested_macros not given, then open yaml_file
    if requested_macros is None:
        s = open(yaml_file, 'r')
        requested_macros = yaml.safe_load(s)

        # Handle case with empty yaml file
        if requested_macros is None:
            xacro_file.write(boiler_plate_bot)
            xacro_file.close()
            return

    # Object must be available
    for key, objects in requested_macros.items():
        # Check if number of objects is valid
        test_fail = num_test(key, len(objects))

        # Create block for each object
        xacro_file.write('    <!-- === %s === -->\n' % key)
        for i in objects:
            # Check for valid parameters
            test_fail = param_test(key, i)

            # Write macro
            # Strip all astrisks from key value to allow us to call
            # the same macro multiple times.
            xacro_file.write('    ' + macro_call_gen(key.strip('*'), i))
        xacro_file.write('\n')

    xacro_file.write(boiler_plate_bot)
    xacro_file.close()
    return test_fail


def add_gazebo_thruster_config(xacro_target,
                               yaml_file=None,
                               requested_macros=None,
                               boiler_plate_top='',
                               boiler_plate_bot='',
                               ):
    """
    Purpose: Append gazebo thruster config tags to a .xacro file to
             create a custom WAM-V .urdf

    Args:
        xacro_target (str): Target file for writing the xacro to
                            NOTE: will append an existing file
                                  should be used on thruster
                                  xacro file created by
                                  create_xacro_file()
        yaml_file (str): .yaml file with requested macros
        requested_macros (dict): if dict is passed directly => ignore yaml file
        boiler_plate_top (str): First string to append to the xacro file
        boiler_plate_bot (str): Last string to append to the xacro file

    Appends gazebo thruster config tags to 'xacro_target'
    """
    # Initialize xacro file for appending
    xacro_file = open(xacro_target, 'ab')
    xacro_file.write(boiler_plate_top)

    # If requested_macros not given, then open yaml_file
    if requested_macros is None:
        s = open(yaml_file, 'r')
        requested_macros = yaml.safe_load(s)

        # Handle case with empty yaml file
        if requested_macros is None:
            xacro_file.write(boiler_plate_bot)
            xacro_file.close()
            return

    # WAM-V Gazebo thrust plugin setup
    for key, objects in requested_macros.items():
        for obj in objects:
            xacro_file.write('      ' +
                             macro_call_gen('wamv_gazebo_thruster_config',
                                            {'name': obj['prefix']}))

    xacro_file.write(boiler_plate_bot)
    xacro_file.close()


def macro_call_gen(name, params={}):
    macro_call = '  <xacro:%s ' % name
    endline = '/>\n'
    insert = []
    for i in params:
        if i[:3] == '/**':
            endline = '>\n'
            insert.append(i[3:])
        else:
            macro_call += '%s="%s" ' % (i, str(params[i]))
    macro_call += endline
    if insert == []:
        return macro_call
    for i in insert:
        macro_call += '    <%s>\n' % i
        macro_call += str(params['/**' + i])
        macro_call += '    </%s>\n' % i
    macro_call += '  </xacro:' + name + '>\n'
    return macro_call


# Note: all functions below are not currently used, but they are intended
#       to be used by compliance tests.
def get_macros(directory):
    xacro_files = get_macro_files(directory)
    macros = {}
    for i in xacro_files:
        name, params = parse_xacro_file(i)
        macros[name] = params
    return macros


def get_macro_files(directory):
    xacro_files = [directory+'/'+f for f in os.listdir(directory)
                   if os.path.isfile(os.path.join(directory, f)) and
                   (f[-6:] == '.xacro')]
    child_directories = [d[0] for d in os.walk(directory)]
    child_directories = child_directories[1:]
    for i in child_directories:
        for j in get_macro_files(i):
            xacro_files.append(j)
    return xacro_files


def parse_xacro_file(xacro_file_name):
    xacro_file = open(xacro_file_name, 'r')
    contents = xacro_file.read()
    # remove comment blocks
    while '<!--' in contents:
        start = contents.find('<!--')
        end = contents.find('-->')
        contents = contents.replace(contents[start:end+3], '')
    # get macro declaration
    start = contents.find('<xacro:macro')
    end = contents.find('>', start)
    declaration = contents[start:end]

    contents = contents.replace(contents[start:end+1], '')
    # remove the declaration from the string so we know we processed it
    name_pose = declaration.find('name')
    start = declaration.find('"', name_pose)
    end = declaration.find('"', start+1)
    name = declaration[start+1:end]

    params_pose = declaration.find('params')
    start = declaration.find('"', params_pose)
    end = declaration.find('"', start+1)
    params_str = declaration[start+1:end].split(' ')
    params = {}
    for i in params_str:
        i = i.replace('\n', '')
        if i != '':
            key = i
            if ':' in i:
                key = i[:i.find(':')]
            if '=' in i:
                value = i[i.find('=')+1:]
            else:
                value = ''
            value = value.replace('\n', '')
            params[key] = value

    if contents.find('<xacro:macro') != -1:
        raise Exception('multiple macros defined in %s' % xacro_file_name)

    return name, params
