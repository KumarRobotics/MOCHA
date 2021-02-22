#!/usr/bin/env python3
import sys
from subprocess import PIPE, Popen
from threading  import Thread
from queue import Queue, Empty
from pprint import pprint
import sys
import os
import time
import yaml
import re
import pdb
import string
import hashlib
import random
import rospy
import std_msgs.msg
import rospkg

NODE_NAME="rajant_log"

CONFIG_FILE = "robotConfigs.yml"

#TODO import database_server.py for get_robot_number()
rospack = rospkg.RosPack()

comms_path = rospack.get_path('distributed_database')
scripts_path = os.path.join(comms_path, 'scripts')
sys.path.append(scripts_path)
import database_server as ds
import database_server_utils as du

def randomNumber(stringLength=4):
    """Generate a random string of fixed length """
    number = random.randint(1000, 9999)
    return str(number)


def enqueue_output(out, queue):
    """ Saves the output of the process in a queue to be parsed
    afterwards """
    for line in iter(out.readline, b''):
        queue.put(line)
    out.close()


def line_parser(line_bytes):
    """ Returns parsed str version of bytes input line
    This is quite magic: rajant output is not yaml but it is very
    yamlish. If we replace the { with :, we remove }, and we do some
    minor modifications everything works out of the box!"""
    line_str = line_bytes.decode('unicode-escape')
    line_str = line_str.replace("{", ":")
    line_str = line_str.replace("}", "")
    # random numbers are added to avoid overwriting the key on the yaml
    line_str = re.sub("wireless",
                      "wireless-" + randomNumber(), line_str)
    line_str = re.sub("peer",
                      "peer-" + randomNumber(), line_str)
    # MACs are a little bit more tricky
    if line_str.replace(" ", "")[:4] == "mac:":
        separator = line_str.find(":") + 2
        mac_str = line_str[separator:]
        mac_bytes = bytes(mac_str, 'raw_unicode_escape')
        mac_decoded = ":".join(["%02x" % c for c in mac_bytes[1:-2]])
        line_str = line_str[:separator] + mac_decoded + "\n"
    return line_str


ON_POSIX = 'posix' in sys.builtin_module_names

if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=True)

    # Get path of the package
    ros_path = rospack.get_path('interface_rajant')
    
    config_path = rospack.get_path('network_configs')

    comms_path = rospack.get_path('distributed_database')
    scripts_path = os.path.join(comms_path, 'scripts')
    sys.path.append(scripts_path)
    import database_server as ds

    robot_config_yml = os.path.join(config_path, 'config', 'robotConfigs.yml')
    with open(robot_config_yml, 'r') as file:
        robot_cfg = yaml.load(file)

    radio_config_yml = os.path.join(config_path, 'config', 'radioConfigs.yml')
    with open(radio_config_yml, 'r') as file:
        radio_cfg = yaml.load(file)

    robot_name = du.get_robot_name(CONFIG_FILE)

    if robot_name in robot_cfg.keys():
        rajant_name = robot_cfg[robot_name]['using-radio']

    if rajant_name in radio_cfg.keys():
        TARGET_IP = radio_cfg[rajant_name]['computed-IP-address']

    # Create ros publisher
    pub = rospy.Publisher(NODE_NAME + '/log',
                          std_msgs.msg.String, queue_size=10)

    # Create subprocess for the java app
    java_bin = os.path.join(ros_path, 'scripts',
                            'thirdParty/watchstate/bcapi-watchstate-11.19.0-SNAPSHOT-jar-with-dependencies.jar')

    # Also write results to binary file
    filename = "output" + randomNumber() + ".log"
    filename = os.path.join(ros_path, filename)
    file = open(filename, "wb")

    p = Popen(['java',
               '-jar',
               java_bin,
               TARGET_IP], stdout=PIPE, bufsize=1, close_fds=ON_POSIX)
    q = Queue()
    t = Thread(target=enqueue_output, args=(p.stdout, q))
    t.daemon = True  # thread dies with the program
    t.start()

    # Go
    while not rospy.is_shutdown():
        if not t.is_alive():
            rospy.logerr('Java process died! Restarting...')
            p = Popen(['java',
                       '-jar',
                       java_bin,
                       TARGET_IP], stdout=PIPE, bufsize=1, close_fds=ON_POSIX)
            q = Queue()
            t = Thread(target=enqueue_output, args=(p.stdout, q))
            t.daemon = True  # thread dies with the program
            t.start()

        rospy.sleep(1)

        try:
            line = q.get_nowait()
        except Empty:
            # No output yet
            pass
        else:  # got line
            answ_array = line_parser(line)
            file.write(line)
            while not rospy.is_shutdown():
                try:
                    newline = q.get_nowait()
                except Empty:
                    break
                else:
                    file.write(newline)
                    answ_array += line_parser(newline)
            file.flush()
            try:
                yaml_res = yaml.load(answ_array, Loader=yaml.Loader)
                if type(yaml_res) == type({}):
                    print(yaml_res, "\n")
                    pub.publish(str(yaml_res))
                else:
                    rospy.logerr("YAML from Rajant did not look like an object!")
            except yaml.scanner.ScannerError:
                rospy.logerr("Could not parse YAML from Rajant!")

