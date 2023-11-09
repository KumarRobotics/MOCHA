#!/usr/bin/env python3
import sys
import subprocess
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


def ping_ip(ip_address):
    try:
        # Run the ping command with a single ping packet (-c 1) and a timeout of 1 second (-W 1)
        result = subprocess.run(["ping", "-c", "1", "-W", "1", ip_address],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
        return result.returncode == 0
    except subprocess.CalledProcessError:
        # An error occurred (ping failed)
        return False


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
    rospy.init_node("rajant_query", anonymous=False)

    # Get robot name from the ~robot_name param
    robot_name = rospy.get_param('~robot_name', 'charon')

    # Get robot configs
    robot_configs_file = rospy.get_param("~robot_configs")
    with open(robot_configs_file, "r") as f:
       robot_configs = yaml.load(f, Loader=yaml.FullLoader)
    if robot_name not in robot_configs.keys():
        rospy.signal_shutdown("Robot not in config file")
        rospy.spin()

    # Get radio configs
    radio_configs_file = rospy.get_param("~radio_configs")
    with open(radio_configs_file, "r") as f:
        radio_configs = yaml.load(f, Loader=yaml.FullLoader)
    radio = robot_configs[robot_name]["using-radio"]
    if radio not in radio_configs.keys():
        rospy.shutdown("Radio not in config file")
        rospy.spin()

    # Get path of the package
    rajant_name = robot_configs[robot_name]['using-radio']
    if rajant_name in radio_configs.keys():
        target_ip = radio_configs[rajant_name]['computed-IP-address']
    else:
        rospy.logerr(f"Radio {rajant_name} for robot {robot_name} not found in configs")
        rospy.signal_shutdown("Radio not in configs")
        rospy.spin()


    # Create ros publisher
    pub = rospy.Publisher('ddb/rajant/log', std_msgs.msg.String, queue_size=10)

    rospack = rospkg.RosPack()
    ros_path = rospack.get_path('interface_rajant')

    # Create subprocess for the java app
    java_bin = os.path.join(ros_path, 'scripts',
                            'thirdParty/watchstate/bcapi-watchstate-11.19.0-SNAPSHOT-jar-with-dependencies.jar')

    p = subprocess.Popen(['java',
               '-jar',
               java_bin,
               target_ip], stdout=subprocess.PIPE, close_fds=ON_POSIX)
    q = Queue()
    t = Thread(target=enqueue_output, args=(p.stdout, q))
    t.daemon = True  # thread dies with the program
    t.start()

    # Go
    rospy.loginfo(f"{robot_name} - Rajant API Query - Starting on {rajant_name}")

    # ping the assigned radio
    if ping_ip(target_ip):
        rospy.loginfo(f"{robot_name} - Rajant API Query - ping success")
    else:
        rospy.logerr(f"{robot_name} - Rajant API Query - Rajant ping failed")
        rospy.signal_shutdown("Rajant IP")
        rospy.spin()

    while not rospy.is_shutdown():
        if not t.is_alive():
            rospy.logerr(f'{robot_name}: Rajant Java process died! Restarting...')
            p = subprocess.Popen(['java',
                       '-jar',
                       java_bin,
                       target_ip], stdout=subprocess.PIPE, close_fds=ON_POSIX)
            q = Queue()
            t = Thread(target=enqueue_output, args=(p.stdout, q))
            t.daemon = True  # thread dies with the program
            t.start()

        # This sleep avoids problem with the java process. DO NOT REMOVE IT
        rospy.sleep(1)

        try:
            line = q.get_nowait()
        except Empty:
            # No output yet
            pass
        else:  # got line
            answ_array = line_parser(line)
            while not rospy.is_shutdown():
                try:
                    newline = q.get_nowait()
                except Empty:
                    break
                else:
                    answ_array += line_parser(newline)
            try:
                yaml_res = yaml.load(answ_array, Loader=yaml.Loader)
                if type(yaml_res) == type({}):
                    # rospy.logdebug(str(yaml_res) + "\n")
                    pub.publish(str(yaml_res))
                else:
                    rospy.logerr(f"{robot_name}: YAML from Rajant did not look like an object!")
            except yaml.scanner.ScannerError:
                rospy.logerr(f"{robot_name}: Could not parse YAML from Rajant!")
