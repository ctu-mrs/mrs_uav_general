#!/usr/bin/env python3
import roslaunch
import rospy
import rosparam
import sys
import os
import time
import errno
import copy
from socket import error as socket_error
from socket import gethostbyname 

TOPIC_START_PORT = 17000
SERVICE_START_PORT = 6000

def print_help():
    print("""
    Runs nimbro_network according to settings in 'config.yaml' files. 
    This script accepts multiple config files as arguments (See usage).

    Usage:  
            rosrun mrs_uav_general run_nimbro.py config.yaml
            rosrun mrs_uav_general run_nimbro.py config1.yaml config2.yaml config3.yaml
            rosrun mrs_uav_general run_nimbro.py `rospack find MY_PACKAGE`/config/config1.yaml
    """)


class Task:
    """
    Class to launch multiple ros launch files
    """
    def __init__(self, argv):
        UAV_NAME = os.getenv('UAV_NAME')
        if UAV_NAME == None:
            print("Missing environment variable UAV_NAME")
            sys.exit(0)
            
        paramlist = []
        try:
            for i in range(1, len(argv)):
                paramlist.append(rosparam.load_file(filename = argv[i])[0])

            robot_names_list = None
            for params, ns in paramlist:
                if  'network' in params:
                    if 'robot_names' in params['network']:
                        robot_names_list = params['network']['robot_names']

            if robot_names_list is None:
                rospy.logerr('!!! List of robot names described by \'network.robot_names\' parameter is missing in config files !!!')
                sys.exit(0)

            this_uav_ip_address = gethostbyname(UAV_NAME)
            this_uav_last_item_in_ip_address = this_uav_ip_address.split('.')[-1] 

            self.wait_for_roscore()
            uuid = roslaunch.rlutil.get_or_generate_uuid(options_runid = None, options_wait_for_master = False)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.scriptapi.ROSLaunch()
            launch.start()

            for i in range(0, len(robot_names_list)): 
                if robot_names_list[i] == UAV_NAME:
                    continue
                ip_address = gethostbyname(robot_names_list[i])
                last_item_in_ip_address = ip_address.split('.')[-1] 

                ## | ---------------------- Topic sender ---------------------- |
                
                package = 'nimbro_topic_transport'
                executable = 'sender'
                name = 'nimbro_topic_sender_{}'.format(robot_names_list[i])
                namespace = UAV_NAME
                args = '_destination_addr:={} _port:={}'.format(str(robot_names_list[i]), 
                                                                int(TOPIC_START_PORT) + int(this_uav_last_item_in_ip_address))
               
                # rosparam load
                param_ns = ''.join([UAV_NAME, '/', name])
                has_params = False
                for params, ns in paramlist:
                    if 'topics' in params:
                        udp_topics = []
                        #check if the list is empty
                        if params['topics'] is None:
                            continue
                        for topic in params['topics']:
                            # check that each topic has the name parameter
                            if not 'name' in topic:
                                rospy.logerr_once('One of the topics doesn\'t contain \'name\' part.') 
                                sys.exit(1)

                            new_topic = copy.copy(topic)
                            #  check if the namespace of UAV should be added
                            if topic['name'][0] != '/':
                                new_topic['name'] = ''.join(['/', UAV_NAME, '/', topic['name']])
                            
                            udp_topics.append(new_topic)
                            
                        rosparam.upload_params(param_ns, {'udp_topics' : udp_topics})
                        has_params = True

                if not has_params:
                    rospy.logwarn_once('Sender doesn\'t have any *topics* specified in config files. Not running the topic sender nodes!')
                else:
                    # Start node
                    node = self.define_node(package, executable, name, namespace, args)
                    launch.launch(node)

                ## | --------------------- Topic receiver --------------------- |

                package = 'nimbro_topic_transport'
                executable = 'receiver'
                name = 'nimbro_topic_receiver_{}'.format(robot_names_list[i])
                namespace = UAV_NAME
                args = '_port:={}'.format(int(TOPIC_START_PORT) + int(last_item_in_ip_address))
                
                # Start node
                node = self.define_node(package, executable, name, namespace, args)
                launch.launch(node)

                ## | --------------------- Service client --------------------- |

                package = 'nimbro_service_transport'
                executable = 'udp_client'
                name = 'nimbro_service_client_{}'.format(robot_names_list[i])
                namespace = UAV_NAME
                args = '_server:={} _port:={}'.format(str(robot_names_list[i]), int(SERVICE_START_PORT) + int(this_uav_last_item_in_ip_address))
               
                # rosparam load
                param_ns = ''.join([UAV_NAME, '/', name])
                has_params = False
                for params, ns in paramlist:
                    if  'services' in params:
                        #check if the list is empty
                        if params['services'] is None:
                            continue
                        processed_services = []
                        for service in params['services']:
                            # check that each service has the name parameter
                            if not 'name' in service:
                                rospy.logerr_once('One of the services doesn\'t contain \'name\' part.') 
                                sys.exit(1)

                            new_service = copy.copy(service)

                            #  check if the namespace of UAV should be added
                            if service['name'][0] != '/':
                                new_service['name'] = ''.join(['/', robot_names_list[i], '/', service['name']])

                            processed_services.append(new_service)

                        rosparam.upload_params(param_ns, {'services' : processed_services})
                        has_params = True

                if not has_params:
                    rospy.logwarn_once('Service client doesn\'t have any *services* specified in config files. Not running the service client nodes!')
                else:
                    # Start node
                    node = self.define_node(package, executable, name, namespace, args)
                    launch.launch(node)

                ## | --------------------- Service server --------------------- |

                package = 'nimbro_service_transport'
                executable = 'udp_server'
                name = 'nimbro_service_server_{}'.format(robot_names_list[i])
                namespace = UAV_NAME
                args = '_port:={}'.format(int(SERVICE_START_PORT) + int(last_item_in_ip_address))
                
                # Start node
                node = self.define_node(package, executable, name, namespace, args)
                launch.launch(node)

            launch.start()
            launch.spin()

        except Exception as e:
            print(e) 
            sys.exit(1)
            return

    def define_node(self, package, executable, name, namespace, args):
        """
        Define ROS node
        """
        return roslaunch.core.Node(package=package, node_type=executable, name=name, namespace=namespace,
                                     machine_name=None, args=args,
                                     respawn=False, respawn_delay=0.0,
                                     remap_args=None, env_args=None, output='screen', cwd=None,
                                     launch_prefix=None, required=False, filename='<unknown>') 
             

    def wait_for_roscore(self):
        """
        Wait till roscore is found.
        """
        ros_master_available = False
        while not rospy.is_shutdown() and not ros_master_available:
            try:  
                 # query ROS Master for published topics   
                 rospy.get_published_topics()  
                 ros_master_available = True
            except socket_error as serr:  
                if serr.errno != errno.ECONNREFUSED:               
                    raise serr  # re-raise error if its not the one we want     
                else:    
                    print("Waiting for roscore")  
            time.sleep(0.2)   

if __name__ == '__main__':
    if len(sys.argv) == 1:
        print_help()
        sys.exit(0)

    Task(sys.argv)
