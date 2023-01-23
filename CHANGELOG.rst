^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrs_general
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.4 (2023-01-20)
------------------
* Delete nimbro.launch
* fixing missing args
* quick core launch fix
* fixed launch, env->optenv
* added missing uav_name params in core.launch
* Adding masks
* added can takeoff publisher
* kill nimbro if one of the specified hostnames could not be resolved
* Updaing uvdar mask
* Adding uvdar masks
* updating Italy safety area
* added Italy safety areas
* Adding two more calibration for UV cameras.
* Contributors: Dan Hert, Pavel Petracek, Tomas Baca, Viktor Walter, Vojtech Spurny, parakh

1.0.3 (2022-05-09)
------------------
* more masks
* Add trajectory checker
* Adding masks for the new UAV12 (lemmy) platform
* Updating a mask
* Adding masks for the x500 UAV14 (Nick)
* updated louka
* added garmin_down for x500
* updated vltava safety area
* Add Temesvar world mockup
* Add world for Temesvar marine
* Add temesvar louka world
* updated run_nimbro.py
* fixed remmaping in run_nimbro.py
* upgdated help in run_nimbro.py script
* update run_nimbro.py and modified communication_config for usage with this script
* added nimbro_launcher script
* added namespace to bluefox cameras, removed unused args
* fixed bluefox in sensors.launch, removed darpa bluefox
* fixed bluefox_down in sensors
* added bluefox front
  New calibration for basler_23743718
* New calibration for basler_23743718.yaml
* re-enabled actuator control plugin
* adding realsense down to sensors
* added missing uav names for sim
* updated check_ros_package scirpt
* blacklisted more mavros plugins
* fixed prints in automatic start
* + install in cmakelists
* added static transform for x500 and rplidar
* adding large safety area simulation world
* added ros package checking script
* removed redundant UAV_MASS argument
* updated path to realsense launch file in sensors.launch, kept only realsense_front and realsense_up_down options
* pedantically fix indentation in launch files
* fixed UAV_MASS param in core.launch
* fixed env variable loading in core.launch
* updated world files
  Add camera calibration file  UAV60
* Add camera calibration file  UAV60
* add world_farm_seven_stars.yaml
* fixed realsense tf
* Contributors: Dan Hert, Giuseppe Silano, Matej Petrlik, Pavel Petracek, Tomas Baca, Tomáš Báča, Vaclav Pritzl, Viktor Walter, Vit Kratky, id_rsa_drone_login, parakh, vojta

1.0.2 (2021-10-04)
------------------
* basler calibrations
* swapped cameras on green
* add world svetovar
* udpated sensors and cameras
* launch static tf of ouster for naki
* Adding UVDAR masks
* Adding UVDAR calibrations
* add world_naki
* Adding new masks
* updated sensors for dofec
* adding info publishers
* minor changes
* added fork of rosbag, with added features
* adding rosbag checker script
* updated usmocku world
* updated mobius in sensors
* adding mobius ;-)
* added basler cameras from green
* add left_right basler cameras to sensors
* added left/right disambiguation to basler cameras
* updated mass loading for simulation
* added realsense_tii_front to sensors.launch
* updated nimbro config
* Contributors: Dan Hert, Matej, Matej Petrlik, Pavel, Pavel Petracek, Tomas Baca, Vaclav Pritzl, Viktor Walter, Vit Kratky

1.0.1 (2021-05-16)
------------------
* version -> 1.0.1
* updated ros::shutdown
* removed csv tracker form core.launch
* updated Cisar world
* add uav_type switch to realsense_up_down in sensors.launch
* added tii net world
* remapped the joystick topic to be namespaced and command only a single drone
* added path to nimbro config
* added node crash checker
* Merge branch 'master' of github.com:ctu-mrs/mrs_uav_general
* adding bluefox front
* updated uav_names
* added klaxalk-xps to robot names
* updated nimbro config
* added farmstay basin world
* updated CI
* Merge branch 'master' of github.com:ctu-mrs/mrs_uav_general
* added realsense_up_down sensor to sensors.launch
* fixed double -> bool variables
* Merge branch 'master' of https://github.com/ctu-mrs/mrs_uav_general
* updated uav names
* Merge branch 'master' of https://github.com/ctu-mrs/mrs_uav_general
* updated world files to be compatible with odometry
* Merge branch 'master' of github.com:ctu-mrs/mrs_uav_general
* Fixing typo
* updated garmin params in px4 config
* Contributors: Matej Petrlik, Matouš Vrba, Tomas Baca, Viktor Walter

1.0.0 (2021-03-18)
------------------
* Major release

0.0.6 (2021-03-16)
------------------
* Noetic-compatible
* +Automatic start node
* World-definition overhaul, configs overhaul
* Contributors: Daniel Hert, Matej Petrlik, Matej Petrlik Pavel Petracek, Robert Penicka, Tomas Baca, Viktor Walter, Vojtech Spurny, Afzal

0.0.5 (2020-02-26)
------------------

* updates in sensors.launch and core.launch
* updates in camera calibration
* Contributors: Matej Petrlik, Matouš Vrba, Pavel Petracek, Petr Stepan, Tomas Baca, UAV_DRONA, Vaclav Pritzl, uav44, uav66, uav71

0.0.4 (2020-02-18)
------------------
* added bluefox net, removed bluefo sky
* renaming thermal cameras for india
* update sensots launch file for echo
* fixed loading of custom config in nimbro
* uncommented source bash in broadcast ip scrupt
* update mrs general based on desert experiments
* fixed setting broadcast ip for nimbro
* Sensors for lima and echo
* fixed core.launch
* added ROS_IP and ROS_MASTER_URI check to core.launch
* fix world ch2 simulation
* updated challenge2 world
* updated safety area
* changed auto start launchfile
* kilo camera calibrations
* Adding calibration file for Hotel
* mapped ball challenge services
* fixed launch file for realsense in fire challenge
* disabled fire realsense nodelet manager option
* updated autostart
* added sensors check to autostart
* remove koryto world
* udpated autostart launch
* added bluefox sky to sensors.launch
* updated delays for brick challenge
* refactored ifs
* modified realsense for fire challenge and automatic takeoff for the same challenge
* added version check to automatic start
* parametrized takeoff handling challenge
* Sensors launch for foxtrot
* Corrected bf brick calibration
* updated autostart
* incresed delay before launching optflow bluefox
* added desert_building world config
* increased optflow start delay
* updated echo's rs tf
* exclude imu/lidar packets for ouster record
* changed var name
* echo broadcast ip
* setting broadcast ip address before starting nimbro
* broadcast ip parametrized
* fixed custom config in nimbro launch
* mike bluefox_of calibration
* updated automatic start params
* remed out garmin frame id
* added new 5/10 deg realsense brick tf
* updated auto start
* fixed autostart bug
* added bluefox3 calibration
* beautified nimbro configs
* added custom custom config option to nimbro
* added disarming on failed motors setting
* brick nodelet working
* added stop service to auto start
* Snesors.launch for brick_detection nodelet
* LOGGER_DEBUG is optional, default false
* remapped balloons autostart topic
* added logger verbosity switch to nodelet manager
* excluded camera topics in the general record script
* de-niced control nodelet manager
* changed record for better balloons performance
* fix ch2 simulation world
* Sensor realsense brick position
* added bfx3 calibration
* thermal frames
* add world_simulation_challenge2
* updated nimbro config
* updated realsense brick tf
* changed the safety area frames
* reorganized core.launch
* updated bluefox brick tf
* updated brick realsense tf
* increased world_local_hector.yaml safety area, added lidar tf offset for mike
* sensors.launch: added rplidar modes
* sensors: added datapodavac, updated garmin serial condition
* added realsense_fire option to sensors.launch
* added world simulation for challenge 1
* Add Ouster lidar and imu to its TF
* Uncomment back mavros launch (uups)
* updated safety area
* updated worlds
* added thermal cameras to sensors.launch
* changed sensors.launch to use the ball_catch.launch for realsense
* fixed tf grou for brick rs
* updated realsense brick tfs (group for echo and delta)
* fixed record script for realsense, added rules for bluefox3
* added bluefox3 to sensors.launch
* updated nimbro config
* fixed nimbros configs
* fixed nimbro launch
* minor changes
* updated nimbro.launch
* updated uav_names
* updated comment in uav_names.yaml
* updated uav_names.yaml
* fixed uav_names.yaml
* increased simulation safety area height
* updated safety area
* updated core.launch
* added config for local world with safety area in hector_origin
* increased exposure for brick bluefox
* updated koryto world
* New bluefox brick calibrations
* fixed args in core.launch
* added WORLD_NAME parameter to launch file
* Kilo bluefox camera calibrations
* increased safety area height
* added camera calibrations
* updates from uav63
* updated world definitions, added desert world
* latlon origin
* added camera calibrations
* Add ouster OS-1 to sensors.launch
* fixed rplidar and garmin tfs (slashless)
* Decrease gray level for brick bluefox
* Update blufox brick in sensor.launch
* enable other joystick ports
* Correct realsense tf in sensor.launch
* rotated optflow camera for t650
* updated sensors and local world
* more updates with frame names
* fixed launch files
* minor changes, polishing
* updated worlds, update main launch
* fix diagnostics topic for real uavs
* fixed fcu frame in sensors launch
* Added record_vio.sh
* added camera calibration
* Corrected brick tf according Echo drone
* Contributors: Andriy, Dan Hert, Jan Bednar, Kilo UAV, Matej Petrlik, Matouš Vrba, Pavel Petracek, Pavel Petráček, Petr Stepan, Petr Štibinger, Robert Penicka, Tomas Baca, UAV_DRONA, Vaclav Pritzl, Viktor Walter, Vojtech Spurny, afzalhmd14, delta, foxtrot, kratkvit, lima, uav, uav43, uav44, uav60, uav61, uav63, uav64, uav66, uav67, uav71, yrii

0.0.3 (2019-10-25)
------------------
* new record.sh shells scripts
* removed old sensor and record launch files
* created new core.launch and sensors.launch
* + calbration files

0.0.2 (2019-07-01)
------------------
* updated world configs
* updated camp safety area
* added realsense to sensors hector launch file
* Add sensor_naki_hector launch
* updated communication config with services
* Add communication configs for nimbro network
* updated brick record launch file
* added velodyne sensors launch
* added .gitignore for world_current
* updated world camp, removed world current
* added brick launch file
* updates from hector drone
* Add calib file for bluefox on NAKI drone
* added new record options for recording svo
* record just compressed msckf images, not others
* fixed world_camp's safety area
* enable distance_sensor plugin for mavros
* blacklisted mavros debug plugin
* updated mavros config files
* updated hector launch files
* added configs for uav f450 hector
* updated the tracker name in automatic start routines
* added new world file
* Contributors: Tomas Baca, Vojtech Spurny

0.0.1 (2019-05-20)
------------------
