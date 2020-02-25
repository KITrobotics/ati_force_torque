^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ati_force_torque
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Updated README with melodic description
* Added static_application parameter to example configs.
* Adapted namespace for sensor parameters
* Contributors: Denis Stogl

1.1.0 (2020-02-22)
------------------
* Addapted to changes in iirob_filters
* Merge pull request `#9 <https://github.com/KITrobotics/ati_force_torque/issues/9>`_ from KITrobotics/updated_params
  Updated param names for can yaml and config node
* Added config files for config node
* Updated param names for can yaml and config node
* Contributors: Denis Štogl

1.0.1 (2018-12-11)
------------------
* Merge pull request #1 from KITrobotics/destogl-patch-1-1
  Update README.md
* Update README.md
* Merge pull request #2 from KITrobotics/fts_restructuring
  FTS restructuring
* Update driver_socket.launch
* Update CMakeLists.txt
* Update .travis.rosinstall
* Update README.md
* Cleaned stuff moved to force_torque_sensor package
* Added plugin for ATI sensor
* Force Torque sensors restructuring
* Raw with fts (#18)
  * Removed dependecy on schunk_description (#6)
  * Removed dependecy on schunk_description
  * Added xacro dependency
  * changes for working raw with fts
  * some changes for namespace read
  * rosparam working
  * added cs calibration params
  * added necessary code for simulation
  * changes for simulation.need to be cleaned
  * changes in fthandle_sim
  * deleted prints
  * merged master branch with raw_with_fts
  * small changes in urdf
  * compatibility changes for iirob_fitlers
  * Use c++11 and update .travis
  * Update .travis.rosinstall
  * Updated compile files and some Sim files headers
  * made some changes
  * some more changes for PR
  * changes for PR
  * changed in fts.gazebo
  * changes for PR
  * changes for PR and added sim parameter
  * changes for working correctly with rosparam_handler
  * changes to work correctly with rosparam and real raw with fts
  * changes for working with real sensor and simulation
  * not depending on own rosparam_handler anymore
  * chages for simulation of sensor
  * using teleop not cob_teleop anymore
  * remapped to cmd_force
  * changed default values for easier debugging
  * small changes
  * changes to use unchanged sensor_offsets
  * added realtime publisher and working with iirob_led
  * changes to work with iirob led
  * some changes to work with rt publisher and iirob_led after test
  * changes for working with iirob filters after changing to plugin
  * changes in urdf file for correct description when using sensor with sr2
  * changes to work with filters pluginlib
  * moved ati ft for sr2 to sr2_bringup
  * undone small change in urdf
  * small change for sensor simulation
  * Update package.xml
  * Update package.xml
  * Removed iirob_led dependency.
  * working branch with iirob_filters
  * changes to work with wrench stamped as template
  * changes to work with iirob_filters moving mean filter
  * use rosparam handler for gravity compensator
  * changes to work with filters
  * Update sensor_mini58.urdf.xacro
  * Update driver_sim.launch
* Add param handler and simulation for sensor (#11)
  Added simulation support for the sensor. The data are inputed using joystick.
* Updated descriptions and meshes for Sensors. Added ATI Mini58 (#10)
* Remove schunk descr dep (#7)
  * Removed dependecy on schunk_description
  * Added xacro dependency
* Update .travis configs
* Update .travis configs
* Added missing packages for cob_driver
* Added cob_driver package and remove schunk_description dependency
* Use kinetic branch of iirob_filters
* Added kinetic ros-distro
* Update for tests
* Renamed rosinstall file for source dependencies
* Added travis config and updated readme
* Merge branch 'indigo-devel' of github.com:iirob/ati_force_torque into indigo-devel
* Removed schunk_description dependecy, sorted dependencies, added tests for lunch files
* working non-static calibration
* Added automatic calibration
* Added scripts for automatic calculation of offsets and tool CoG.
* Updated paramters names and CoG paramters
* Changed name of offsets paramters
* New service type for calibration and catching average masurements
* Corrected typo in launch files
* Added mutithreading in the node and reduced output of transformations erros. Now is every 100th error shown.
* Added publishing of movingmean filtered data.
* Merge branch 'indigo-devel' into kinetic-devel
* Renamed CMake constant EIGEN3->Eigen
* Added fts_reference_link as default sensor_frame
* Updated xacro and default world_frame
* Merge branch 'indigo-devel' into kinetic-devel
* Create README.md
* Merge branch 'indigo-devel' into kinetic-devel
* Optimisation of parameters
* Merge branch 'indigo-devel' into kinetic-devel
* Paramter name change
* Merge branch 'indigo-devel' into kinetic-devel
* Change of default paramter for future_baudrate and config to use fts_frame on default.
* Merge branch 'indigo-devel' into kinetic-devel
* Merge branch 'indigo-devel' of gitlab.ira.uka.de:iirob/ati_force_torque into indigo-devel
* Updated Eigen dependecy and restructured launch files
* Changed parameters. Removing empty lines.
* Cleaned and median -> moving_mean
* Cleaned and median -> moving_mean
* Updated names. Removed old files.
* Updated names. Removed old files.
* Updated licence header
* added filters to ATI
  included filters from iirob_filters
* Rename of package to ati_force_torque
* Rename of lunch files
* Refractoring of code and licence update.
* Convert config to defaults
* Merge branch 'master' of gitlab.ira.uka.de:iirob/ati_mini_45
* change params to work with new robot definition
* readd service for dynamic recalibration from other branch
* change config parameters to work with new robot layout
* Code cleaning before for publication. Needs to be tested.
* Updated dependencies
* Cleaned lunch file
* SocketCAN config updated
* Check for all return codes
* if no canType mentioned use default constructor
* updated to start
* New files for Socket can
* Extended for SocketCan
* Merge remote-tracking branch 'origin/alex'
  Conflicts:
  ros/src/fts_node.cpp
* Compile under indigo.
* Working GetTemperature Service
* Merge branch 'master' into alex
* safety commit before merge
* disabled calibration on init, customized launch & config files
* Correct compile errors
* Merge branch 'master' of gitlab.ira.uka.de:iirob/ati_mini_45
* config and launch
* Extension to read diagnostic ADC voltages
* added rviz (with config) to launchfile
* Working version
* Calibrate is new function now and called with initialisation
* Filter removed to extern node and added Transformation for FTS.
* Added filter topic. 1st-Order low-pass filter implemented.
* Code cleaning: remove marker publishing
* Finaly, won! Git vs Denis: 0:1
* Really wrong
* Added: status checking, reseting of sensor and some minor code edits
* Error handling
* Merge branch 'ati_baudrate' of gitlab.ira.uka.de:iirob/ati_mini_45
  Conflicts:
  common/include/cob_forcetorque/ForceTorqueCtrl.h
  common/src/ForceTorqueCtrl.cpp
  config/can_ati.yaml
  ros/src/fts_node.cpp
* High rate, workable version, it seams that works really nice
* Added: config node and changed baudrate of FTS.
* wtf commit
* Added force transformation
* Some quite working version
* receiveMsg changed to recieveMsgRetry
* finished migration to tf2
* Migration to tf2
* Error correction: name changeing
* Changed package name
* Changed message type to geometry_msgs/Wrench
* Now read parameters form parameter server, correction of error
* Read paramter from parameter server
* Removed old files for CanESD which is now in cob_generic_can
* Added smo checking, easy changeble ID...
* Catknisation and first working version
* Initial commit
* Contributors: Alexander Pollmann, Andreea Tulbure, Denis Štogl, Format Bot, IIROB Praktikum 1, IPR-SR2, Timo Leitritz, andreeatulbure
