Run MAPLE and get pose data
############################

MAPLE runs in a Docker container which makes it easy to deploy on different hardware and software environments. You can `learn
more about Docker here <https://www.docker.com/>`_.

Starting and stopping
=======================
Navigate to the directory with your ``docker-compose.yml`` file in it (``~/maple-config`` by default).

Start in background
~~~~~~~~~~~~~~~~~~~~

.. code:: bash

   docker-compose up -d

MAPLE starts in the background, attempts to restart if it fails, and will automatically start when the coprocessor is powered on.

Start in foreground
~~~~~~~~~~~~~~~~~~~~~

.. code:: bash

   docker-compose up

MAPLE starts in the foreground, displays logs to the terminal, and can be stopped by pressing ``ctrl-c``. Useful when configuring
and calibrating cameras. Will not automatically restart if it fails or start when coprocessor is powered on.

Stop
~~~~~

.. code:: bash

   docker-compose down

Stops MAPLE running in the background. Will disable MAPLE from starting when coprocessor is powered on.


Logging
=========

A running log file of all MAPLE info, warning, and error messages is available in ``~/maple-config/logs/maple_log.txt``. If
trajectory logging is enabled, a new logfile will be created in ``~/maple-config/logs`` each time MAPLE is started.

Pose data on Network Tables
============================

If a team number is set in the ``config.yml`` file, MAPLE will attempt to `connect to the Network Tables server
using the team number <https://docs.wpilib.org/en/stable/docs/software/networktables/client-side-program.html>`_.

Pose data is available on the table **MAPLE** on topics **position** and **orientation**.

.. list-table::
    :widths: 20 80
    :header-rows: 1
    :class: tight-table

    * - Topic
      - Description

    * - position
      - Vector of robot pose x, y, and z in meters.

    * - orientation
      - Vector of robot orientation roll, pitch, and yaw in degrees.

    * - robot_relative_tagid
      - Vector of detected robot relative tag IDs. No duplicate tag IDs. The best tag detection (lowest error) across all cameras.

    * - robot_relative_x
      - Vector of detected robot relative tag x distances, i.e. the distance from the robot center to the tag in the x axis. See :doc:`./configure` for axis conventions.

    * - robot_relative_y
      - Vector of detected robot relative tag y distances, i.e. the distance from the robot center to the tag in the y axis. See :doc:`./configure` for axis conventions.

    * - robot_relative_yaw
      - Vector of detected robot relative tag yaw angles, i.e. the rotation in degrees from robot forwards to tag forwards. 0 degrees means the robot is perfectly parallel to the tag. See :doc:`./configure` for axis conventions.


Code examples
~~~~~~~~~~~~~~~~

Access the position and orientation arrays from NetworkTables like any other array.

.. tabs::

   .. code-tab:: c++

        #include <networktables/DoubleArrayTopic.h>
        #include <networktables/NetworkTablesInstance.h>

        nt::DoubleArraySubscriber positionSub;
        nt::DoubleArraySubscriber orientationSub;

        void Robot::RobotInit() override {

          // Add to your RobotInit, Command, or Subsystem
          auto table = nt::NetworkTableInstance::GetDefault().GetTable("MAPLE");
          positionSub = table->GetDoubleArrayTopic("position").Subscribe({});
          orientationSub = table->GetDoubleArrayTopic("orientation").Subscribe({});
        }

        void Robot::TeleopPeriodic() override {

          // Get the latest value for position and orientation. If the value hasn't been updated since the last time
          // we read the table, use the same value.
          std::vector<double> position = positionSub.Get();
          std::vector<double> orientation = orientationSub.Get();

          std::cout << "position XYZ: ";
          for (double pos : position) {
            std::cout << pos << " ";
          }
          std::cout << std::endl;

          std::cout << "orientation RPY: ";
          for (double angle : orientation) {
            std::cout << angle << " ";
          }
          std::cout << std::endl;
        }

   .. code-tab:: java

        DoubleArraySubscriber positionSub;
        DoubleArraySubscriber orientationSub;

        @Override
        public void robotInit() {

          // Add to your robotInit, Command, or Subsystem
          NetworkTable table = NetworkTableInstance.getDefault().getTable("MAPLE");
          positionSub = table.getDoubleArrayTopic("position").subscribe(new double[] {});
          orientationSub = table.getDoubleArrayTopic("orientation").subscribe(new double[] {});
        }

        @Override
        public void teleopPeriodic() {

            // Get the latest value for position and orientation. If the value hasn't been updated since the last time
            // we read the table, use the same value.
            double[] position = positionSub.get();
            double[] orientation = orientationSub.get();

            System.out.print("position XYZ: " );
            for (double pos : position) {
              System.out.print(pos + " ");
            }
            System.out.println();

            System.out.print("orientation RPY: " );
            for (double angle : orientation) {
              System.out.print(angle + " ");
            }
            System.out.println();
        }

   .. code-tab:: py

        def robotInit(self):

            # Add to your robotInit, Command, or Subsystem
            table = ntcore.NetworkTableInstance.getDefault().getTable("MAPLE")
            self.positionSub = table.getDoubleArrayTopic("position").subscribe([])
            self.orientationSub = table.getDoubleArrayTopic("orientation").subscribe([])

        def teleopPeriodic(self):

            # Get the latest value for position and orientation. If the value hasn't been updated since the last time
            # we read the table, use the same value.
            position = self.positionSub.get()
            orientation = self.orientationSub.get()

            print("position XYZ: ", position)
            print("orientation RPY: ", orientation)

Updating MAPLE
===============

Navigate to the directory with your ``docker-compose.yml`` file in it and run the following command

.. code:: bash

   docker-compose pull


.. toctree::
   :maxdepth: 1
