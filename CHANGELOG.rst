^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package foxglove_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2023-02-15)
------------------
* Update README with suggestion to build from source, minor fixes
* Do not build docker images, remove corresponding documentation (`#159 <https://github.com/foxglove/ros-foxglove-bridge/issues/159>`_)
* Add option to use permessage-deflate compression (`#152 <https://github.com/foxglove/ros-foxglove-bridge/issues/152>`_)
* Improve launch file documentation, add missing launch file arguments (`#158 <https://github.com/foxglove/ros-foxglove-bridge/issues/158>`_)
* Allow unsetting (deleting) parameters (`#145 <https://github.com/foxglove/ros-foxglove-bridge/issues/145>`_)
* Improve mutex usage (`#154 <https://github.com/foxglove/ros-foxglove-bridge/issues/154>`_)
* Add sessionId to serverInfo (`#153 <https://github.com/foxglove/ros-foxglove-bridge/issues/153>`_)
* Performance improvements (`#151 <https://github.com/foxglove/ros-foxglove-bridge/issues/151>`_)
* Add ROS2 support for calling server-advertised services (`#142 <https://github.com/foxglove/ros-foxglove-bridge/issues/142>`_)
* Add ROS1 support for calling server-advertised services (`#136 <https://github.com/foxglove/ros-foxglove-bridge/issues/136>`_)
* ROS2 smoke test: Increase default timeout 8->10 seconds (`#143 <https://github.com/foxglove/ros-foxglove-bridge/issues/143>`_)
* Fix flaky parameter test (noetic) (`#141 <https://github.com/foxglove/ros-foxglove-bridge/issues/141>`_)
* Always --pull when building docker images in the makefile (`#140 <https://github.com/foxglove/ros-foxglove-bridge/issues/140>`_)
* Fix failed tests not causing CI to fail (`#138 <https://github.com/foxglove/ros-foxglove-bridge/issues/138>`_)
* Fix setting `int` / `int[]` parameters not working (ROS 1) (`#135 <https://github.com/foxglove/ros-foxglove-bridge/issues/135>`_)
* Send ROS_DISTRO to clients via metadata field (`#134 <https://github.com/foxglove/ros-foxglove-bridge/issues/134>`_)
* Communicate supported encodings for client-side publishing (`#131 <https://github.com/foxglove/ros-foxglove-bridge/issues/131>`_)
* Fix client advertised channels not being updated on unadvertise (`#132 <https://github.com/foxglove/ros-foxglove-bridge/issues/132>`_)
* Add support for optional request id for `setParameter` operation (`#133 <https://github.com/foxglove/ros-foxglove-bridge/issues/133>`_)
* Fix exception when setting parameter to empty array (`#130 <https://github.com/foxglove/ros-foxglove-bridge/issues/130>`_)
* Fix wrong parameter field names being used (`#129 <https://github.com/foxglove/ros-foxglove-bridge/issues/129>`_)
* Add parameter support (`#112 <https://github.com/foxglove/ros-foxglove-bridge/issues/112>`_)
* Add throttled logging when send buffer is full (`#128 <https://github.com/foxglove/ros-foxglove-bridge/issues/128>`_)
* Contributors: Hans-Joachim Krauch, John Hurliman

0.3.0 (2023-01-04)
------------------
* Add launch files, add install instructions to README (`#125 <https://github.com/foxglove/ros-foxglove-bridge/issues/125>`_)
* Drop messages when connection send buffer limit has been reached (`#126 <https://github.com/foxglove/ros-foxglove-bridge/issues/126>`_)
* Remove references to galactic support from README (`#117 <https://github.com/foxglove/ros-foxglove-bridge/issues/117>`_)
* Add missing build instructions (`#123 <https://github.com/foxglove/ros-foxglove-bridge/issues/123>`_)
* Use a single reentrant callback group for all subscriptions (`#122 <https://github.com/foxglove/ros-foxglove-bridge/issues/122>`_)
* Fix clang compilation errors (`#119 <https://github.com/foxglove/ros-foxglove-bridge/issues/119>`_)
* Publish binary time data when `use_sim_time` parameter is `true` (`#114 <https://github.com/foxglove/ros-foxglove-bridge/issues/114>`_)
* Optimize Dockerfiles (`#110 <https://github.com/foxglove/ros-foxglove-bridge/issues/110>`_)
* Contributors: Hans-Joachim Krauch, Ruffin

0.2.2 (2022-12-12)
------------------
* Fix messages not being received anymore after unadvertising a client publication (`#109 <https://github.com/foxglove/ros-foxglove-bridge/issues/109>`_)
* Allow to whitelist topics via a ROS paramater (`#108 <https://github.com/foxglove/ros-foxglove-bridge/issues/108>`_)
* Contributors: Hans-Joachim Krauch

0.2.1 (2022-12-05)
------------------
* Fix compilation on platforms where size_t is defined as `unsigned int`
* Contributors: Hans-Joachim Krauch

0.2.0 (2022-12-01)
------------------

* Add support for client channels (`#66 <https://github.com/foxglove/ros-foxglove-bridge/issues/66>`_)
* Add smoke tests (`#72 <https://github.com/foxglove/ros-foxglove-bridge/issues/72>`_)
* Update package maintainers (`#70 <https://github.com/foxglove/ros-foxglove-bridge/issues/70>`_)
* [ROS2]: Fix messages not being received anymore after unsubscribing a topic (`#92 <https://github.com/foxglove/ros-foxglove-bridge/issues/92>`_)
* [ROS2]: Refactor node as a component (`#63 <https://github.com/foxglove/ros-foxglove-bridge/issues/63>`_)
* [ROS2]: Fix message definition loading for `.msg` or `.idl` files not located in `msg/` (`#95 <https://github.com/foxglove/ros-foxglove-bridge/issues/95>`_)
* [ROS1]: Mirror ROS 2 node behavior when `/clock`` topic is present (`#99 <https://github.com/foxglove/ros-foxglove-bridge/issues/99>`_)
* [ROS1]: Fix topic discovery function not being called frequently at startup (`#68 <https://github.com/foxglove/ros-foxglove-bridge/issues/68>`_)
* Contributors: Hans-Joachim Krauch, Jacob Bandes-Storch, John Hurliman

0.1.0 (2022-11-21)
------------------
* Initial release, topic subscription only
