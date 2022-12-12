^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package foxglove_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
