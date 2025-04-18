## Detailed Plan & Methods (C++ Specific):

This plan outlines the steps and methods for incrementally testing the migration of the `image_covering_coordination` package's C++ source code from ROS1 to ROS2.

**Core Philosophy: Incremental Migration and Testing**

Avoid migrating all `.cpp`/`.hpp` files at once. Instead, focus on one node or a small, cohesive set of functionalities at a time. Test each migrated piece thoroughly before moving on.

**Testing Layers (C++ Focus):**

1.  **Compilation Testing:** Ensure your C++ code compiles against `rclcpp` and other ROS2 libraries using `ament_cmake`.
2.  **Static Analysis & Linting:** Catch potential issues and enforce code style early.
3.  **Unit Testing:** Test C++ classes and functions in isolation using `gtest`.
4.  **Component/Node Testing:** Verify a single node's ROS2 interfaces (`rclcpp` publishers, subscribers, services, actions, parameters) and basic runtime behavior.
5.  **Integration Testing:** Test the interaction between multiple C++ nodes within your package, often orchestrated using ROS2 launch files and potentially launch testing frameworks.
6.  **System Testing:** Validate the overall package functionality within your larger robotic system (simulation or hardware).

---

### Phase 1: Initial Code Migration & Compilation

* **Action:**
    * Select a primary C++ source file (e.g., `coordinator_node.cpp`) and its corresponding header (`coordinator_node.hpp`) if applicable.
    * Begin translating ROS1 C++ API calls to ROS2 `rclcpp` equivalents (includes, initialization, publishers, subscribers, services, parameters, spinning, logging).
    * Ensure your `CMakeLists.txt` correctly finds necessary ROS2 packages (`find_package(rclcpp REQUIRED)`, etc.), defines executables (`add_executable(...)`), links dependencies (`ament_target_dependencies(...)`), and installs targets (`install(TARGETS ...)`).
* **Testing Method:** **Compilation Testing**
* **Goal:** Verify that the C++ code compiles successfully against ROS2 libraries without errors.
* **How:** Regularly attempt to build your package after making a coherent set of changes.
* **Commands:**
    ```bash
    # Navigate to your ROS2 workspace root
    cd ~/your_ros2_ws

    # Source your ROS2 environment (e.g., Humble, Iron - update if needed)
    # As of 2025-04-04, Humble and Iron are common. Rolling is the development branch.
    source /opt/ros/humble/setup.bash # Or /opt/ros/iron/setup.bash

    # Build only your specific C++ package
    # --symlink-install helps speed up iterations by linking files instead of copying
    colcon build --packages-select image_covering_coordination --symlink-install --event-handlers console_direct+
    ```
* **Check:** Look for C++ compiler errors (syntax, missing headers, linking issues, incorrect `rclcpp` usage). Resolve these before adding more changes.

---

### Phase 2: Static Analysis and Linting

* **Action:** Integrate static analysis tools into your build process.
* **Testing Method:** **Linting**
* **Goal:** Catch common C++ pitfalls, style inconsistencies, and potential bugs without running the code.
* **How:** Add linting dependencies and configurations to your `package.xml` and `CMakeLists.txt`. Common linters include `ament_cppcheck`, `ament_cpplint`, and `ament_lint_cmake`.
* **Setup:**
    * In `package.xml`: Add `<test_depend>ament_cmake_cppcheck</test_depend>`, `<test_depend>ament_cmake_cpplint</test_depend>`, `<test_depend>ament_lint_auto</test_depend>`, `<test_depend>ament_lint_common</test_depend>`.
    * In `CMakeLists.txt`: Add `find_package(ament_cmake_lint_cmake REQUIRED)`, `find_package(ament_cmake_cpplint REQUIRED)`, `find_package(ament_cmake_cppcheck REQUIRED)`, and call `ament_lint_auto()` or specific linting functions if needed.
* **Commands:** Linting checks are typically run as part of the test phase.
    ```bash
    # Build first
    colcon build --packages-select image_covering_coordination --symlink-install

    # Run tests (including linters)
    colcon test --packages-select image_covering_coordination

    # Check detailed results
    colcon test-result --verbose
    ```
* **Check:** Review any reported linting errors or warnings and fix them to improve code quality and maintainability.

---

### Phase 3: Unit Testing Core C++ Logic

* **Action:** Write unit tests for non-ROS-dependent C++ classes and functions (e.g., image processing algorithms, mathematical calculations, state machines).
* **Testing Method:** **Unit Testing with gtest**
* **Goal:** Verify the correctness of isolated C++ components.
* **How:** Use the Google Test (`gtest`) framework, integrated via `ament_cmake_gtest`.
* **Setup:**
    * In `package.xml`: Add `<test_depend>ament_cmake_gtest</test_depend>`. You might also need `<test_depend>gtest</test_depend>`.
    * In `CMakeLists.txt`:
        * Find the package: `find_package(ament_cmake_gtest REQUIRED)`
        * Add your test source files (typically in a `test/` directory, e.g., `test/test_image_utils.cpp`): `ament_add_gtest(my_unit_test test/test_image_utils.cpp)`
        * Link test against libraries if needed: `ament_target_dependencies(my_unit_test PRIVATE ${YOUR_LIBRARY_TARGET})`
    * Write test files (e.g., `test/test_image_utils.cpp`) using `gtest` macros (`TEST`, `EXPECT_EQ`, `ASSERT_TRUE`, etc.).
* **Commands:**
    ```bash
    # Build the package AND its tests
    colcon build --packages-select image_covering_coordination --symlink-install

    # Run the tests for your package
    colcon test --packages-select image_covering_coordination

    # Show detailed test results
    colcon test-result --verbose
    ```
* **Check:** Ensure all C++ unit tests pass. This validates your core algorithms before integrating them with ROS2 communication.

---

### Phase 4: Component/Node Testing (rclcpp Interfaces)

* **Action:** Focus on a single compiled C++ node executable that has been migrated.
* **Testing Method:** **Manual Command-Line Interaction**
* **Goal:** Verify the node starts, interacts correctly via `rclcpp` publishers/subscribers/services/actions, and handles parameters as expected.
* **How:**
    1.  Build your package: `colcon build ...`
    2.  Source the local workspace: `source install/setup.bash`
    3.  Run the specific C++ node executable (defined in your `CMakeLists.txt` with `add_executable` and `install`).
    4.  Use ROS2 CLI tools (`ros2 topic`, `ros2 service`, `ros2 action`, `ros2 param`) in another terminal to inspect and interact with the node.
* **Commands (Examples):**
    ```bash
    # Terminal 1: Source and run your C++ node
    cd ~/your_ros2_ws
    source install/setup.bash
    # Replace <your_cpp_executable> with the name from add_executable() in CMakeLists.txt
    ros2 run image_covering_coordination <your_cpp_executable>

    # Terminal 2: Source and interact
    cd ~/your_ros2_ws
    source install/setup.bash

    # Check node status and interfaces
    ros2 node list
    ros2 node info /<node_name_used_in_code> # e.g., /coordinator_node if you used that name
    ros2 topic list
    ros2 topic echo /some_published_topic
    ros2 service call /some_service <service_type> "{request_field: value}"
    ros2 param list /<node_name_used_in_code>
    ros2 param get /<node_name_used_in_code> <parameter_name>
    ```
* **Check:** Does the C++ node run without crashing? Are `rclcpp` interfaces (topics, services, etc.) advertised correctly? Does the node respond appropriately to CLI interactions? Check the node's terminal output for `RCLCPP_INFO` messages.

---

### Phase 5: Launch File and Integration Testing

* **Action:** Migrate ROS1 XML `.launch` files to ROS2 Python launch files (`.launch.py`). Define how your migrated C++ nodes should run together.
* **Testing Method 1: Manual Launch File Execution**
    * **Goal:** Ensure the launch file starts the C++ nodes correctly with parameters, namespaces, etc.
    * **How:** Execute the launch file and use CLI/GUI tools (`rqt_graph`, `rviz2`) to verify the setup.
    * **Commands:**
        ```bash
        # Navigate, source
        cd ~/your_ros2_ws
        source install/setup.bash

        # Launch (ensure launch files are installed via CMakeLists.txt)
        ros2 launch image_covering_coordination <your_launch_file.launch.py>

        # In another terminal, inspect with ros2 tools, rqt_graph
        ```
    * **Check:** Do all specified C++ nodes start? Are connections correct in `rqt_graph`?

* **Testing Method 2: Automated Integration Tests (Launch Tests)**
    * **Goal:** Automatically verify interactions *between* your running C++ nodes.
    * **How:** Use ROS2 `launch_testing`. While the nodes under test are C++, the test orchestration script itself is typically written in Python.
    * **Setup:**
        * In `package.xml`: Add test dependencies like `<test_depend>launch_testing</test_depend>`, `<test_depend>launch_testing_ament_cmake</test_depend>` (or `launch_testing_ros`), `<test_depend>python3-pytest</test_depend>`.
        * In `CMakeLists.txt`: Use `find_package(ament_cmake_pytest REQUIRED)` and `ament_add_launch_test(test/my_integration_test.launch.py)`.
        * Write a Python launch file (e.g., `test/my_integration_test.launch.py`) that launches your C++ nodes (`launch_ros.actions.Node`) and includes test actions/assertions using `launch_testing` and `pytest`.
    * **Commands:**
        ```bash
        # Build including tests
        colcon build --packages-select image_covering_coordination --symlink-install

        # Run tests (executes unit and launch tests)
        colcon test --packages-select image_covering_coordination

        # Check results
        colcon test-result --verbose
        ```
    * **Check:** Do the automated integration tests pass, confirming expected communication and behavior between your C++ nodes?

---

### Phase 6: System Testing

* **Action:** Test the fully migrated `image_covering_coordination` C++ package within its intended environment (e.g., Gazebo simulation, real robot).
* **Testing Method:** **Scenario-Based Testing**
* **Goal:** End-to-end validation of the package's role in the overall robotic task.
* **How:** Define and execute realistic use cases. Launch the complete system. Use `RViz2` and other relevant tools to monitor behavior.
* **Commands:** `ros2 launch` to start the system, visualization/simulation tools.
* **Check:** Does the system achieve the desired outcome (e.g., area covered, coordination successful)? Does the behavior match expectations based on the C++ logic implemented?

---

### Key C++ Migration Tools & Practices:

* **Version Control (Git):** Absolutely essential. Use a branch, commit often with clear messages.
* **rclcpp API Docs:** Keep the `rclcpp` documentation handy.
* **ament_cmake:** Understand its role in finding packages, defining targets, and installing files.
* **Colcon:** Your primary build and test tool. Use flags like `--packages-select`, `--symlink-install`, `--event-handlers console_direct+`, `--cmake-args -DCMAKE_BUILD_TYPE=Debug` (for debugging symbols).
* **ROS1/ROS2 Bridge:** Use `ros1_bridge` if you need interoperability with ROS1 components during a phased migration.
* **Debugging:** Build with debug symbols (`-DCMAKE_BUILD_TYPE=Debug`) and use `gdb` or IDE debuggers attached to your running C++ nodes. `RCLCPP_DEBUG` logging is also very helpful.
* **Visualization & Introspection:** Master `ros2` CLI tools, `rqt_graph`, and `RViz2`.