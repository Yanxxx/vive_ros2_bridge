# ROS2 HTC Vive Bridge

![ROS2 Version](https://img.shields.io/badge/ROS2-Humble-blue)
![Platform](https://img.shields.io/badge/Ubuntu-22.04-orange)
![License](https://img.shields.io/badge/License-MIT-green.svg)

A simple and effective ROS2 bridge for HTC Vive devices, specifically tested on **Ubuntu 22.04** with **ROS2 Humble**. This node captures the position and orientation of Vive controllers and trackers and broadcasts them as `/tf` transforms.

This allows you to easily integrate real-world motion tracking into your ROS2 projects, such as for robot teleoperation, virtual reality interfaces, or motion capture experiments.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Configuration: Using Without a Headset](#configuration-using-without-a-headset)
- [Usage](#usage)
- [Published Transforms](#published-transforms)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## Prerequisites

Before you begin, ensure you have the following installed and configured:

* **OS:** Ubuntu 22.04 LTS
* **ROS2:** ROS2 Humble Hawksbill
* **Steam & SteamVR:** You must have Steam and SteamVR installed and running correctly on your Ubuntu system.
* **Hardware:** An HTC Vive setup (Base Stations and at least one controller or tracker).

## Installation

This is a standard ROS2 package.

1.  **Navigate to your workspace:**
    Open a terminal and navigate to the `src` directory of your ROS2 workspace.

    ```bash
    cd ~/ros2_ws/src
    ```

2.  **Clone the repository:**
    Clone this repository into your `src` folder.

    ```bash
    git clone [https://github.com/your-username/your-repo-name.git](https://github.com/your-username/your-repo-name.git)
    ```
    *(Replace `your-username/your-repo-name` with your actual repository URL)*

3.  **Build the package:**
    Navigate to the root of your workspace and build the package using `colcon`.

    ```bash
    cd ~/ros2_ws
    colcon build --packages-select your_package_name
    ```
    *(Replace `your_package_name` with the actual name of your ROS2 package)*

4.  **Source the workspace:**
    To make the new package available in your environment, source your workspace's setup file.

    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

## Configuration: Using Without a Headset

By default, SteamVR requires an active Head-Mounted Display (HMD) to run. If you want to use only the controllers or trackers without the headset, you need to enable "headless" mode.

1.  **Locate the SteamVR settings file.** It is typically found at:
    ```
    ~/.local/share/Steam/steamapps/common/SteamVR/resources/settings/default.vrsettings
    ```

2.  **Edit the file.** Open `default.vrsettings` with a text editor and find the following line:
    ```json
    "requireHmd": true,
    ```

3.  **Change the value to `false`**:
    ```json
    "requireHmd": false,
    ```

4.  **Save the file and restart SteamVR.** SteamVR will now launch without needing the HMD to be connected.

## Usage

1.  **Start SteamVR:** Make sure your Vive Base Stations are powered on, your controllers/trackers are active, and SteamVR is running and detecting the devices.

2.  **Source your workspace:**
    Open a new terminal and source your ROS2 workspace.
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

3.  **Launch the node:**
    Use `ros2 run` to start the bridge node.
    ```bash
    ros2 run your_package_name vive_bridge_node
    ```
    *(Replace `your_package_name` and `vive_bridge_node` with the actual package and executable names)*

4.  **Visualize the transforms:**
    You can now use tools like RViz2 to see the broadcasted `/tf` frames. Add a TF display and set the fixed frame to `world` or your Vive's base frame to see the controllers moving in real-time.

## Published Transforms

The node publishes the 6-DoF pose (location and quaternion) of each detected device. The transforms are broadcasted between a parent frame (e.g., `world`) and a child frame representing the device.

* `world` -> `vive_controller_1`
* `world` -> `vive_tracker_1`
* ...and so on for all connected devices.

## Troubleshooting

* **"Waiting for devices..."**: If the node starts but doesn't detect any devices, ensure SteamVR is running and has successfully detected your controllers/trackers. Check the status icons in the small SteamVR window.
* **Permission Issues:** You may need to adjust udev rules for SteamVR to access the USB devices correctly. Refer to SteamVR for Linux documentation if you encounter this.

## Contributing

Contributions are welcome! If you find a bug or have a suggestion for improvement, please feel free to open an issue or submit a pull request.

1.  Fork the Project
2.  Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3.  Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4.  Push to the Branch (`git push origin feature/AmazingFeature`)
5.  Open a Pull Request

## License

This project is distributed under the MIT License. See the `LICENSE` file for more information.
