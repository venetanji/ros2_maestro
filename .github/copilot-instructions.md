<!-- Use this file to provide workspace-specific custom instructions to Copilot. For more details, visit https://code.visualstudio.com/docs/copilot/copilot-customization#_use-a-githubcopilotinstructionsmd-file -->

This project is a ROS 2 Python node that subscribes to the /face_motors topic (expects an array of motor positions) and sends these values to a Pololu Maestro 12-channel servo controller over serial, following the design pattern of the provided motor_controller.py. Use rclpy and standard ROS 2 best practices.
