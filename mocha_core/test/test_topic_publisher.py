#!/usr/bin/env python3

import os
import sys
import time
import unittest
import warnings

import rclpy
import yaml
from colorama import Fore, Style


class test_topic_publisher(unittest.TestCase):
    def setUp(self):
        test_name = self._testMethodName
        print("\n", Fore.RED, 20 * "=", test_name, 20 * "=", Style.RESET_ALL)

        # Ignore pesky warnings about sockets
        warnings.filterwarnings(
            action="ignore", message="unclosed", category=ResourceWarning
        )

        # Initialize ROS2 if not already done
        if not rclpy.ok():
            rclpy.init()

        super().setUp()

    def tearDown(self):
        # Cleanup topic publisher
        if hasattr(self, 'topic_publisher'):
            self.topic_publisher.shutdown()
        if hasattr(self, 'publisher_node'):
            self.publisher_node.destroy_node()
        
        time.sleep(1)
        super().tearDown()

    def test_topic_publisher_creates_topics(self):
        """Test that topic publisher creates expected topics based on testConfigs"""
        # Create topic publisher node for basestation
        self.publisher_node, self.topic_publisher = tp.create_topic_publisher_node("basestation")
        
        # Wait for setup
        time.sleep(1.0)
        
        # Get all topics that have publishers
        topic_names_and_types = self.publisher_node.get_topic_names_and_types()
        topic_names = [name for name, _ in topic_names_and_types]
        
        # Based on testConfigs, we should have topics for:
        # charon (ground_robot): /charon/odometry, /charon/pose
        # quad1 (aerial_robot): /quad1/image, /quad1/pose  
        # basestation (base_station): /basestation/target_goals
        expected_topics = [
            "/charon/odometry",
            "/charon/pose", 
            "/quad1/image",
            "/quad1/pose",
            "/basestation/target_goals"
        ]
        
        # Verify that each expected topic exists
        for expected_topic in expected_topics:
            publishers = self.publisher_node.get_publishers_info_by_topic(expected_topic)
            self.assertGreater(len(publishers), 0, 
                             f"No publisher found for expected topic: {expected_topic}")


# Add the mocha_core module path for imports
current_dir = os.path.dirname(os.path.abspath(__file__))
mocha_core_path = os.path.join(current_dir, "..", "mocha_core")
sys.path.append(mocha_core_path)

import topic_publisher as tp

if __name__ == "__main__":
    unittest.main()