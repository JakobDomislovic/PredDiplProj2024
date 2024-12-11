#!/usr/bin/env python3
import rospy

class TestFlight:

    def __init__(self):
        print(f'Starting TestFlight node.')
        print(f'Generating objects and navigating towards them.')


if __name__ == '__main__':
    rospy.init_node('laser_scan_to_points')
    TestFlight()
    rospy.spin()

# TODO Find reference frame of TMUX simulation
# TODO Find exact topics for drone's postion and drone's veloctiy
# TODO Write a subscriber and a publisher
# TODO Write a subscriber callback function
# TODO Implement object generation function
# TODO Implement trajectory generation function

    

