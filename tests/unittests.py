"""
This module defines two test cases for unit testing.
More test cases can be added here for a sequential execution.
Define corresponding output variables in test_plan.py file.
To define more testcases, please refer the __doc__ string of the test_plan file.
"""

import sys
import numpy as np
sys.path.append('../')
import unittest
import test_plan

A1, A2, A3 = 30,25,20       #given link lengths

def forward_kinematics_calc(theta1, theta2, theta3):

    """ 
        Function call to calculate the end efector position given a combination of three angles for a 3 DoF planar robot.
        The equations defined here for x1,x2,x3 and y1,y2,y3 are derived using DH table and a homogenous Transformation matrix
    """
    try:
        x0, y0 = 0,0
        x1 = A1*np.cos(np.radians(theta1))
        y1 = A1*np.sin(np.radians(theta1))
        
        x2 = x1+A2*np.cos(np.radians(theta1+theta2))
        y2 = y1+A2*np.sin(np.radians(theta1+theta2))
        
        x3 = x2+A3*np.cos(np.radians(theta1+theta2+theta3))
        y3 = y2+A3*np.sin(np.radians(theta1+theta2+theta3))

        x = [np.round(x0,4),np.round(x1,4),np.round(x2,4),np.round(x3,4)]
        y = [np.round(y0,4),np.round(y1,4),np.round(y2,4),np.round(y3,4)]
        orientation = np.radians(theta1+theta2+theta3)
        orientation = np.round(orientation,4)
    except:
        raise TypeError
    
    return x,y,orientation

class TestPlanerRobot(unittest.TestCase):
    def test_zero_angle_test(self):
        """
        Zero Angle Test: The arm in a straight horizontal position, along X-axis.
                         Setting all 3 angles to '0'. 
        """

        test_result = forward_kinematics_calc(0,0,0)
        print(f"DEBUG: Angles(deg)={test_result}, Radians={np.radians([90,0,0])}")
        self.assertTupleEqual(test_result, test_plan.expectedResult_1)


    def test_vertical_test(self):
        """
            Vertical Test:  The arm pointing stright up, along Y-axis.
                            Setting theta1 to 90 degrees and the other two angles to '0' degrees.
        """
        test_result = forward_kinematics_calc(90,0,0)
        print(f"DEBUG: Angles(deg)={test_result}, Radians={np.radians([0,90,0,0])}")
        self.assertTupleEqual(test_result,test_plan.expectedResult_2)

    def test_folded_test_link2(self):
        """
            Folded Test: Link 2 of the arm folds back exactly on top of Link1.
                         Setting theta2=180 degrees and the other two angles to '0' degrees.
        """
        test_result = forward_kinematics_calc(0,180,0)
        print(f"DEBUG: Angles(deg)={test_result}, Radians={np.radians([0,0,180,0])}")
        self.assertTupleEqual(test_result,test_plan.expectedResult_3)

    def test_folded_test_link3(self):
        """
            Folded Test: Link 3 of the arm folds back exactly on top of Link2.
                         Setting theta3=180 degrees and the other two angles to '0' degrees.
        """
        test_result = forward_kinematics_calc(0,0,180)
        print(f"DEBUG: Angles(deg)={test_result}, Radians={np.radians([0,0,180,0])}")
        self.assertTupleEqual(test_result,test_plan.expectedResult_4)


if __name__ == '__main__':
    unittest.main()

