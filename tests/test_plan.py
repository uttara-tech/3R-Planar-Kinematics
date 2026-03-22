

"""
  expectedResult_1: The end effector should be fully extended along the x-axis at a1 + a2 + a3 = 30 + 25 + 20 = 75.
  End effector position: x = 75, y = 0
"""
expectedResult_1 = ([0, 30.0, 55.0, 75.0],[0.0, 0.0, 0.0, 0.0],0)

"""
  expectedResult_2: The end effector should be fully extended along the y-axis at a1 + a2 + a3 = 30 + 25 + 20 = 75.
  End effector position: x = 0, y = 75
"""
expectedResult_2 = ([0.0, 0.0, 0.0, 0.0],[0, 30.0, 55.0, 75.0],1.5708)

"""
  expectedResult_3: The second link of the planar manipulator folds back on the first. The end effector position should be a1 - a2 - a3 = 30 - 25 - 20 = -15.
  End effector position: x = -15, y = 0
"""
expectedResult_3 = ([0.0, 30.0, 5.0, -15.0],[0.0, 0.0, 0.0, 0.0],3.1416)

"""
  expectedResult_4: The third link of the planar manipulator folds back on the first. The end effector position should be a1 + a2 - a3 = 30 + 25 - 20 = 35.
  End effector position: x = -15, y = 0
"""
expectedResult_4 = ([0.0, 30.0, 55.0, 35.0],[0.0, 0.0, 0.0, 0.0],3.1416)
