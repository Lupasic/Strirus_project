#!/usr/bin/env python


writeFile = open('Output.txt', 'w')


for num in range(19):
    writeFile.write("  leg_left_"+str(num)+"_revolute_controller:\n    type: velocity_controllers/JointVelocityController\n    joint: leg_left_"+str(num)+"_revolute_joint\n    pid: {p: 100.0, i: 0.01, d: 10.0}\n")
print("\n")
for num in range(19):
    writeFile.write("  leg_right_"+str(num)+"_revolute_controller:\n    type: velocity_controllers/JointVelocityController\n    joint: leg_right_"+str(num)+"_revolute_joint\n    pid: {p: 100.0, i: 0.01, d: 10.0}\n")

writeFile.close()
