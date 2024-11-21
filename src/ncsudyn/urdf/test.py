#!/usr/bin/env python3

import pinocchio as pin

from ncsudyn.urdf import urdf


rhea_urdf = urdf.get_robot_urdf("rhea")

model = pin.buildModelFromXML(rhea_urdf)
data = model.createData()

print(model)