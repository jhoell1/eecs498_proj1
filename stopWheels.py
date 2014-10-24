import ckbot.logical as L

c = L.Cluster()
c.populate(count =2)
right_wheel = c.at.Nx02;
left_wheel = c.at.Nx18;
right_wheel.set_torque(0)
left_wheel.set_torque(0)