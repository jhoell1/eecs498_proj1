This is a README for the EECS 498 Red Team Project Repo.
TL,DR: I needed to have some file so I could create a repo. 

# To Run the simulation code:
#
#
# First, you need to have the pyckbot folder somewhere (the virtual machines have it already)
# Update the pyckbot folder if you have not yet done so:
# from inside the pyckbot folder, run: git pull -u 
#
#
#
# Note: You need two terminals to run everything because you need to run the waypoint server as well as our simulation
#
#
#
# Second:
#   Navigate to pyckbot/apps/hrb
#   run: ipython --pylab
#   From inside ipython, run: execfile("waypointServer.py")
#   It will prompt you for options. enter: -d 127.0.0.1
#
#
#
# Third, in the other terminal:
#   Navigate to wherever our git repo is
#   run: ipython REDsimTagStreamer.py 127.0.0.1
#  
# And that should be it!

