README.txt
==========
Copyright Ian Dobbie (ian.dobbie@jhu.edu) 2021


This file contains instructions for setting up a simulated microscope
environment using Microscope-Cockpit and Python-Microscope. The
environment includes a large tiled image of which segments are
returned to simulate stage movement and different colour channels
returned to simulate changing an emission filter. The simulated
microscope is then used to test the findNuclei script showing the ease
of extending Cockpit functionality with Python libraries,
Python-OpenCV is used in this case.


Instructions for setting up and testing the findNuclei.py script.
=================================================================


By default with no configuration files Microscope-Cockpit will start
in a simulation mode which enables basic experimentation and testing
of how the system works. However, in this mode the system returns
random noise in its images so is not useful for online image
analysis. 

We have created a simple script to find nuclei in the DAPI channel of
a fluorescent images. To go along with this test script we have
provided a large multi-channel fluorescent image which enables this
setup to be tested, utilising a compound simulation device which
includes simulations of a camera, a stage and a filter wheel. The
simulated camera returns images from a subsection of a large image
(22,000x22,000 pixels with 3 channels). The returned image depends on
the stage XY coordinates to determine which region, and
the filter wheel position to determine which channel of the large
image to return.  

The findNuclei script has two important callable functions, start()
which grabs images that are captured by the camera and then
processed. The complementary stop() function which stops processing
input images.

Once the process has been started with the start() function all
collected images are passed to the analysis routine which uses OpenCV
to find circular objects of the correct size. The location of found
objects are transformed into stage coordinates and added to the marked
points list for later use.

The recommended way to run this process is the load the module, setup
the DAPI channel to have sensible intensity, initiate the processing
by calling start() and then start a spiral mosaic. The system will
then automatically add marks to all found nuclei in the mosaic
images. After you have found enough nuclei stop the mosaic and prevent
future captured images from being processed by calling stop().

Setting up and running this simulation environment requires the
following steps.

1) install python-microscope and microscope-cockpit and opencv-python
pip install python-microscope
pip install microscope-cockpit
pip install opencv-python

2) Create directory to run the code from and ensure it has the
relevant config files, the findNuclei.py script and the large test
image.

create a directory cockpit-sim
Copy in the files
microscope_sim.py
cockpit_sim.depot
cockpit_sim.config
merged-zaber-rgb.jpg
channels-cockpit_sim
findNuclei.py

3) start the microscope server with simulated images, stage etc...

deviceserver microscope_sim.py

This might take some time as the very large jpg image is expanded
during this startup process to about 1.5 GB.

4) start cockpit connecting to the simulated instruments.

cockpit --depot-file cockpit_sim.depot --config-file cockpit_sim.config

The previous 2 steps might generate firewall notices as they utilise
the loopback network (localhost or 127.0.0.1) to connect between
Microscope-Cockpit and Python-Microscope. The Python process will need
to have access to this network for this setup to work. 

5) Ensure that the simulated stage etc are working, connect to the
simulated camera and activate a light source.

6) open the pyshell window in cockpit and load the find nuclei script

import findNuclei

Instantiate the class in the pyshell window with

f=findNuclei.findNuclei()

Ensure the system is setup to collect images in the "Blue, 460"
channel so you are imaging cell nuclei and start the online processing
with:

f.start()

Collect a  spiral mosaic by selecting the mosaic window and click
``Run mosaic''. The system should collect images and add marked
positions to nuclei in the images.

Stop the mosaic by clicking the ``Stop mosaic'' button and halt the
processing by switching to the pyshell window and calling

f.stop()

The marked positions can be used by either double clicking on them in
the marked point list in the mosaic window or use the multi site experiment
to define an experiment to be performed at a number of stage
positions. 


An example of what you might see is shown in the image
findNuclei-example.jpg
