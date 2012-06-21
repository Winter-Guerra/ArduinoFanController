In early 2012, I decided that my home-built Ubuntu server needed more hard drive space. Since there was no more hard drive case slots open in my server, 
I decided to cram three hard drives into the two open 5.25" CD bays using a 3D printed adapter. This solved my space issues, but only left about a few millimeters of breathing room
between the two hard drives.
I was pleased with the setup for a while, but then one day I opened the case to troubleshoot a problem (obviously with the power off). My hand brushed the HDD bank by accident.
YOW! That's too hot to touch! 
It turns out that the HDDs, given no space to dissipate heat, would routinely heat up to an excess of 46C in less than hour after bootup. 
That's more than 6C above the recommended maximum operating temperature! What to do, what to do.... I know! Let's cram a Arduino controlled fan into the HDD bank to cool the HDDs!

Here resides the code needed to run the Arduino controlled HDD cooling fan:
The code is split into two folders,
-FanControllerArduino holds the latest code and todo list for the Arduino side of the fan controller
-FanControllerHostPy holds the latest code and todo list for the server side of the fan controller (coded in Python) 
