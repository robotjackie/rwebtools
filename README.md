##Robot Web Tools
Jackie Wu and Athulya Simon

Jarvis Schultz, EECS 495 Embedded Systems in ROS, Fall 2014, Northwestern University


####What We Were Trying to Do
Our assignment was to allow an Internet browser to interact with ROS.


2. As an extension goal of the assignment, create a more complex user interface to control the Turtlesim
3. As an extension goal of the assignment, put the Turtlesim visualization online and control its movements with a node on the local computer
4. As a practical extension goal of the assignment, help Professor Lynch create a website to better visualize joint transformations for students in his Robotic Manipulation class

####What We Used
* `roslibjs` - Javascript library that has standard ROS functions for JS, such as publisher/subscriber, services, TF, URDF, etc.
* `rosbridge` - connects ROS to non-ROS programs, written with JSON
* WebSocket - protocol that connects `rosbridge` to a web browser or server, over TCP
* An html file with Javascript functions from the `roslibjs` library
* Weebly.com, a free site creator to host our site
* `turtlesim`, the simple ROS package that we wanted to put on our webpage

####The Result

####How We Did It
On a computer with a ROS active node, we used `rosbridge` to connect to WebSocket, a protocol that allows remote devices to communicate to a web browser, to connect to a website. In our website's HTML file, we import the necessary Javascript libraries such as `roslibjs` to write certain ROS functions for the HTML website.

We found code on [this site, Iguanatronics](http://iguanatronics.com/igtron/?p=313), that showed how to create an HTML webpage on our local computer to control the `Turtlesim` with, via the `turtle_teleop_key` node. This meant the first part of our assignment was very easy. 

However, we also tried to:
1. Put the HTML webpage online
2. Do the extension assignments


####Interesting Insights

####Difficulties We Had

####Mistakes We Found

####Useful Resources
* [`rosbridge page on ROS.org`](http://wiki.ros.org/rosbridge_suite)


