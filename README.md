##Robot Web Tools
Jackie Wu and Athulya Simon

Jarvis Schultz, EECS 495 Embedded Systems in ROS, Fall 2014, Northwestern University



####Project Goals
Our assignment was to allow an Internet browser to interact with ROS.

Specifically, we wanted to control Turtlesim on a local computer, through controls on a webpage. We wanted to create:

* arrow keys analogous to the function of `turtle_teleop_key` that would control left/right turn and forward/backward motion of the turtle
* input boxes that controlled the motion of the turtle (through its linear and angular velocity components)
* the ability to change `turtlesim`'s background color (with sliders that controlled the RGB values of the background parameter)

To do this, we needed to figure out how to publish and subscribe topics, get and set parameters, and call services from ROS on our computer to the web. 

* As an extension goal of the assignment, put the Turtlesim visualization online and control its movements with a node on another computer
* As a practical extension goal of the assignment, help Professor Lynch create a website to better visualize joint transformations for students in his Robotic Manipulation class

####ROS Package Dependencies and Other Things Needed
* `roslibjs` - Javascript library that has standard ROS functions for JS, such as publisher/subscriber, services, TF, URDF, etc.
* `rosbridge` - connects ROS to non-ROS programs, written with JSON
* WebSocket - protocol that connects `rosbridge` to a web browser or server, over TCP
* An html file with Javascript functions from the `roslibjs` library
* Weebly.com, a free site creator to host our site
* `turtlesim`, the simple ROS package that we wanted to put on our webpage

####Result

####How We Did It
**First**, we wanted to get `turtlesim` to run on a web browser hosted locally.

We found code on this site, [Iguanatronics](http://iguanatronics.com/igtron/?p=313), that showed how to create an HTML webpage, stored on our local computer, to control `turtlesim` via submitting commands to the `turtle_teleop_key` node. This meant the first part of our assignment was very easy. 

On a computer with an active `turtlesim` node, we used the ROS package `rosbridge` to connect to WebSocket, which is a protocol that allows remote devices to communicate to a web browser. This in turn connected to a website where we wrote a user interface to control `turtle_teleop_key`. 

After running `roscore` and opening the `turtlesim` node with
    
    rosrun turtlesim turtlesim_node
    
we installed `rosbridge`:

    sudo apt-get install ros-hydro-rosbridge-suite
    
Then we launched it with

    roslaunch rosbridge_server rosbridge_websocket.launch
  
Next, we created an HTML file to create a website on our local machine. In our website's HTML file, we import the necessary Javascript libraries, which are:

* `roslibjs` to write certain ROS functions for the HTML website

* EventEmitter2, something that allows us to emit "events" back to ROS, written in Javascript with Node.js, an open-source app building environment.

~~~html
    <!DOCTYPE html>
    <html>
    <head>
    <script type="text/javascript" src="http://cdn.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js"></script>
    <script type="text/javascript" src="http://cdn.robotwebtools.org/roslibjs/current/roslib.min.js"></script>
    <script type="text/javascript" type="text/javascript">
~~~


Next, there is some documented code on connecting, disconnecting, or error in connecting to `rosbridge`.

~~~html    
	// This function connects to the rosbridge server running on the local computer on port 9090
	var rbServer = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
	});

    // This function is called upon the rosbridge connection event
    rbServer.on('connection', function() {
    // Write appropriate message to #feedback div when successfully connected to rosbridge
    var fbDiv = document.getElementById('feedback');
    fbDiv.innerHTML += "<p>Connected to websocket server.</p>";
    });

    // This function is called when there is an error attempting to connect to rosbridge
    rbServer.on('error', function(error) {
    // Write appropriate message to #feedback div upon error when attempting to connect to rosbridge
    var fbDiv = document.getElementById('feedback');
    fbDiv.innerHTML += "<p>Error connecting to websocket server.</p>";
    });

    // This function is called when the connection to rosbridge is closed
    rbServer.on('close', function() {
    // Write appropriate message to #feedback div upon closing connection to rosbridge
    var fbDiv = document.getElementById('feedback');
    fbDiv.innerHTML += "<p>Connection to websocket server closed.</p>";
    });
~~~  


Now we write the functions to create a topic and write messages to ROS.

~~~HTML
// These lines create a topic object as defined by roslibjs
  var cmdVelTopic = new ROSLIB.Topic({
    ros : rbServer,
    name : '/turtle1/cmd_vel',
    messageType : 'geometry_msgs/Twist'
  });

  // These lines create a message that conforms to the structure of the Twist defined in our ROS installation
  // It initalizes all properties to zero. They will be set to appropriate values before we publish this message.
  var twist = new ROSLIB.Message({
    linear : {
      x : 0.0,
      y : 0.0,
      z : 0.0
    },
    angular : {
      x : 0.0,
      y : 0.0,
      z : 0.0
    }
  });
~~~


Now we write the function to take the numeric value for the Twist objects and publish them to the cmd_velocity topic in ROS, controlling the `turtlesim`. This ends the script.

~~~HTML
 /* This function:
	- retrieves numeric values from the text boxes
	- assigns these values to the appropriate values in the twist message
	- publishes the message to the cmd_vel topic.
  */
  function pubMessage() {

    /**
    Set the appropriate values on the twist message object according to values in text boxes
    It seems that turtlesim only uses the x property of the linear object 
    and the z property of the angular object
    **/
    var linearX = 0.0;
    var angularZ = 0.0;

    // get values from text input fields. Note for simplicity we are not validating.
    linearX = 0 + Number(document.getElementById('linearXText').value);
    angularZ = 0 + Number(document.getElementById('angularZText').value);

    // Set the appropriate values on the message object
    twist.linear.x = linearX;
    twist.angular.z = angularZ;

    // Publish the message 
    cmdVelTopic.publish(twist);
  }

</script>
</head>
~~~

Lastly, we create the control panel to write values for the `turtlesim` via our webpage that publishes its messages to our script, before we end the HTML.

~~~HTML
<body>
<form name="ctrlPanel">
<p>Enter positive or negative numeric decimal values in the boxes below</p>
<table>
    <tr><td>Linear X</td><td><input id="linearXText" name="linearXText" type="text" value="1.5"/></td></tr>
    <tr><td>Angular Z</td><td><input id="angularZText" name="angularZText" type="text" value="1.5"/></td></tr>
</table>
<button id="sendMsg" type="button" onclick="pubMessage()">Publish Message</button>
</form>
<div id="feedback"></div>
</body>
</html>
~~~

To execute this, we created a .launch file which would automatically start `roscore`, WebSocket, `turtlesim_node`, and `turtle_teleop_key`. A user would need to run this launch file after opening the HTML webpage (or refresh the HTML page, if that was opened _before_ running the .launch file).

EXTRA: We also added arrow key graphics to the HTML to simulate the `turtle_teleop_key` function from ROS. By clicking left or right, the turtle would turn ccw or cw (by manipulating the angular velocity of the Twist message); forward and backward led the turtle to move forward or backward (with the spatial velocity of Twist). The code is below:

~~~HTML
	//moving with arrow keys
	
	function moveup() { 
	twist.linear.x = 1.0;
	twist.angular.z = 0.0;
	cmdVelTopic.publish(twist); 
	};
	
	function movedown() { 
	var linearX = -1.0;
	var angularZ = 0.0;
	twist.linear.x = linearX;
	twist.angular.z = angularZ;
	cmdVelTopic.publish(twist); 
	};
	
	function turnleft() { 
	var linearX = 0.0;
	var angularZ = 1.570796;
	twist.linear.x = linearX;
	twist.angular.z = angularZ;
	cmdVelTopic.publish(twist); 
	};
	
	function turnright() { 
	var linearX = 0.0;
	var angularZ = -1.570796;
	twist.linear.x = linearX;
	twist.angular.z = angularZ;
	cmdVelTopic.publish(twist); 
	};
~~~



The actual keys that use this functionality are coded by:

~~~html
	<style>
	 button.pos_left {
	   position: absolute;
	   left: 20px;
	   top: 350px;
	 }
	 button.pos_right{
	   position: absolute;
	   left: 75px;
	   top: 350px;
	 }
	 button.pos_up{
	   position: absolute;
	   left: 50px;
	   top: 325px;
	 }
	 button.pos_down{
	   position: absolute;
	   left: 50px;
	   top: 350px;
	 }
	
	</style>
~~~

These can be placed anywhere. We placed them below the rest of the control box in our webpage.

**Second**, we want to control the 'turtlesim' on a local computer online. We put the HTML code on a free site builder, [http://robotwebtools.weebly.com/turtlesim.html], which allowed us to control our running `turtlesim_node` (with WebSocket, `roscore` running). 

**Third**, we tried running `turtlesim` over each other's computer via Ethernet. But we ran into some problems and this did not work (see **Problems We Faced** section).



####Insights

####Difficulties We Faced

Our third step was trying to run `turtlesim` over each other's computer via Ethernet. However, this didn't work, and we ran into the following problems:

* Jackie's computer stopped being able to open `turtlesim_node` for some reason, giving the error 
> Couldn't find executable named turtlesim_node below /opt/ros/indigo/share/turtlesim

* Another classmate's laptop didn't work, giving an error that something was not defined in EventEmitter, which was the package that emitted "events" from the HTML back through WebSocket and `rosbridge`. This is probably regarding Ethernet connection.

Another problem we had was when we tried to add functionality of controlling the turtle's direction with arrow keys in the HTML webpage. However, we were not able to figure out the Javascript syntax to allow this to happen. 

####Mistakes We Found

####Useful Resources
* [`rosbridge`](http://wiki.ros.org/rosbridge_suite) page on ROS.org

