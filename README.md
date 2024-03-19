#Micro-ROS Multiple Publisher, Subscriber, and Service Client Example

This example demonstrates how to create a single script containing code for multiple publisher, subscriber, and service client functionalities using the Micro-ROS platform. The code is supported by PlatformIO for compiling and debugging.

#Prerequisites
Before running this example, ensure you have the following:

Micro-ROS installed and configured on your platform.
PlatformIO installed and configured.
Access to a ROS 2 environment.

#Usage
Clone this repository to your local machine.
Open the provided platformio.ini file and make sure it is configured correctly for your platform and environment.
Open the provided main.cpp file in your preferred text editor or IDE.
Modify the code according to your specific requirements, such as changing message types, topics, service types, etc.
Compile the code using PlatformIO.
Upload the compiled firmware to your Micro-ROS supported platform.
Run the Micro-ROS client on your platform.
Monitor the output to observe the behavior of the publisher, subscriber, and service client.

#Code Overview
PublisherNode: Publishes a message of type std_msgs::msg::String to the topic "topic" every 1 second.
SubscriberNode: Subscribes to the same topic "topic" and displays received messages.
ServiceClientNode: Calls an empty service named "service" and logs whether the service call was successful.
Customization
Feel free to customize the code according to your specific requirements. You can modify message types, topics, service types, etc., to suit your application needs.


