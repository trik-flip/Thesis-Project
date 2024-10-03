# creating a custom message for an publisher and subscriber
## Creating a Message
1. If it doesn't already exist create a `msg` folder
2. and create a file called `<your custom message>.msg`
## Edit the CMake file
1. inside the 
```
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)
```

add the line `message_generation` like so
```
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)
```
2. uncomment the `generate_messages` lines
like so
```
generate_messages(
  DEPENDENCIES
  std_msgs
)
```
3. uncomment `add_message_files` like so
and edit it like so
```
add_message_files(
  FILES
  <your custom message>.msg
)
```
## Edit the package file
uncomment the `<build_depend>message_generation</build_depend>`
and `<exec_depend>message_runtime</exec_depend>` lines
## Rebuild the package
go to the top workspace and run `catkin_make`, makesure there are no errors
## Using the custom message
Now you should be able to import the messages with `from <your package>.msg import <your custom message>`
## An example Message
```
string name
int32 age
float32 x
float32 y
```
# Advanced messages
## Nested Messages
When a message is created you can use it inside another message like with `People` reffering to `Person`
## Arrays in messages
When creating a type like `string name`, you can append `[]` to the end of the type to create an array of it. like `int64[] sequence`, would be a sequence of numbers.