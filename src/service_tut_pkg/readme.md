
## Creating a Service
1. If it doesn't already exist create a `srv` folder
2. and create a file called `<your custom service>.srv`
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
3. uncomment `add_service_files` like so
and edit it like so
```
add_service_files(
  FILES
  <your custom service>.srv
)
```
## Edit the package file
uncomment the `<build_depend>message_generation</build_depend>`
and `<exec_depend>message_runtime</exec_depend>` lines
## Rebuild the package
go to the top workspace and run `catkin_make`, makesure there are no errors
## Using the custom message
Now you should be able to import the messages with `from <your package>.srv import <your custom service>, <your custom service>Response`
## An example Service
```
float32 x
float32 y
---
string name
int32 age
```
