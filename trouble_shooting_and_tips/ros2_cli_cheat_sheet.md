# 🚀 ROS 2 CLI Cheat Sheet

## 🐢 Core Commands

### Create a new ROS 2 package
```bash
ros2 pkg create <package_name> --build-type ament_cmake
```

### List all available packages
```bash
ros2 pkg list
```

### Find package path
```bash
ros2 pkg prefix <package_name>
```

---

## 📡 Nodes

### Run a node
```bash
ros2 run <package_name> <executable_name>
```

### List running nodes
```bash
ros2 node list
```

### Get node info
```bash
ros2 node info <node_name>
```

---

## 📨 Topics

### List topics
```bash
ros2 topic list
```

### List topic types
```bash
ros2 topic list -t
```

### Echo topic messages
```bash
ros2 topic echo <topic_name>
```

### Get topic info
```bash
ros2 topic info <topic_name>
```

### Publish to a topic
```bash
ros2 topic pub <topic_name> <msg_type> '{data: value}'
```

---

## 📬 Services

### List services
```bash
ros2 service list
```

### List service type
```bash
ros2 service type /service_name
```

### Find services of a type
```bash
ros2 service find package_name/srv/ServiceName
```

### Call a service
```bash
ros2 service call <service_name> <srv_type> '{field: value}'
```

### Get service info
```bash
ros2 service info <service_name>
```

---

## 📦 Parameters

### List parameters
```bash
ros2 param list
```
### Dump parameters
```bash
ros2 param dump
```

### List parameters for a specific node
```bash
ros2 param list /node_name
```

### Get parameter value
```bash
ros2 param get <node_name> <param_name>
```

### Set parameter value
```bash
ros2 param set <node_name> <param_name> <value>
```

---

## 🧪 Actions

### List actions
```bash
ros2 action list
```

### Get action type
```bash
ros2 action type /action_name
```

### Send a goal
```bash
ros2 action send_goal <action_name> <action_type> '{field: value}'
```

### Get action info
```bash
ros2 action info <action_name>
```

---

## ⚙️ Lifecycle Nodes

### List lifecycle nodes
```bash
ros2 lifecycle list
```

### Get lifecycle node state
```bash
ros2 lifecycle get <node_name>
```

### Change lifecycle state
```bash
ros2 lifecycle set <node_name> <state>
# States: configure, activate, deactivate, cleanup, shutdown
```

---

## 🛠️ ROS 2 Control

### List available controllers
```bash
ros2 control list_controllers
```

### List available controller types
```bash
ros2 control list_controller_types
```

### List hardware interfaces
```bash
ros2 control list_hardware_interfaces
```

### Load a controller (but don’t start it yet)
```bash
ros2 control load_controller <controller_name>
```

### Configure a controller
```bash
ros2 control set_controller_state <controller_name> configure
```

### Start a controller
```bash
ros2 control set_controller_state <controller_name> start
```

### Stop a controller
```bash
ros2 control set_controller_state <controller_name> stop
```

### Unload a controller
```bash
ros2 control unload_controller <controller_name>
```

### Switch controllers (stop/start in one command)
```bash
ros2 control switch_controllers   --stop-controllers <controller1> <controller2>   --start-controllers <controller3>
```

### List loaded controller managers
```bash
ros2 control list_controller_managers
```
