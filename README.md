# Overview

sysmon_ros is for monitoring a system status on ROS environment.

# Message

## netif

```
string if_name			//Interface name
string[] ip			//IP address list
int32 tx_bps			//Transmit data rate in bit per second
int32 rx_bps			//Recieve data rate in bit per second
float32 tx_error_rate		//Transmit error rate in percent
float32 rx_error_rate		//Revieve error rate in percent
```

# Nodes

## cpumon

CPU monitor node.
This node measure a CPU usage.
The value is read from ```/proc/stat```.

### Subscribed Topics

- None

### Published Topics

- ```cpumon/CPUNAME``` (std_msgs/Float32)  
CPUNAME depends on your system.

### Parameters

- ~hz (float32)  
reflesh rate in Hz

## memmon

Memory monitor node.
This node measure a memory (include swap) usage.
The value is read from ```/proc/meminfo```.

### Subscribed Topics

- None

### Published Topics

- ```memmon/rate_real``` (std_msgs/Float32)  
Real (physical) memory usage in percent.

- ```memmon/rate_swap``` (std_msgs/Float32)  
Swap usage in percent.

- ```memmon/rate_total``` (std_msgs/Float32)  
Total (real + swap) memory usage in percent.

### Parameters

- ~hz (float32)  
reflesh rate in Hz

## netmon

Network monitor node.
This node measure a network usage.
The value is read from ```sys/socket.h```.

### Subscribed Topics

- None

### Published Topics

- ```netmon/NICNAME``` (sysmon_ros/netif)  
NICNAME depends on your system.

### Parameters

- ~hz (float32)  
reflesh rate in Hz

## tempmon

Temperature monitor node.
This node measure a temperature on the system.
The value is read from ```/sys/class/hwmon/```.

### Subscribed Topics

- None

### Published Topics

- ```tempmon/SENSORNAME``` (std_msgs/Float32)  
SENSORNAME depends on your system.
If there have some sensors on the system, the node provide all of them with specified label.

### Parameters

- ~hz (float32)  
reflesh rate in Hz


