# snaak_pneumatic

This package is responsible for interfacing with the Pneumatic components and establishing serial communication with the Clear Core controller to Enable, Disable, and Eject the Vacuum based on the following ROS2 services.

## **snaak_pneumatic/enable_vacuum**  
Activates the vacuum system by sending an "enable" command via serial communication.  

**Input:**  
- None  

**Output:**  
- Success status and message ("Vacuum enabled")  

**CLI Usage Example:**  
```bash
ros2 service call /snaak_pneumatic/enable_vacuum std_srvs/srv/Trigger
```
**Example Output:**  
```bash
requester: making request: std_srvs.srv.Trigger_Request()

response:
  success: True
  message: "Vacuum enabled"
```

---

## **snaak_pneumatic/disable_vacuum**  
Deactivates the vacuum system by sending a "disable" command via serial communication.  

**Input:**  
- None  

**Output:**  
- Success status and message ("Vacuum disabled")  

**CLI Usage Example:**  
```bash
ros2 service call /snaak_pneumatic/disable_vacuum std_srvs/srv/Trigger
```
**Example Output:**  
```bash
requester: making request: std_srvs.srv.Trigger_Request()

response:
  success: True
  message: "Vacuum disabled"
```

---

## **snaak_pneumatic/eject_vacuum**  
Triggers the vacuum eject function for a specified duration. The duration is clamped between 100ms and 5000ms.  

**Input:**  
- `duration` (int): Duration of the eject function in milliseconds (100ms - 5000ms)  

**Output:**  
- Success status and message indicating the duration of vacuum ejection  

**CLI Usage Example:**  
```bash
ros2 service call /snaak_pneumatic/eject_vacuum example_interfaces/srv/SetBool "{data: 1000}"
```
*(This example sets the ejection duration to 1000ms.)*  

**Example Output:**  
```bash
requester: making request: example_interfaces.srv.SetBool_Request(data=1000)

response:
  success: True
  message: "Vacuum ejected for 1000 ms"
