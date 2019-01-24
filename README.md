# Pioneer_run

## initial settings 

1. connect NUC with Pioneer via USB 
2. sudo chmod 666 /dev/ttyUSBX

## ros settings (in your project launch file) 
```
  <node pkg="p2os_driver" name="p2os" type="p2os_driver" args="_port:=/dev    /ttyUSB1" output="screen"/>
```
**safety switch**
```
rostopic pub /cmd_motor_state p2os_msgs/MotorState "state ...."
```
