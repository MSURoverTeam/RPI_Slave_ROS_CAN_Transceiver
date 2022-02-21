# RPI_Slave_ROS_CAN_Transceiver
A ROS package to translate CAN message to UInt16MultiArray ROS message. 
Array consists of 9 values in range 0-255
First byte is CAN ID of the target device, the rest are the message
CAN_Rx_Buffer is for received messages, CAN_Tx_Buffer is for messages to be set via CAN.
