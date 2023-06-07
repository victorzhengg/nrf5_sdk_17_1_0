# nrf5_sdk_17_1_0

This code show how to synchronize the timer between multi nodes. 
1. The BLE service is a customized service named multi node synchronize service.
2. each node have total three link. It include a central connection and two peripheral connection
3. The BLE service is a customized service named multi node synchronize service.
4. button 1 enable the synchronize
5. button 2 disable the synchronize
6. button 4 show the information of node which connect to current node
7. LED1 turn on if current node connect other node as a gatt client, GAP role is central
8. LED2 turn on if current node is connected by other node. current node work as a gatt server, GAP role is peripheral. 
9. LED3 turn on if current node is connected by other node. current node work as a gatt server, GAP role is peripheral. 
10. LED4 blinky according the period define by macro MNS_CONTROL_LED_PERIOD
