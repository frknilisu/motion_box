# motion_box
Motion Box with timelapse, slow motion vs.

## How to run to RFCOMM Bluetooth without being root?  
https://stackoverflow.com/questions/34599703/rfcomm-bluetooth-permission-denied-error-raspberry-pi
1. Add --compat option to bluetoothd in bluetooth.service
    ```
    $ sudo nano /lib/systemd/system/bluetooth.service
    ExecStart=/usr/lib/bluetooth/bluetoothd --compat
    ```

2. change permission of sdp in bluetooth.service
    ```
    $ sudo nano /lib/systemd/system/bluetooth.service
    ExecStartPost=/bin/chmod 777 /var/run/sdp
    ```

3. make sure your pi user is in the bluetooth group:
    ```
    $ cat /etc/group | grep bluetooth
    bluetooth:x:113:pi
    ``` 
        
	3.1. If it's not, add pi to bluetooth group:
	`$ sudo usermod -aG bluetooth pi`

4. Change group of the /var/run/sdp file:
    ```
    $ sudo chgrp bluetooth /var/run/sdp
    ```

## How to connect to Bluetooth LE devices without being root?
https://unix.stackexchange.com/questions/348441/how-to-allow-non-root-systemd-service-to-use-dbus-for-ble-operation
Create a D-Bus security policy for the pi user.
```
sudo nano /etc/dbus-1/system.d/bluetooth.conf
  <policy user="pi">
    <allow own="org.bluez"/>
    <allow send_destination="org.bluez"/>
    <allow send_interface="org.bluez.Agent1"/>
    <allow send_interface="org.bluez.MediaEndpoint1"/>
    <allow send_interface="org.bluez.MediaPlayer1"/>
    <allow send_interface="org.bluez.Profile1"/>
    <allow send_interface="org.bluez.GattCharacteristic1"/>
    <allow send_interface="org.bluez.GattDescriptor1"/>
    <allow send_interface="org.bluez.LEAdvertisement1"/>
    <allow send_interface="org.freedesktop.DBus.ObjectManager"/>
    <allow send_interface="org.freedesktop.DBus.Properties"/>
  </policy>
```
```
sudo systemctl daemon-reload
sudo systemctl restart bluetooth
sudo systemctl restart dbus
```

