# Python-App
Python code for mavproxy

### SITL Command to run at the Dorothea Dix:


Run in ArduCopter dir on your loacl machine. ```cd ./ardupilot/ArduCopter/```
```
sim_vehicle.py -j4 -l 35.768701,-78.662681,96,180 --out 192.168.1.118:14550
```
Replace the ip address with your pi's ip address

### Fight Gear Command

Run in the autotest dir. ```./arudpiolt/Tools/autotest/```

```
./fg_quad_view.sh
```
### Mavproxy Command for connecting the the SITL

Run on the pi
```
mavproxy.py --sitl=192.168.1.132:14550
```
Replace ip with your loacl computer's ip address
### Mavproxy to connect to app and SITL

Run on the pi.

```
mavproxy.py --sitl=192.168.1.132:14550 --out=127.0.0.1:5760
```
Same as before. 

### Adding MP

Run on the loacl machine.
```
mavproxy.py --sitl=192.168.1.132:14550 --out=127.0.0.1:5760 --out=192.168.1.123:14550
```
Same as before

### Run Python script
```
python drone.py
```

