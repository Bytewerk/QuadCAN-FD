## these are instructions for hardware version 1!

### install dependencies
```
apt-get install raspberrypi-kernel-headers
```

### build module
```
make -C /lib/modules/$(uname -r)/build M=$PWD
```

### install module
```
sudo make -C /lib/modules/$(uname -r)/build M=$PWD modules_install
sudo depmod -a
```

# compile and install device tree overlay
```
sudo dtc -@ -I dts -O dtb -o /boot/overlays/4xMCP2517FD.dtbo 4xMCP2517FD.dts
```

# activate device tree overlay
```
sudo sh -c 'echo "dtoverlay=4xMCP2517FD" >> /boot/config.txt'
```
