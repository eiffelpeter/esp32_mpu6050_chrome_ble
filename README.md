***

# Setup esp-idf on ubuntu
[link](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html)
  

# Clone code
`git clone https://github.com/eiffelpeter/esp32_mpu6050_chrome_ble.git`

# Connect esp32 and mpu6050
![IMAGE ALT TEXT HERE](./img/00_esp32_mpu6050.JPG)

# Build and program it
```
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

# Enable BLE on chrome
![IMAGE ALT TEXT HERE](./img/01_enable_chrome_flags.png)

# Execute html to connect esp32 BLE
`chrome/chrome_esp32_ble_accel.html`

# Demo on windows
[![IMAGE ALT TEXT HERE](./img/02_use_ble_on_chrome.png)](https://drive.google.com/file/d/1eNkAKaKVG8bJvVYZDnKEYRxeGJdmrHrm/view?usp=sharing)
