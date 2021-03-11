# ESP32-homekit-SHT20

This repository contains homekit, SHT20 driver and web server

## WIFI connect (using esptouch)

Esptouch app at app store: [link](https://apps.apple.com/tw/app/espressif-esptouch/id1071176700

## Resetful API
* system_infotmation
  - URI: http://IP/system_infotmation
  - Method: HTTP POST
  - Request: Empty
  - Response: {"Temperature":temperature_value, "Humidity":humidity}

* tempertature_humidity
  - URI: http://IP/tempertature_humidity
  - Method: HTTP POST
  - Request: Empty
  - Response: {"PRODUCT_NAME":PRODUCT_NAME, "MANUFACTURER": MANUFACTURER, "MODEL":MODEL,"SERIALNUMBER":SERIALNUMBER}

Please select EspTouch instead of EspTouchv2, when using app.

## Compile and install guide

[esp32-homekit](https://github.com/espressif/esp-apple-homekit-adk)

## Expain

[hackmd](https://hackmd.io/hv_N_z3KSpWupW4E0z5_Bg)
