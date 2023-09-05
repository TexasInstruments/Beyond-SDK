```
 ______                                          __         ______   ______   ___  ____   
|_   _ \                                        |  ]      .' ____ \ |_   _ `.|_  ||_  _|  
  | |_) |  .---.   _   __   .--.   _ .--.   .--.| | ______| (___ \_|  | | `. \ | |_/ /    
  |  __'. / /__\\ [ \ [  ]/ .'`\ \[ `.-. |/ /'`\' ||______|_.____`.   | |  | | |  __'.    
 _| |__) || \__.,  \ '/ / | \__. | | | | || \__/  |       | \____) | _| |_.' /_| |  \ \_  
|_______/  '.__.'[\_:  /   '.__.' [___||__]'.__.;__]       \______.'|______.'|____||____| 
                  \__.'                                                                   
```

*Welcome to `Beyond-SDK`... Your one-stop solution for different examples outside of the MCU+ SDK examples...*

## How to export an example into MCU+ SDK

It is noted that the examples can not work on their own. They need to be integrated into the specific MCU+ SDK to be able to import the example in CCS or build the example on the command line itself. Please follow the below instructions to integrate an example:

- Create a directory named `beyond-sdk` under `${MCU_PLUS_SDK_PATH}/examples`.
- Copy the example folder `${device}/examples/${example_name}` from this repo & paste it under the above created directory.
- Once copied, the integrated example should look like `${MCU_PLUS_SDK_PATH}/examples/beyond-sdk/${example_name}`

After the above steps, the integrated example is just like any other example in the SDK. So, the integrated example can be imported in CCS or built on the command line itself following the same process as followed for any other example.

