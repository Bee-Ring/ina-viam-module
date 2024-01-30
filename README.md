# INA power sensor Viam module
Module for INA3221 series power monitors

Tested and known to work on the INA3221. Support for other INA modules already in mainline Viam RDK.

This can act as a `sensor`, reporting all channels simultaneously, or as a `power_sensor`, for which each channel must be configured individually.

## Usage

### 1. Build binary

If you clone this repository to the target environment where you run your Viam robot, then you can build a binary named `inaModule` with:

```
go build -o inaModule
```

### 2. Add to robot configuration

Copy the binary to the robot (system where viam-server is running) and add the following to your configuration to use as a sensor:

```
  "components": [
    {
      "name": "ina_for_example",
      "type": "sensor",
      "model": "bee-ring:sensor:ina",
      "attributes": {
        "model": "ina3221",
        "i2c_bus": "0"
      },
      "depends_on": []
    }
  ],
  "modules": [
    {
      "name": "inaModule_for_example",
      "executable_path": "/path/to/executable/inaModule"
    }
  ]
```

And/or, use the following to configure as a power sensor:

```
  "components": [
    {
      "name": "solarChannel",
      "type": "power_sensor",
      "model": "bee-ring:power_sensor:ina",
      "attributes": {
        "model": "ina3221",
        "channel": "channel1",
        "i2c_bus": "0"
      },
      "depends_on": []
    },
    {
      "name": "boardConsumption",
      "type": "power_sensor",
      "model": "bee-ring:power_sensor:ina",
      "attributes": {
        "model": "ina3221",
        "channel": "channel2",
        "i2c_bus": "0"
      },
      "depends_on": []
    }
  ],
  "modules": [
    {
      "name": "inaModule_for_example",
      "executable_path": "/path/to/executable/inaModule"
    }
  ]
```

Additionally, you may set a floating point number in the `voltageMul` attributes field, and the voltage returned by `Voltage()` will be multiplied by this number. This is useful in the case where you are monitoring a higher-voltage battery and are using a voltage divider to halve the voltage. Note that only the output of `Voltage()` is changed, the voltage value returned by `Readings()` will be the raw value.

Example, if you used a voltage divider to halve the voltage:
```
      "attributes": {
        "model": "ina3221",
        "channel": "channel2",
        "voltageMul": 2.0,
        "i2c_bus": "0"
      },
```
