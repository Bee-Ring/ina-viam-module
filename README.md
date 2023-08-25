# INA power sensor Viam module
Module for INA3221 series power monitors

Tested and known to work on the INA3221. Support for other INA modules already in mainline Viam RDK.

This can act as a `sensor`, reporting all channels simultaneously, or as a `powersensor`, for which each channel must be configured individually.

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
      "model": "beering:sensor:ina",
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
      "type": "powersensor",
      "model": "beering:sensor:ina",
      "attributes": {
        "model": "ina3221",
        "channel": "channel1",
        "i2c_bus": "0"
      },
      "depends_on": []
    },
    {
      "name": "boardConsumption",
      "type": "powersensor",
      "model": "beering:sensor:ina",
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
