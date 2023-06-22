# INA power sensor Viam module
Module for INA series power monitors

Tested and known to work on at least the INA219 and the INA3221. Tentative support for the INA226.

## Usage

### 1. Build binary

If you clone this repository to the target environment where you run your Viam robot, then you can build a binary named `inaModule` with:

```
go build -o inaModule
```

### 2. Add to robot configuration

Copy the binary to the robot (system where viam-server is running) and add the following to your configuration:

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
