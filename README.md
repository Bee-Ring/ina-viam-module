# INA power sensor Viam module
Module for INA series power monitors

Known to work on at least the INA219 and the INA3221

## Usage

### 1. Build binary

If you clone this repository to the target environment where you run your Viam robot, then you can build a binary named `inaModule` with:

```
go build -o inaModule
```

Alternatively, if you want to build a binary for a different target environment, please use the [viam canon tool](https://github.com/viamrobotics/canon).

### 2. Add to robot configuration

Copy the binary to the robot (system where viam-server is running) and add the following to your configuration:

```
  "components": [
    {
      "name": "ina_for_example",
      "type": "sensor",
      "model": "nau7802",
      "attributes": {
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

For more information on how to configure modular components, [see this example](https://docs.viam.com/services/slam/run-slam-cartographer/#step-1-add-your-rdiplar-as-a-modular-component).
