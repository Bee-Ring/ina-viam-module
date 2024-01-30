// Package inamodule implements ina voltage/current/power monitor sensors -
// typically used for battery state monitoring.

// code and configuration loosely based on the implementation in esphome
// https://github.com/esphome/esphome/blob/dev/esphome/components/ina3221/ina3221.cpp
package inamodule

import (
	"context"
	"encoding/hex"
	"encoding/binary"
	"errors"
	"fmt"
	"time"

	"go.viam.com/utils"

	"go.viam.com/rdk/components/board/genericlinux/buses"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/components/powersensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
)

var Model = resource.ModelNamespace("bee-ring").WithFamily("sensor").WithModel("ina")
var PowerModel = resource.ModelNamespace("bee-ring").WithFamily("power_sensor").WithModel("ina")

const (
	milliAmp                   = 1000 * 1000 // milliAmp = 1000 microAmpere * 1000 nanoAmpere
	milliOhm                   = 1000 * 1000 // milliOhm = 1000 microOhm * 1000 nanoOhm
	defaultI2Caddr             = 0x40
	senseResistor      float64 = 0.1  // .1 ohm
	maxCurrent           int64 = 3200 * milliAmp // 3.2 amp
	calibratescale             = ((int64(1000*milliAmp) * int64(1000*milliOhm)) / 100000) << 12
)

// Config is used for converting config attributes.
type Config struct {
	Model   string `json:"model"`
	I2CBus  string `json:"i2c_bus"`
	I2cAddr int    `json:"i2c_addr,omitempty"`
	Channel string    `json:"channel,omitempty"`
	VoltageMul float64    `json:"voltageMul,omitempty"`
}

// Validate ensures all parts of the config are valid.
func (config *Config) Validate(path string) ([]string, error) {
	var deps []string
	if len(config.I2CBus) == 0 {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "i2c_bus")
	}
	if config.Model == "" {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "model")
	}
	if config.Channel != "" {
		if config.Channel != "channel1" && config.Channel != "channel2" && config.Channel != "channel3" {
			return nil, fmt.Errorf("specified channel was %s. Valid options are channel1, channel2, or channel3", config.Channel)
		}
	}
	return deps, nil
}

func init() {
	resource.RegisterComponent(
		sensor.API,
		Model,
		resource.Registration[sensor.Sensor, *Config]{
			Constructor: func(
				ctx context.Context,
				deps resource.Dependencies,
				conf resource.Config,
				logger logging.Logger,
			) (sensor.Sensor, error) {
				newConf, err := resource.NativeConfig[*Config](conf)
				if err != nil {
					return nil, err
				}
				return newSensor(ctx, deps, conf.ResourceName(), newConf, logger)
		},
	})
	resource.RegisterComponent(
		powersensor.API,
		PowerModel,
		resource.Registration[powersensor.PowerSensor, *Config]{
			Constructor: func(
				ctx context.Context,
				deps resource.Dependencies,
				conf resource.Config,
				logger logging.Logger,
			) (powersensor.PowerSensor, error) {
				newConf, err := resource.NativeConfig[*Config](conf)
				if err != nil {
					return nil, err
				}
				return newSensor(ctx, deps, conf.ResourceName(), newConf, logger)
		},
	})
}

func newSensor(
	ctx context.Context,
	deps resource.Dependencies,
	name resource.Name,
	attr *Config,
	logger logging.Logger,
) (*inaSensor, error) {
	regFunc, ok := modelInitializer[attr.Model]
	if !ok {
		avail := ""
		for k, _ := range modelInitializer {
			avail = avail + ", " + k
		}
		return nil, fmt.Errorf("%s is not a supported INA model, supported models are: %s", attr.Model, avail)
	}
	reg := regFunc()
	
	i2cbus, err := buses.NewI2cBus(attr.I2CBus)
	if err != nil {
		return nil, err
	}
	addr := attr.I2cAddr
	if addr == 0 {
		addr = defaultI2Caddr
		logger.Warnf("using i2c address : 0x%s", hex.EncodeToString([]byte{byte(addr)}))
	}
	
	s := &inaSensor{
		Named:    name.AsNamed(),
		logger: logger,
		bus:    i2cbus,
		addr:   byte(addr),
		reg: reg,
		boardModel: attr.Model,
		channel: attr.Channel,
	}
	
	switch s.boardModel {
		case "ina3221":
			err := s.configure3221(ctx)
			if err != nil {
				return nil, err
			}
		default:
			return nil, fmt.Errorf("%s is not a supported INA model", s.boardModel)
	}

	return s, nil
}

// inaSensor is a i2c sensor device that reports voltage, current and power across N channels that should support multiple INA chip models
type inaSensor struct {
	resource.Named
	resource.AlwaysRebuild
	resource.TriviallyCloseable
	logger     logging.Logger
	bus        buses.I2C
	addr       byte
	reg       map[RegName]Register
	currentLSB int64
	powerLSB   int64
	cal        uint16
	boardModel string
	channel    string
}

type powerMonitor struct {
	Shunt   int64
	Voltage float64
	Current float64
	Power   float64
}

func (ina *inaSensor) configure3221(ctx context.Context) error {
	handle, err := ina.bus.OpenHandle(ina.addr)
	if err != nil {
		ina.logger.Errorf("can't open inaSensor i2c: %s", err)
		return err
	}
	defer utils.UncheckedErrorFunc(handle.Close)
	buf := make([]byte, 2)
	binary.BigEndian.PutUint16(buf, uint16(0x8000))
	err = handle.WriteBlockData(ctx, ina.reg[Configuration].Address, buf)
	if err != nil {
		return err
	}
	
	utils.SelectContextOrWait(ctx, 1. * time.Millisecond)
	
	// configuration bitmask
	config := uint16(0b0000000000000000)
	
	// 0b0xxx000000000000 << 12 Channel Enables (1 -> ON)
	config |= 0b0111000000000000
	// 0b0000xxx000000000 << 9 Averaging Mode (0 -> 1 sample, 111 -> 1024 samples)
	config |= 0b0000000000000000
	// 0b0000000xxx000000 << 6 Bus Voltage Conversion time (100 -> 1.1ms, 111 -> 8.244 ms)
	config |= 0b0000000111000000
	// 0b0000000000xxx000 << 3 Shunt Voltage Conversion time (same as above)
	config |= 0b0000000000111000
	// 0b0000000000000xxx << 0 Operating mode (111 -> Shunt and bus, continuous)
	config |= 0b0000000000000111
	buf = make([]byte, 2)
	binary.BigEndian.PutUint16(buf, config)
	return handle.WriteBlockData(ctx, ina.reg[Configuration].Address, buf)
}

// Readings returns a list containing three items (voltage, current, and power).
func (ina *inaSensor) Readings(ctx context.Context, extra map[string]interface{}) (map[string]interface{}, error) {
	
	handle, err := ina.bus.OpenHandle(ina.addr)
	if err != nil {
		ina.logger.Errorf("can't open inaSensor i2c: %s", err)
		return nil, err
	}
	defer utils.UncheckedErrorFunc(handle.Close)

	if calReg, ok := ina.reg[Calibration]; ok {
		// use the calibration result to set the scaling factor
		// of the current and power registers for the maximum resolution
		buf := make([]byte, 2)
		binary.BigEndian.PutUint16(buf, ina.cal)
		err = handle.WriteBlockData(ctx, calReg.Address, buf)
		if err != nil {
			return nil, err
		}
	}

	var pm powerMonitor

	bus, err := handle.ReadBlockData(ctx, ina.reg[Channel1BusVoltage].Address, 2)
	if err != nil {
		return nil, err
	}
	// Check if bit zero is set, if set the ADC has overflowed.
	if binary.BigEndian.Uint16(bus)&1 > 0 {
		return nil, errors.New("inaSensor bus 1 voltage register overflow")
	}
	pm.Voltage = float64(binary.BigEndian.Uint16(bus)) / 1000.

	if currentRegister, ok := ina.reg[Channel1Current]; ok {
		// the chip might have both current + power supported for ch1, or neither.
		current, err := handle.ReadBlockData(ctx, currentRegister.Address, 2)
		if err != nil {
			return nil, err
		}

		pm.Current = float64(int64(binary.BigEndian.Uint16(current))*ina.currentLSB) / 1000000000

		power, err := handle.ReadBlockData(ctx, ina.reg[Channel1Power].Address, 2)
		if err != nil {
			return nil, err
		}
		pm.Power = float64(int64(binary.BigEndian.Uint16(power))*ina.powerLSB) / 1000000000
	} else {
		// Without current+ power registries, we must pull the shunt voltage and do the math ourselves
		
		shunt, err := handle.ReadBlockData(ctx, ina.reg[Channel1ShuntVoltage].Address, 2)
		if err != nil {
			return nil, err
		}
		ina.logger.Infof("inaSensor shunt raw : %d", shunt)
		shuntV := float64(binary.BigEndian.Uint16(shunt)) * 40. / 8. / 1000000.
		ina.logger.Infof("inaSensor shunt uint: %d float: %f", binary.BigEndian.Uint16(shunt), shuntV)
		if binary.BigEndian.Uint16(shunt)&1 > 0 {
			return nil, errors.New("inaSensor shunt 1 voltage register overflow")
		}
		pm.Current, pm.Power = currentAndPowerFromVoltages(shunt, pm.Voltage)
	}
	
	// Just one channel being measured. We return now.
	if _, ok := ina.reg[Channel2BusVoltage]; !ok {
		return map[string]interface{}{
			"volts": pm.Voltage,
			"amps":  pm.Current,
			"watts": pm.Power,
		}, nil
	}
	
	// Below here is INA3221 only
	
	var pm2 powerMonitor
	var pm3 powerMonitor
	
	bus, err = handle.ReadBlockData(ctx, ina.reg[Channel2BusVoltage].Address, 2)
	if err != nil {
		return nil, err
	}
	// Check if bit zero is set, if set the ADC has overflowed.
	if binary.BigEndian.Uint16(bus)&1 > 0 {
		return nil, errors.New("inaSensor bus 2 voltage register overflow")
	}
	pm2.Voltage = float64(binary.BigEndian.Uint16(bus)) / 1000.
	shunt, err := handle.ReadBlockData(ctx, ina.reg[Channel2ShuntVoltage].Address, 2)
	if err != nil {
		return nil, err
	}
	if binary.BigEndian.Uint16(shunt)&1 > 0 {
		return nil, errors.New("inaSensor shunt 1 voltage register overflow")
	}
	pm2.Current, pm2.Power = currentAndPowerFromVoltages(shunt, pm2.Voltage)
	
	bus, err = handle.ReadBlockData(ctx, ina.reg[Channel3BusVoltage].Address, 2)
	if err != nil {
		return nil, err
	}
	// Check if bit zero is set, if set the ADC has overflowed.
	if binary.BigEndian.Uint16(bus)&1 > 0 {
		return nil, errors.New("inaSensor bus 3 voltage register overflow")
	}
	pm3.Voltage = float64(binary.BigEndian.Uint16(bus)) / 1000.
	shunt, err = handle.ReadBlockData(ctx, ina.reg[Channel3ShuntVoltage].Address, 2)
	if err != nil {
		return nil, err
	}
	if binary.BigEndian.Uint16(shunt)&1 > 0 {
		return nil, errors.New("inaSensor shunt 1 voltage register overflow")
	}
	pm3.Current, pm3.Power = currentAndPowerFromVoltages(shunt, pm3.Voltage)
	return map[string]interface{}{
		"channel1": map[string]interface{}{
			"volts": pm.Voltage,
			"amps":  pm.Current,
			"watts": pm.Power,
		},
		"channel2": map[string]interface{}{
			"volts": pm2.Voltage,
			"amps":  pm2.Current,
			"watts": pm2.Power,
		},
		"channel3": map[string]interface{}{
			"volts": pm3.Voltage,
			"amps":  pm3.Current,
			"watts": pm3.Power,
		},
	}, nil
}

func (ina *inaSensor) Voltage(ctx context.Context, extra map[string]interface{}) (float64, bool, error) {
	if ina.channel == "" {
		return 0, false, errors.New("no channel specified")
	}
	readings, err := ina.Readings(ctx, extra)
	if err != nil {
		return 0, false, err
	}
	
	vals := readings[ina.channel].(map[string]interface{})
	volts := vals["volts"].(float64)
	
	return volts, false, nil
}

func (ina *inaSensor) Current(ctx context.Context, extra map[string]interface{}) (float64, bool, error) {
	if ina.channel == "" {
		return 0, false, errors.New("no channel specified")
	}
	readings, err := ina.Readings(ctx, extra)
	if err != nil {
		return 0, false, err
	}
	
	vals := readings[ina.channel].(map[string]interface{})
	amps := vals["amps"].(float64)
	
	return amps, false, nil
}

func (ina *inaSensor) Power(ctx context.Context, extra map[string]interface{}) (float64, error) {
	if ina.channel == "" {
		return 0, errors.New("no channel specified")
	}
	readings, err := ina.Readings(ctx, extra)
	if err != nil {
		return 0, err
	}
	
	vals := readings[ina.channel].(map[string]interface{})
	watts := vals["watts"].(float64)
	
	return watts, nil
}

func currentAndPowerFromVoltages(shuntRaw []byte, busV float64) (float64, float64) {
	shuntV := float64(int16(binary.BigEndian.Uint16(shuntRaw))) * 40. / 8. / 1000000.
	current := float64(shuntV) / senseResistor // TODO: adjustable sense resistor values
	power := current * busV
	return current, power
}
