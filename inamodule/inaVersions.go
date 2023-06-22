package inamodule

type RegName int

const (
	Configuration RegName = iota
	Channel1ShuntVoltage
	Channel1BusVoltage
	Channel2ShuntVoltage
	Channel2BusVoltage
	Channel3ShuntVoltage
	Channel3BusVoltage
	Channel1CriticalAlertLimit
	Channel1WarningAlertLimit
	Channel2CriticalAlertLimit
	Channel2WarningAlertLimit
	Channel3CriticalAlertLimit
	Channel3WarningAlertLimit
	ShuntVoltageSum
	ShuntVoltageSumLimit
	MaskEnable
	PowerValidUpperLimit
	PowerValidLowerLimit
	ManufacturerID
	DieID
	Channel1Power
	Channel1Current
	Calibration
)

type Register struct {
	Address byte
	Writable bool
}

var modelInitializer = map[string]func() map[RegName]Register {
	"ina219": initINA219,
	"ina226": initINA226,
	"ina3221": initINA3221,
}

func initINA3221() map[RegName]Register {
	return map[RegName]Register {
		Configuration: {0x00, true},
		Channel1ShuntVoltage: {0x01, false},
		Channel1BusVoltage: {0x02, false},
		Channel2ShuntVoltage: {0x03, false},
		Channel2BusVoltage: {0x04, false},
		Channel3ShuntVoltage: {0x05, false},
		Channel3BusVoltage: {0x06, false},
		Channel1CriticalAlertLimit: {0x07, true},
		Channel1WarningAlertLimit: {0x08, true},
		Channel2CriticalAlertLimit: {0x09, true},
		Channel2WarningAlertLimit: {0x0A, true},
		Channel3CriticalAlertLimit: {0x0B, true},
		Channel3WarningAlertLimit: {0x0C, true},
		ShuntVoltageSum: {0x0D, false},
		ShuntVoltageSumLimit: {0x0E, true},
		MaskEnable: {0x0F, true},
		PowerValidUpperLimit: {0x10, true},
		PowerValidLowerLimit: {0x11, true},
		ManufacturerID: {0xFE, false},
		DieID: {0xFF, false},
	}
}

func initINA226() map[RegName]Register {
	return map[RegName]Register {
		Configuration: {0x00, true},
		Channel1ShuntVoltage: {0x01, false},
		Channel1BusVoltage: {0x02, false},
		Channel1Power: {0x03, false},
		Channel1Current: {0x04, false},
		Calibration: {0x05, true},
		MaskEnable: {0x06, true},
		Channel1WarningAlertLimit: {0x07, true},
		ManufacturerID: {0xFE, false},
		DieID: {0xFF, false},
	}
}

func initINA219() map[RegName]Register {
	return map[RegName]Register {
		Configuration: {0x00, true},
		Channel1ShuntVoltage: {0x01, false},
		Channel1BusVoltage: {0x02, false},
		Channel1Power: {0x03, false},
		Channel1Current: {0x04, false},
		Calibration: {0x05, true},
	}
}
