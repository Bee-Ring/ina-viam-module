// package main is a module for INA series power monitors
package main

import (
	"context"

	"go.viam.com/rdk/logging"
	"github.com/bee-ring/ina-viam-module/inamodule"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/components/powersensor"
	"go.viam.com/rdk/module"
	"go.viam.com/utils"
)

func main() {
	utils.ContextualMain(mainWithArgs, logging.NewLogger("inaModule"))
}

func mainWithArgs(ctx context.Context, args []string, logger logging.Logger) error {
	inaSensorModule, err := module.NewModuleFromArgs(ctx, logger)
	if err != nil {
		return err
	}

	inaSensorModule.AddModelFromRegistry(ctx, sensor.API, inamodule.Model)
	inaSensorModule.AddModelFromRegistry(ctx, powersensor.API, inamodule.PowerModel)

	err = inaSensorModule.Start(ctx)
	defer inaSensorModule.Close(ctx)
	if err != nil {
		return err
	}

	<-ctx.Done()
	return nil
}
