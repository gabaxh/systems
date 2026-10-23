/*******************************************************************************
 * Copyright (c) 2026 Synecdoque
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, subject to the following conditions:
 *
 * The software is licensed under the MIT License. See the LICENSE file in this
 * repository for details.
 *
 * Contributors:
 *   Gabriel Axheim Gustafsson, Luleå - initial implementation
 ***************************************************************************SDG*/

// The lidar system is the Livox MID-360 as an Arrowhead system. Today it serves
// the gyro inside the device, as the angle the vehicle has turned about the
// vertical. The point cloud is to become a second asset beside it, sharing the
// device code in livox.go and the clock in clock.go, because it comes from the
// same device, over the same link and on the same clock, and has to be
// de-skewed with this same gyro.
//
// Like the guetteur, it is the sensor and nothing more. It knows the Livox
// protocol, the device's clock and which way is up for the mount it was
// calibrated on; it knows nothing about wheels, tracks or maps. Fusing the yaw
// with the wheels belongs to the cartographer (see
// cartographer/gyrodometry_proposal_2026-10-20.md).
//
// "lidar" is a working name. It lives in systemName and the directory, so a
// rename is mechanical.
package main

import (
	"context"
	"crypto/x509/pkix"
	"encoding/json"
	"log"
	"net/http"
	"time"

	"github.com/sdoque/mbaigo/components"
	"github.com/sdoque/mbaigo/usecases"
)

const systemName = "lidar"

func main() {
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	sys := components.NewSystem(systemName, ctx)
	usecases.WatchShutdown(&sys, cancel)

	sys.Husk = &components.Husk{
		Description: "publishes the yaw of a Livox MID-360's built-in gyro",
		Details:     map[string][]string{"Developer": {"Synecdoque"}},
		Host:        components.NewDevice(),
		ProtoPort:   map[string]int{"https": 30202, "http": 20202, "coap": 0},
		InfoLink:    "https://github.com/sdoque/systems/tree/main/" + systemName,
		DName: pkix.Name{
			CommonName:         sys.Name,
			Organization:       []string{"Synecdoque"},
			OrganizationalUnit: []string{"Systems"},
			Locality:           []string{"Luleå"},
			Province:           []string{"Norrbotten"},
			Country:            []string{"SE"},
		},
		RegistrarChan: make(chan *components.CoreSystem, 1),
		Messengers:    make(map[string]int),
	}

	assetTemplate := initTemplate()
	sys.UAssets[assetTemplate.GetName()] = assetTemplate

	rawResources, err := usecases.Configure(&sys)
	if err != nil {
		log.Fatalf("configuration error: %v\n", err)
	}
	sys.UAssets = make(map[string]*components.UnitAsset)
	for _, raw := range rawResources {
		var uac usecases.ConfigurableAsset
		if err := json.Unmarshal(raw, &uac); err != nil {
			log.Fatalf("resource configuration error: %+v\n", err)
		}
		ua, cleanup := newResource(uac, &sys)
		defer cleanup()
		sys.UAssets[ua.GetName()] = ua
	}

	usecases.RequestCertificate(&sys)
	usecases.RegisterServices(&sys)
	go usecases.SetoutServers(&sys)

	<-sys.Ctx.Done()
	log.Println("shutting down system", sys.Name)
	time.Sleep(2 * time.Second)
}

func serving(t *Traits, w http.ResponseWriter, r *http.Request, servicePath string) {
	switch servicePath {
	case "yaw":
		t.yawService(w, r)
	case "yawRate":
		t.yawRateService(w, r)
	case "bias":
		t.biasService(w, r)
	default:
		http.Error(w, "Invalid service path", http.StatusBadRequest)
	}
}

// logf logs with the system's name in front, as the other systems do by hand.
func logf(format string, args ...any) {
	log.Printf(systemName+": "+format, args...)
}
