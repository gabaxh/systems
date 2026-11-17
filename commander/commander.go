/*******************************************************************************
 * Copyright (c) 2026 Synecdoque
 *
 * The software is licensed under the MIT License. See the LICENSE file in this
 * repository for details.
 ***************************************************************************SDG*/

// The commander is the page a person works the vehicle from: it shows the map,
// the vehicle and the route, passes a clicked destination to the navigator, and
// tells the chauffeur when to go. It plans nothing and drives nothing.
package main

import (
	"context"
	"crypto/x509/pkix"
	"encoding/json"
	"log"
	"net"
	"net/http"
	"time"

	"github.com/sdoque/mbaigo/components"
	"github.com/sdoque/mbaigo/usecases"
)

func main() {
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	sys := components.NewSystem("commander", ctx)
	usecases.WatchShutdown(&sys, cancel)

	sys.Husk = &components.Husk{
		Description: "the page a person sets the vehicle's destination from",
		Details:     map[string][]string{"Developer": {"Synecdoque"}},
		Host:        components.NewDevice(),
		ProtoPort:   map[string]int{"https": 30207, "http": 20207, "coap": 0},
		InfoLink:    "https://github.com/sdoque/systems/tree/main/commander",
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

// fromThisMachine is whether a request came from a browser on this machine, or
// through an SSH tunnel to it.
//
// A browser cannot present the certificate the HTTPS port asks for, so the page is
// reached over plain HTTP, which has no login. Anyone who could reach it could send
// the vehicle somewhere, so it answers only locally. The Host header is checked as
// well as the address: without it a web page open in a browser on this machine
// could reach the commander by pointing a name of its own at 127.0.0.1.
func fromThisMachine(r *http.Request) bool {
	host, _, err := net.SplitHostPort(r.RemoteAddr)
	if err != nil {
		return false
	}
	if ip := net.ParseIP(host); ip == nil || !ip.IsLoopback() {
		return false
	}
	name, _, err := net.SplitHostPort(r.Host)
	if err != nil {
		name = r.Host
	}
	if name == "localhost" {
		return true
	}
	if len(name) > 1 && name[0] == '[' {
		name = name[1 : len(name)-1]
	}
	ip := net.ParseIP(name)
	return ip != nil && ip.IsLoopback()
}

func serving(t *Traits, w http.ResponseWriter, r *http.Request, servicePath string) {
	if !fromThisMachine(r) {
		http.Error(w, "the commander answers only on the machine it runs on; use an SSH tunnel", http.StatusForbidden)
		return
	}
	switch servicePath {
	case "view":
		t.viewService(w, r)
	case "overview":
		t.overviewService(w, r)
	case "mapImage":
		t.mapImageService(w, r)
	case "destination":
		t.destinationService(w, r)
	case "command":
		t.commandService(w, r)
	default:
		http.Error(w, "Invalid service path", http.StatusBadRequest)
	}
}
