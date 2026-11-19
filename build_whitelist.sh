#!/usr/bin/env bash
# Build the systems this vehicle runs and approve exactly those binaries.
set -euo pipefail

# ca and maitreD are built but not listed: they are never attested
SYSTEMS="esr orchestrator guetteur cartographer loader gamer collector vigie kgrapher envoy"


for s in ca maitreD $SYSTEMS; do
	(cd "$s" && go build -o "${s}_ilin" .)
done


{
  echo '['
  sep=' '
  for s in $SYSTEMS; do
	echo " $sep\"$(sha256sum "$s/${s}_ilin" | cut -d' ' -f1)\""
    sep=','
  done
  echo ']'
} > ca/whitelist.json
