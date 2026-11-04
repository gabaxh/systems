# Starting the Artitrax local cloud

How to start the systems on the Artitrax laptop with certificates enabled, using
the certificate authority (`ca`) and `maitreD`.

## How the check works

A system only gets a certificate if the file it is running is on the whitelist:

1. `ca` starts. It signs its own certificate, so it needs nothing from anyone.
2. `maitreD` asks `ca` for a certificate. `ca` only checks that the request comes
   from an allowed IP address.
3. A system such as `loader` asks `ca` for a certificate and states its own
   process ID (PID).
4. `ca` sends that PID and a random challenge to `maitreD`.
5. `maitreD` finds the file the PID is running, hashes it (SHA-256), signs the
   result and sends it back.
6. `ca` checks the signature, then looks for the hash in `ca/whitelist.json`. If
   it is there, `ca` issues the certificate.

The whitelist holds hashes of binaries, so **any rebuild that changes a binary
also changes its hash**. Certificates last only as long as the process, so this
check runs every time a system starts.

## Every boot

The whitelist, configs and CA keys stay on disk. **You do not need to rebuild or
run `build_whitelist.sh` unless the code has changed.**

### 1. Bring up the CAN buses

Skip this if they already come up on their own after a reboot.

```bash
sudo ip link set can0 up type can bitrate 500000
```

```bash
sudo ip link set can1 up type can bitrate 250000
```

### 2. Start the systems

Start each system in its own terminal, from its own folder. Do **not** use `sudo`
or `go run .`.

| Order | System | Command (from the `systems` folder) |
|---|---|---|
| 1 | `ca` | `cd ca && ./ca_ilin` |
| 2 | `maitreD` | `cd maitreD && ./maitreD_ilin` |
| 3 | `esr` | `cd esr && ./esr_ilin` |
| 4 | `orchestrator` | `cd orchestrator && ./orchestrator_ilin` |
| 5 | `loader` | `cd loader && ./loader_ilin` |

Systems keep retrying, so a wrong order only causes error lines for a moment.
This order keeps the logs easy to read.

### 3. Check that it worked

- The `maitreD` terminal shows `System maitreD's parsed Certificate:`.
- The `ca` terminal shows `certify: attested pid=...` for each system except `ca`
  and `maitreD`.
- Each system shows `up with its web server available at https://...`.

`failed to find lead registrar` is normal until `esr` is up.

## After a code change

Run this from the `systems` folder, after `git pull` or any edit:

```bash
./build_whitelist.sh
```

The script builds `ca`, `maitreD` and every system in its `SYSTEMS` list as
`<name>_ilin`, then writes their hashes to `ca/whitelist.json`. Then restart every
running system, so that each one runs its new binary.

Always use the script rather than `go build` on its own, because a binary with a
new hash is refused until the whitelist is updated. Run it on the Artitrax: the
whitelist has to match the binaries that run there.

## First-time setup

Do this on a fresh checkout, or after deleting a `systemconfig.json`.

1. Build the binaries and create the whitelist:

   ```bash
   ./build_whitelist.sh
   ```

2. Start each system once to generate its config. It writes `systemconfig.json`
   and exits with `new config file was created`, which is expected.

   ```bash
   for s in ca maitreD esr orchestrator loader gamer guetteur vigie cartographer collector; do (cd "$s" && timeout 10 "./${s}_ilin"); done
   ```

   Only run this loop for systems with no config yet. A system that already has
   one starts normally and is stopped after 10 seconds.

3. Point the configs at `127.0.0.1`. `ca` only allows `maitreD` from `127.0.0.1`
   and `::1` by default, and the generated configs use the laptop's real IP.

   ```bash
   sed -i 's#//[pc-IP-address]:#//127.0.0.1:#g' */systemconfig.json
   ```

   - change [pc-IP-address] to the ip adress of the pc you are using.

   If the laptop has a different IP, use the one in the generated files. Change
   the address only: `http` and `https` must stay as they are. The `authorizer`
   URL stays empty, which leaves the authorizer off.

4. Follow [Every boot](#every-boot).

## Adding a new system

1. Add it to `SYSTEMS` in `build_whitelist.sh`.
2. Run `./build_whitelist.sh`, then restart the running systems.
3. Start the new system once to generate its config.
4. Run the `sed` command above.
5. Start the system again.




