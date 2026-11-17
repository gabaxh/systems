# commander

The page a person works the vehicle from. It shows the map, the vehicle and the
route, passes a clicked destination to the [navigator](../navigator/), and tells the
[chauffeur](../chauffeur/) when to go. It plans nothing and drives nothing.

> **The page's Stop button is not a safety device.** A page, a browser or a tunnel
> can fail. **Keep the pad in hand:** that is the safety stop.

## Using it

Open this on the machine the commander runs on:

```
http://localhost:20207/commander/Console/view
```

From another computer, tunnel to it, and open the same address:

```bash
ssh -L 20207:localhost:20207 <user>@<artitrax-laptop>
```

1. **Map the area by hand,** and leave the cartographer running. Restarting it
   starts a new map and drops the destination.
2. **Hand over:** take control with the pad, then release it. See
   [who drives](../loader/README.md#who-drives).
3. **Click where the vehicle should go.** The route appears in red, its end in blue.
   If it ends away from the click, the page says how far, and marks the click.
4. **Check the route, and press Start.** Nothing moves before that.
5. **If it holds,** the page says why. Look, and press Start to go on.
6. On arrival it comes to rest and releases control. Click again, with no pad needed.

Only ground the cartographer has seen in this run can be driven to, and only going
forward: a destination behind the vehicle needs a loop or about 5 m of turning width.

## Services

| service | what it is |
|---|---|
| `view` | **GET** the page |
| `overview` | **GET** what the page shows, as JSON |
| `mapImage` | **GET** the seen part of the map as a PNG, with where it lies |
| `destination` | **PUT** a `PoseA` in the map or a `PolarA` from the vehicle, for the navigator. `409` while on a route |
| `command` | **PUT** `?do=start`, `cancel` or `stop` |

So everything the page does can be sent instead of clicked:

```bash
C=http://localhost:20207/commander/Console
curl -X PUT $C/destination -d '{"magnitude": 12, "direction": 0, "frame": "vehicle", "version": "PolarA_v1.0"}'
```

```bash
curl -X PUT "$C/command?do=start"
```

| command | goes to |
|---|---|
| `start` | the chauffeur's `engage 1`; `409` if there is no route |
| `cancel` | the chauffeur's `engage 0` and the navigator's `cancelGoal` |
| `stop` | the loader's `stop`. Only the pad can take the vehicle back after it |

## Only from this machine

A browser cannot present the certificate the HTTPS port asks for, so the page is
reached over plain HTTP, which has no login. Anyone who could reach it could send
the vehicle somewhere, so it answers only requests from this machine, and an SSH
tunnel counts as that. Two more things keep a web page in another tab from pressing
the buttons: the Host header must name this machine, and every command is a PUT.

In an authorized cloud the policy must allow the commander to read the
cartographer's `map` and `pose`, the navigator's `path` and the chauffeur's
`drivingState`, and to write the navigator's `goal` and `cancelGoal`, the
chauffeur's `engage` and the loader's `stop`.

| field | default | |
|---|---|---|
| `vehicle` | `{"Model": ["artitrax"]}` | picks the loader, for its stop |

## Not here yet

- **The reason a route ends away from the click.** The navigator gives it in a
  response header, which the framework's consumers do not see. It is in the
  navigator's log, and the page gives the distance.
- **Saved maps.** The map is the current run's.
- **A login,** to use the page from elsewhere without a tunnel.
