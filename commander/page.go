/*******************************************************************************
 * Copyright (c) 2026 Synecdoque
 *
 * The software is licensed under the MIT License. See the LICENSE file in this
 * repository for details.
 ***************************************************************************SDG*/

package main

// pageHTML is the whole page: nothing is fetched from anywhere else, because the
// vehicle's network may reach nothing else. Text from other systems is always set
// as text, never as markup.
const pageHTML = `<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Commander</title>
<style>
  :root { --ink:#1b1f24; --paper:#f4f5f6; --line:#c9ced4; --go:#1a7f37; --hold:#b35900; --stop:#c62828; --panel:#fff; }
  @media (prefers-color-scheme: dark) { :root { --ink:#e6e8eb; --paper:#16191d; --line:#3a4149; --panel:#22262b; } }
  * { box-sizing: border-box; }
  html, body { margin: 0; height: 100%; background: var(--paper); color: var(--ink);
               font: 15px/1.4 system-ui, sans-serif; }
  body { display: flex; flex-direction: column; }
  header { display: flex; flex-wrap: wrap; align-items: center; gap: 8px 12px; padding: 10px 16px;
           background: var(--panel); border-bottom: 1px solid var(--line); }
  #state { flex: 1 1 280px; font-weight: 600; }
  #state.go { color: var(--go); } #state.hold { color: var(--hold); } #state.stop { color: var(--stop); }
  button { font: inherit; font-weight: 600; padding: 8px 18px; border-radius: 6px; border: 1px solid var(--line);
           background: var(--panel); color: var(--ink); cursor: pointer; }
  button:disabled { opacity: .45; cursor: default; }
  #start:not(:disabled) { background: var(--go); border-color: var(--go); color: #fff; }
  #stop { background: var(--stop); border-color: var(--stop); color: #fff; }
  #note { padding: 6px 16px; min-height: 2em; font-size: 14px; }
  #note.bad { color: var(--stop); font-weight: 600; }
  #wrap { flex: 1; min-height: 0; padding: 0 16px 16px; }
  canvas { display: block; width: 100%; height: 100%; background: #c8c8c8; border: 1px solid var(--line);
           cursor: crosshair; touch-action: none; }
</style>
</head>
<body>
<header>
  <div id="state" role="status">Connecting</div>
  <button id="start" disabled>Start</button>
  <button id="cancel" disabled>Cancel</button>
  <button id="stop">Stop</button>
</header>
<div id="note">Click on the map to set a destination. The pad is the safety stop: keep it in hand.</div>
<div id="wrap"><canvas id="map"></canvas></div>
<script>
"use strict";
var base = location.pathname.replace(/\/view$/, "");
var canvas = document.getElementById("map");
var ctx = canvas.getContext("2d");
var stateEl = document.getElementById("state");
var noteEl = document.getElementById("note");
var startBtn = document.getElementById("start");
var cancelBtn = document.getElementById("cancel");
var stopBtn = document.getElementById("stop");

var picture = null;   // the map picture's numbers
var image = null;     // and the picture
var view = null;      // the last overview
var haveGeneration = 0;

function note(text, bad) {
  noteEl.textContent = text;
  noteEl.className = bad ? "bad" : "";
}

// How the picture sits on the canvas, worked out afresh for each drawing: the map
// grows, and the window can change size.
function layout() {
  var w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
  var scale = Math.min(w / picture.width, h / picture.height);
  return { scale: scale, left: (w - picture.width * scale) / 2, top: (h - picture.height * scale) / 2 };
}

function toCanvas(x, y, l) {
  return {
    x: l.left + (x - picture.minX) / picture.resolution * l.scale,
    y: l.top + (picture.maxY - y) / picture.resolution * l.scale
  };
}

function toMap(px, py, l) {
  return {
    x: picture.minX + (px - l.left) / l.scale * picture.resolution,
    y: picture.maxY - (py - l.top) / l.scale * picture.resolution
  };
}

function draw() {
  if (!picture || !image) { return; }
  var l = layout();
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(image, l.left, l.top, picture.width * l.scale, picture.height * l.scale);
  if (!view) { return; }

  if (view.Route && view.Route.X && view.Route.X.length > 1) {
    ctx.strokeStyle = "#d32f2f";
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (var i = 0; i < view.Route.X.length; i++) {
      var p = toCanvas(view.Route.X[i], view.Route.Y[i], l);
      if (i === 0) { ctx.moveTo(p.x, p.y); } else { ctx.lineTo(p.x, p.y); }
    }
    ctx.stroke();
  }
  if (view.Goal) {
    var asked = toCanvas(view.Goal.AskedX, view.Goal.AskedY, l);
    var end = toCanvas(view.Goal.X, view.Goal.Y, l);
    if (view.Goal.MovedBy > 0.05) {
      ctx.strokeStyle = "#7b1fa2";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(asked.x - 6, asked.y - 6); ctx.lineTo(asked.x + 6, asked.y + 6);
      ctx.moveTo(asked.x + 6, asked.y - 6); ctx.lineTo(asked.x - 6, asked.y + 6);
      ctx.stroke();
    }
    ctx.fillStyle = "#1565c0";
    ctx.beginPath();
    ctx.arc(end.x, end.y, 6, 0, 2 * Math.PI);
    ctx.fill();
  }
  if (view.Pose) {
    var at = toCanvas(view.Pose.X, view.Pose.Y, l);
    var th = view.Pose.Heading * Math.PI / 180;
    ctx.save();
    ctx.translate(at.x, at.y);
    ctx.rotate(-th); // the canvas's y runs down, the map's up
    ctx.fillStyle = "#1a7f37";
    ctx.strokeStyle = "#fff";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(13, 0); ctx.lineTo(-8, 8); ctx.lineTo(-4, 0); ctx.lineTo(-8, -8);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }
}

function send(path, body) {
  var options = { method: "PUT" };
  if (body) {
    options.headers = { "Content-Type": "application/json" };
    options.body = JSON.stringify(body);
  }
  return fetch(base + path, options).then(function (r) {
    if (r.ok) { return r.status === 204 ? null : r.json(); }
    return r.text().then(function (text) { throw new Error(text.trim() || r.statusText); });
  });
}

function command(what) {
  return send("/command?do=" + what).catch(function (e) { note(e.message, true); });
}

canvas.addEventListener("click", function (e) {
  if (!picture || !image) { return; }
  if (view && view.State.Active) {
    note("The vehicle is on a route: cancel it before setting another destination.", true);
    return;
  }
  var box = canvas.getBoundingClientRect();
  var at = toMap(e.clientX - box.left, e.clientY - box.top, layout());
  note("Planning a route to (" + at.x.toFixed(2) + ", " + at.y.toFixed(2) + ")");
  // The map's frame goes with the click, so a click on a map the cartographer has
  // since replaced is refused and not taken to mean another place.
  send("/destination", { x: at.x, y: at.y, frame: picture.frame, version: "PoseA_v1.0" }).then(function (g) {
    if (g.MovedBy > 0.3) {
      note("The route ends " + g.MovedBy.toFixed(1) + " m from where you clicked, at the nearest point it can reach. Check it, then press Start.", true);
    } else {
      note("Check the route, then press Start.");
    }
    refresh();
  }).catch(function (err) { note(err.message, true); });
});

startBtn.addEventListener("click", function () {
  command("start").then(function () { note("The pad is the safety stop: keep it in hand."); });
});
cancelBtn.addEventListener("click", function () {
  command("cancel").then(function () { note("Cancelled. Click on the map to set a destination."); });
});
stopBtn.addEventListener("click", function () {
  command("stop").then(function () { note("Stop sent. Only the pad can take the vehicle back.", true); });
});

function showState() {
  var s = view.State;
  stateEl.textContent = s.Text;
  stateEl.className = !s.Reachable ? "stop"
    : s.Code >= 20 ? "stop"
    : s.Code >= 10 ? "hold"
    : s.Code === 2 || s.Code === 3 ? "go" : "";
  var holding = s.Code >= 10 && s.Code < 20;
  startBtn.disabled = !s.Reachable || !view.Route || (s.Active && !holding);
  cancelBtn.disabled = !s.Active && !view.Goal;
}

function refresh() {
  return fetch(base + "/overview").then(function (r) { return r.json(); }).then(function (o) {
    view = o;
    showState();
    if (o.Generation !== haveGeneration) { return loadMap(o.Generation); }
    if (!picture && o.MapError) { note("No map yet: " + o.MapError); }
  }).then(draw).catch(function () {
    stateEl.textContent = "The commander cannot be reached";
    stateEl.className = "stop";
  });
}

function loadMap(generation) {
  return fetch(base + "/mapImage").then(function (r) {
    if (!r.ok) { throw new Error("no map"); }
    return r.json();
  }).then(function (p) {
    return new Promise(function (resolve, reject) {
      var img = new Image();
      img.onload = function () {
        // Taken together, so the picture and the numbers that place it always match.
        picture = p; image = img; haveGeneration = generation;
        resolve();
      };
      img.onerror = reject;
      img.src = "data:image/png;base64," + p.png;
    });
  });
}

window.addEventListener("resize", draw);
setInterval(refresh, 500);
refresh();
</script>
</body>
</html>
`
