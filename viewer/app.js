// Track viewer: draws the GNSS-only, fused and reference tracks from a
// `gins export` GeoJSON file, with simulated outage windows highlighted.
(function () {
  "use strict";

  const STYLE = {
    gnss: { color: "var(--gnss)", label: "GNSS only", weight: 2, dash: "4 4" },
    fused: { color: "var(--fused)", label: "Fused (GNSS + IMU)", weight: 3 },
    reference: { color: "var(--reference)", label: "Reference (RTK)", weight: 2 },
    outage: { color: "var(--outage)", label: "GNSS masked", weight: 6 },
  };
  const css = getComputedStyle(document.documentElement);
  const resolve = (c) => (c.startsWith("var(") ? css.getPropertyValue(c.slice(4, -1)).trim() : c);

  const map = L.map("map", { preferCanvas: true }).setView([49.2606, -123.246], 13);
  L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 20,
    maxNativeZoom: 19,
    attribution: "&copy; OpenStreetMap contributors",
  }).addTo(map);

  let layers = {};

  function clear() {
    Object.values(layers).forEach((l) => map.removeLayer(l));
    layers = {};
    document.getElementById("tracks").innerHTML = "";
    document.getElementById("stats").innerHTML = "";
  }

  function lengthKm(coords) {
    let d = 0;
    for (let i = 1; i < coords.length; i++) {
      d += map.distance([coords[i - 1][1], coords[i - 1][0]], [coords[i][1], coords[i][0]]);
    }
    return d / 1000;
  }

  function show(fc) {
    clear();
    const groups = {};
    for (const f of fc.features || []) {
      const name = f.properties && f.properties.track;
      if (!STYLE[name]) continue;
      (groups[name] = groups[name] || []).push(f);
    }
    const order = ["gnss", "reference", "fused", "outage"];
    const bounds = L.latLngBounds([]);
    const list = document.getElementById("tracks");
    for (const name of order) {
      if (!groups[name]) continue;
      const s = STYLE[name];
      const layer = L.geoJSON(groups[name], {
        style: { color: resolve(s.color), weight: s.weight, opacity: 0.9, dashArray: s.dash || null },
        onEachFeature: (f, l) => {
          if (name === "outage") l.bindTooltip(`GNSS masked for ${f.properties.seconds} s`);
        },
      }).addTo(map);
      layers[name] = layer;
      bounds.extend(layer.getBounds());

      const li = document.createElement("li");
      const box = document.createElement("input");
      box.type = "checkbox";
      box.checked = true;
      box.addEventListener("change", () => (box.checked ? layer.addTo(map) : map.removeLayer(layer)));
      const sw = document.createElement("span");
      sw.className = "swatch";
      sw.style.background = resolve(s.color);
      const label = document.createElement("span");
      label.textContent = `${s.label} (${groups[name].length > 1 ? groups[name].length + " segments" : groups[name][0].properties.points + " pts"})`;
      li.append(box, sw, label);
      list.append(li);
    }
    if (bounds.isValid()) map.fitBounds(bounds, { padding: [20, 20] });

    document.getElementById("ride").textContent = (fc.properties && fc.properties.ride_id) || "ride";
    const stats = document.getElementById("stats");
    if (groups.reference) {
      stats.innerHTML += `<div>Distance: ${lengthKm(groups.reference[0].geometry.coordinates).toFixed(2)} km</div>`;
    }
    if (groups.outage) {
      const secs = groups.outage.reduce((a, f) => a + (f.properties.seconds || 0), 0);
      stats.innerHTML += `<div>Masked: ${groups.outage.length} windows, ${secs.toFixed(0)} s</div>`;
    }
  }

  function load(file) {
    const reader = new FileReader();
    reader.onload = () => {
      try {
        show(JSON.parse(reader.result));
      } catch (e) {
        document.getElementById("ride").textContent = `Could not read ${file.name}: ${e.message}`;
      }
    };
    reader.readAsText(file);
  }

  document.getElementById("file").addEventListener("change", (e) => e.target.files[0] && load(e.target.files[0]));
  const body = document.body;
  ["dragenter", "dragover"].forEach((ev) =>
    body.addEventListener(ev, (e) => { e.preventDefault(); body.classList.add("drag"); }));
  ["dragleave", "drop"].forEach((ev) =>
    body.addEventListener(ev, (e) => { e.preventDefault(); body.classList.remove("drag"); }));
  body.addEventListener("drop", (e) => e.dataTransfer.files[0] && load(e.dataTransfer.files[0]));

  // ?demo loads viewer/demo.geojson when served over HTTP (python -m http.server).
  if (location.search.includes("demo")) {
    fetch("demo.geojson").then((r) => r.json()).then(show).catch(() => {});
  }
})();
