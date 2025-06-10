async function post(endpoint) {
  const res = await fetch(endpoint, { method: "POST" });
  const json = await res.json();
  alert(json.status);
}

const socket = new WebSocket(`ws://${location.host}/ws/telemetry`);

socket.onmessage = (event) => {
  const data = JSON.parse(event.data);
  if (data.error) return console.error("Telemetry error:", data.error);

  ["roll", "pitch", "yaw", "height", "battery"].forEach(key => {
    const el = document.getElementById(key);
    if (el) el.textContent = data[key];
  });

  if (data.rc) {
    data.rc.forEach((val, idx) => {
      const el = document.getElementById(`rc${idx}`);
      if (el) el.textContent = val;
    });
  }
};

socket.onerror = err => console.error("WebSocket error:", err);
