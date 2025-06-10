// async function sendCommand(endpoint) {
//     let response = await fetch(endpoint, { method: 'POST' });
//     let result = await response.json();
//     alert(result.status);
// }

async function post(endpoint) {
  const response = await fetch(endpoint, { method: 'POST' });
  const result = await response.json();
  alert(result.status);
}

// WebSocket for telemetry
const socket = new WebSocket(`ws://${location.host}/ws/telemetry`);

socket.onmessage = (event) => {
  const data = JSON.parse(event.data);
  document.getElementById("roll").textContent = data.roll ?? '-';
  document.getElementById("pitch").textContent = data.pitch ?? '-';
  document.getElementById("yaw").textContent = data.yaw ?? '-';
  document.getElementById("height").textContent = data.height ?? '-';
  document.getElementById("battery").textContent = data.battery ?? '-';
};

socket.onerror = (err) => {
  console.error("WebSocket error:", err);
};
