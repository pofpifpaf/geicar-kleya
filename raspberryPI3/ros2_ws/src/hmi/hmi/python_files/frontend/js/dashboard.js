const ws = new WebSocket("ws://localhost:8000/ws/state");

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);

  document.getElementById("speed").innerText = data.speed.toFixed(0);
  document.getElementById("rpm").innerText = data.RPM.toFixed(0);
  document.getElementById("battery").innerText = data.battery.toFixed(0) + "%";
};
