fetch("http://localhost:8000/adas")
  .then(r => r.json())
  .then(cfg => {
    collision.checked = cfg.Collision;
    airbag.checked = cfg.Airbag;
    esp.checked = cfg.ESP;
  });

function save() {
  fetch("http://localhost:8000/adas", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({
      Collision: collision.checked,
      Airbag: airbag.checked,
      ESP: esp.checked
    })
  });
}
