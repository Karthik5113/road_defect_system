const API = "https://your-render-url.onrender.com/defects";

const map = L.map('map').setView([12.97, 77.59], 13);

L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);

async function loadData() {

  const res = await fetch(API);
  const data = await res.json();

  let counts = {};

  data.forEach(d => {

    L.circleMarker([d.latitude, d.longitude], {
      color: d.defect_type === "pothole" ? "red" : "orange"
    }).addTo(map);

    counts[d.defect_type] = (counts[d.defect_type] || 0) + 1;
  });

  new Chart(document.getElementById("chart"), {
    type: "pie",
    data: {
      labels: Object.keys(counts),
      datasets: [{ data: Object.values(counts) }]
    }
  });
}

loadData();