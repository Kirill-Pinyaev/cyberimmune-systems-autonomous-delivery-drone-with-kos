document.getElementById('submit').onclick = submit;
document.getElementById('submit-speed').onclick = submitSpeed;
document.getElementById('submit-events').onclick = submitEvents;
document.getElementById('submit-telemetry-csv').onclick = submitTelemetryCsv;

let chartInstance = null;

function toggleContainers(logsVisible, chartVisible, eventsVisible) {
    document.getElementById('logs-container').classList.toggle('hidden', !logsVisible);
    document.getElementById('speed-chart-container').classList.toggle('hidden', !chartVisible);
    document.getElementById('events-container').classList.toggle('hidden', !eventsVisible);
}


async function submit() {
    let id = document.getElementById('id').value;
    if (id != null && id !== '') {
        toggleContainers(true, false, false);
        document.getElementById('events-container').innerHTML = "";
        let chartContainer = document.getElementById('speed-chart').getContext('2d');
        chartContainer.clearRect(0, 0, chartContainer.canvas.width, chartContainer.canvas.height);
        if (chartInstance) {
            chartInstance.destroy();
            chartInstance = null;
        }

        let logs_resp = await fetch('logs/get_logs?id=' + id);
        let logs = await logs_resp.text();
        logs = logs.split('\n');
        let logs_container = document.getElementById('logs-container');
        logs_container.innerHTML = "";
        for (let idx = 0; idx < logs.length; idx++) {
            let logs_text = document.createElement('p');
            logs_text.style.color = 'black';
            logs_text.textContent = logs[idx];
            logs_container.appendChild(logs_text);
        }
    } else {
        alert('Введите серийный номер.')
    }
}

async function submitSpeed() {
    let id = document.getElementById('id').value;
    if (id != null && id !== '') {
        toggleContainers(false, true, false);

        let response = await fetch('logs/get_telemetry_csv?id=' + id);
        let csv = await response.text();
        let rows = csv.split('\n').slice(1);
        let times = [];
        let speeds = [];
        rows.forEach(row => {
            let cols = row.split(',');
            if (cols.length >= 7) {
                times.push(new Date(cols[0]));
                speeds.push(parseFloat(cols[7]));
            }
        });
        drawChart(times, speeds);
    } else {
        alert('Введите серийный номер.')
    }
}

async function submitEvents() {
    let id = document.getElementById('id').value;
    if (id != null && id !== '') {
        toggleContainers(false, false, true);
        document.getElementById('logs-container').innerHTML = "";
        let chartContainer = document.getElementById('speed-chart').getContext('2d');
        chartContainer.clearRect(0, 0, chartContainer.canvas.width, chartContainer.canvas.height);
        if (chartInstance) {
            chartInstance.destroy();
            chartInstance = null;
        }

        let events_resp = await fetch('logs/get_events?id=' + id);
        let events = await events_resp.json();
        let events_container = document.getElementById('events-container');
        events_container.innerHTML = "";

        if (events.length > 0) {
            let table = document.createElement('table');
            table.className = 'log-table'; // Add a class for styling
            let thead = document.createElement('thead');
            let tbody = document.createElement('tbody');
            let headRow = document.createElement('tr');
            
            let headers = ['Timestamp', 'Type', 'Event'];
            headers.forEach(headerText => {
                let th = document.createElement('th');
                th.textContent = headerText;
                headRow.appendChild(th);
            });

            thead.appendChild(headRow);
            table.appendChild(thead);

            events.forEach(event => {
                let row = document.createElement('tr');
                
                let timestampCell = document.createElement('td');
                timestampCell.textContent = new Date(event.timestamp).toLocaleString();
                row.appendChild(timestampCell);

                let typeCell = document.createElement('td');
                typeCell.textContent = event.type;
                row.appendChild(typeCell);

                let eventCell = document.createElement('td');
                eventCell.textContent = event.event;
                row.appendChild(eventCell);

                tbody.appendChild(row);
            });

            table.appendChild(tbody);
            events_container.appendChild(table);
        } else {
            events_container.textContent = 'No events found.';
        }
    } else {
        alert('Введите серийный номер.');
    }
}

async function submitTelemetryCsv() {
    let id = document.getElementById('id').value;
    if (id != null && id !== '') {
        try {
            let response = await fetch('logs/get_telemetry_csv?id=' + id);
            if (response.ok) {
                let blob = await response.blob();
                let url = window.URL.createObjectURL(blob);
                let a = document.createElement('a');
                a.style.display = 'none';
                a.href = url;
                a.download = `telemetry_${id}.csv`;
                document.body.appendChild(a);
                a.click();
                window.URL.revokeObjectURL(url);
                a.remove();
            } else {
                alert('Ошибка при скачивании телеметрии: ' + response.statusText);
            }
        } catch (error) {
            alert('Ошибка сети при скачивании телеметрии: ' + error);
        }
    } else {
        alert('Введите серийный номер.');
    }
}

function drawChart(labels, data) {
    let ctx = document.getElementById('speed-chart').getContext('2d');
    if (chartInstance) {
        chartInstance.destroy();
    }
    chartInstance = new Chart(ctx, {
        type: 'line',
        data: {
            labels: labels,
            datasets: [{
                label: 'Speed over Time',
                data: data,
                borderColor: 'rgba(75, 192, 192, 1)',
                borderWidth: 1,
                fill: false
            }]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            scales: {
                x: {
                    type: 'time',
                    time: {
                        unit: 'minute'
                    }
                },
                y: {
                    beginAtZero: true
                }
            }
        }
    });
}