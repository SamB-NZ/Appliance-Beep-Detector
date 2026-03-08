import msvcrt
import csv
import serial
import time
from plotly.subplots import make_subplots
import plotly.graph_objects as go
import plotly.io as pio
import os
from datetime import datetime

pio.renderers.default = "browser"  # ensure graph opens in default browser

PORT = "COM3"  # Windows example. Linux/Mac often "/dev/ttyACM0"
BAUD = 115200
LABELS = [
    "possbeep",
    "posspause",
    "endCycle",   
    "magh - magHAvg",
    "magl - magLAvg",
    "magr - magRAvg",
    "timer_micros",
]

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)
ser.reset_input_buffer()
input("Press ENTER to start recording...")
print("Recording... Press ENTER again to stop.\n")

timestamps = []
data = [[] for _ in LABELS]
t2_START = None

try:
    while True:
        if msvcrt.kbhit() and msvcrt.getch() == b'\r':
            break
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            parts = line.split(",")
            if len(parts) == len(LABELS):
                try:
                    values = [float(v) for v in parts]
                except ValueError:
                    continue
                t2_Arduino = values[-1]
                if t2_START is None:
                    t2_START = t2_Arduino            
                t = (t2_Arduino - t2_START)/1_000_000.0
                timestamps.append(t)
                for i, val in enumerate(values):
                    data[i].append(val)
except KeyboardInterrupt:
    pass

print("Recording stopped.")
ser.close()

fig = make_subplots(specs=[[{"secondary_y": True}]])

for label in LABELS[3:-1]:
    fig.add_trace(
        go.Scatter(x=timestamps, y=data[LABELS.index(label)],
                   mode="lines", name=label),
                   secondary_y=False
    )

for label in LABELS[:3]:
    fig.add_trace(
        go.Scatter(x=timestamps, y=data[LABELS.index(label)],
                   mode="lines", name=label),
                   secondary_y=True                   
    )

fig.update_layout(
    title="Arduino Serial Data (Dual Axis)",
    template="plotly_dark",
    legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1)
)

fig.update_xaxes(title_text="time (s)")
fig.update_yaxes(title_text="magnitude data", secondary_y=False)
fig.update_yaxes(title_text="Beep/Pause (0-1)", secondary_y=True)

os.makedirs("Goertzel_Logging", exist_ok=True)
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

plotname = f"arduino_plot_{timestamp}.html"
path = os.path.join("Goertzel_Logging", plotname)
fig.write_html(path)
print(f"Plot saved as: {path}")

csvname = f"arduino_data_{timestamp}.csv"
csvpath = os.path.join("Goertzel_Logging", csvname)

with open(csvpath, "w", newline="") as f:
    writer = csv.writer(f)

    writer.writerow(["time_s"] + LABELS)

    for i in range(len(timestamps)):
        row = [timestamps[i]] + [data[j][i] for j in range(len(LABELS))]
        writer.writerow(row)

print(f"CSV saved as: {csvpath}")


