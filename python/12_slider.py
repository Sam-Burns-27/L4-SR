import tkinter as tk
import serial

# === Change this to match your ESP32 port ===
SERIAL_PORT = "COM3"
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print("✅ Connected to ESP32")
except:
    print("❌ Could not connect to ESP32. Check your COM port.")
    ser = None

def send_angle(leg, index, val):
    angle = int(float(val))
    cmd = f"servo,{leg},{index},{angle}\n"
    print(f"→ Sending: {cmd.strip()}")
    if ser and ser.is_open:
        ser.write(cmd.encode())

root = tk.Tk()
root.title("Quadruped Servo Controller")

for leg in range(1, 5):        # Legs 1 to 4
    frame = tk.LabelFrame(root, text=f"Leg {leg}", padx=10, pady=10)
    frame.pack(side=tk.LEFT, padx=10, pady=10)

    for index in range(1, 4):  # Servo indexes 1 to 3
        slider = tk.Scale(
            frame,
            from_=0,
            to=180,
            orient=tk.VERTICAL,
            label=f"S{index}",
            command=lambda val, l=leg, i=index: send_angle(l, i, val)
        )
        slider.set(90)
        slider.pack(padx=5, pady=5)

root.mainloop()
