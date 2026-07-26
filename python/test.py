import tkinter as tk
import serial

# === Change this to match your ESP32 port ===
SERIAL_PORT = "COM3"  # e.g. COM4 or /dev/ttyUSB0
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print("✅ Connected to ESP32")
except:
    print("❌ Could not connect to ESP32. Check your port.")
    ser = None

def send_angle(val):
    angle = int(val)
    cmd = f"servo,1,1,{angle}\n"
    print(f"→ Sending: {cmd.strip()}")
    if ser and ser.is_open:
        ser.write(cmd.encode())

root = tk.Tk()
root.title("Servo Angle Test")

slider = tk.Scale(root, from_=0, to=180, orient=tk.HORIZONTAL, command=send_angle)
slider.set(90)
slider.pack(padx=20, pady=20)

root.mainloop()
