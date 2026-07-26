import tkinter as tk
from tkinter import messagebox
import serial
import time

# Set up serial communication
SERIAL_PORT = 'COM5'  # Update this to match your system
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print("✅ Connected to ESP32")
except:
    print("❌ Could not connect to ESP32. Check your COM port.")
    ser = None

# Dictionary to hold pose presets
poses = {
    'Stand': [90] * 12,
    'Sit': [90] * 12,
    'Rest': [90] * 12
}

# Current slider values
current_angles = [90] * 12

# Send angle command to servo
def send_servo_command(servo_num, angle):
    if ser and ser.is_open:
        # Convert servo_num (0–11) to (leg, index)
        leg = servo_num // 3 + 1
        index = servo_num % 3 + 1
        cmd = f"servo,{leg},{index},{angle}\n"
        ser.write(cmd.encode())
        print(f"→ Sent: {cmd.strip()}")


# Update angle from slider
def update_angle(servo_num, val):
    angle = int(float(val))
    current_angles[servo_num] = angle
    send_servo_command(servo_num, angle)

# Save current angles as pose
def save_pose(pose_name):
    poses[pose_name] = current_angles.copy()
    messagebox.showinfo("Pose Saved", f"Saved current angles as {pose_name}")

# Move robot to saved pose
def move_to_pose(pose_name):
    if pose_name in poses:
        target = poses[pose_name]
        steps = 10  # Smooth interpolation steps
        for step in range(1, steps + 1):
            for i in range(12):
                start = current_angles[i]
                end = target[i]
                angle = int(start + (end - start) * step / steps)
                sliders[i].set(angle)
                send_servo_command(i, angle)
            time.sleep(0.1)  # Delay between each step
        # Save final angles
        for i in range(12):
            current_angles[i] = target[i]
    else:
        messagebox.showerror("Pose Error", f"Pose '{pose_name}' not found")

# UI setup
root = tk.Tk()
root.title("Quadruped Pose Controller")

sliders = []
for i in range(12):
    frame = tk.Frame(root)
    frame.pack()
    label = tk.Label(frame, text=f"Servo {i}")
    label.pack(side="left")
    slider = tk.Scale(frame, from_=0, to=180, orient="horizontal",
                      command=lambda val, idx=i: update_angle(idx, val))
    slider.set(90)
    slider.pack(side="left")
    sliders.append(slider)

button_frame = tk.Frame(root)
button_frame.pack(pady=10)

for pose in ['Stand', 'Sit', 'Rest']:
    tk.Button(button_frame, text=f"Save {pose}", command=lambda p=pose: save_pose(p)).pack(side="left", padx=5)
    tk.Button(button_frame, text=f"Move to {pose}", command=lambda p=pose: move_to_pose(p)).pack(side="left", padx=5)

root.mainloop()
