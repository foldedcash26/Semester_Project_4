import socket
import threading
import csv
import re
import time
import tkinter as tk
from tkinter import scrolledtext

HOST = "0.0.0.0"
PORT = 6767

# ===== GLOBALS =====
log_data = []
buffer = ""
running = True

# =========================================================
# CREATE SERVER
# =========================================================
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen(1)

print(f"Waiting for ESP32 on port {PORT}...")

conn, addr = server.accept()

print(f"Connected by {addr}")

# =========================================================
# TKINTER UI
# =========================================================
root = tk.Tk()
root.title("ESP32 Control Panel")
root.geometry("700x500")

# ===== OUTPUT BOX =====
output_box = scrolledtext.ScrolledText(root, width=100, height=25)
output_box.pack(padx=10, pady=10)

# ===== COMMAND ENTRY =====
command_entry = tk.Entry(root, width=50)
command_entry.pack(pady=5)

# =========================================================
# LOG FUNCTION
# =========================================================
def log(text):
    output_box.insert(tk.END, text + "\n")
    output_box.see(tk.END)

# =========================================================
# RECEIVE THREAD
# =========================================================
def receive_thread():
    global buffer, running

    while running:
        try:
            data = conn.recv(4096)

            if not data:
                log("ESP32 disconnected")
                running = False
                break

            buffer += data.decode(errors="ignore")

            while "\n" in buffer:

                line, buffer = buffer.split("\n", 1)

                message = line.strip()

                log("ESP32: " + message)

                # ===== PARSE SENSOR DATA =====
                match = re.match(r"SF:\s*(.*)", message)

                if match:

                    reading = match.group(1)

                    row = {
                        "time_server": time.time()
                    }

                    pairs = reading.split(";")

                    for pair in pairs:

                        if ":" in pair:

                            key, value = pair.split(":", 1)

                            row[key.strip()] = value.strip()

                    log_data.append(row)

        except Exception as e:
            log(f"Receive error: {e}")
            break

# =========================================================
# EXPORT CSV
# =========================================================
def export_csv():

    if not log_data:
        log("No data to export")
        return

    fieldnames = set()

    for row in log_data:
        fieldnames.update(row.keys())

    fieldnames = ["Time"] + sorted(fieldnames - {"Time"})

    filename = f"esp32_log_{int(time.time())}.csv"

    with open(filename, "w", newline="") as csvfile:

        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        writer.writeheader()

        writer.writerows(log_data)

    log(f"CSV exported: {filename}")

# =========================================================
# SEND COMMAND
# =========================================================
def send_command(cmd):

    try:
        conn.sendall((cmd + "\n").encode())
        log("PC -> ESP32: " + cmd)

    except Exception as e:
        log(f"Send error: {e}")

# =========================================================
# HANDLE ENTER BUTTON
# =========================================================
def on_send():

    command = command_entry.get().strip()

    if not command:
        return

    # ===== COMMAND MAPPING =====
    if command == "stop":
        cmd = "STOP"

    elif command.startswith("change yaw"):
        val = command.split(":")[1]
        cmd = f"change yaw:{val}"

    elif command.startswith("change pitch"):
        val = command.split(":")[1]
        cmd = f"change pitch:{val}"

    elif command.startswith("set yaw"):
        val = command.split(":")[1]
        cmd = f"set yaw:{val}"

    elif command.startswith("set pitch"):
        val = command.split(":")[1]
        cmd = f"set pitch:{val}"

    elif command.startswith("set kp yaw"):
        val = command.split(":")[1]
        cmd = f"kp_yaw:{val}"

    elif command.startswith("set kd yaw"):
        val = command.split(":")[1]
        cmd = f"kd_yaw:{val}"

    elif command.startswith("set kp pitch"):
        val = command.split(":")[1]
        cmd = f"kp_pitch:{val}"

    elif command.startswith("set kd pitch"):
        val = command.split(":")[1]
        cmd = f"kd_pitch:{val}" 

    else:
        cmd = f"THROTTLE:{command}"

    send_command(cmd)

    command_entry.delete(0, tk.END)

# =========================================================
# BUTTONS
# =========================================================
button_frame = tk.Frame(root)
button_frame.pack(pady=5)

send_button = tk.Button(button_frame, text="Send", command=on_send)
send_button.grid(row=0, column=0, padx=5)

stop_button = tk.Button(
    button_frame,
    text="STOP",
    command=lambda: send_command("STOP")
)
stop_button.grid(row=0, column=1, padx=5)

export_button = tk.Button(
    button_frame,
    text="Export CSV",
    command=export_csv
)
export_button.grid(row=0, column=2, padx=5)

# =========================================================
# START RECEIVE THREAD
# =========================================================
t_receive = threading.Thread(target=receive_thread, daemon=True)
t_receive.start()

# =========================================================
# CLOSE WINDOW
# =========================================================
def on_close():
    global running

    running = False

    try:
        conn.close()
        server.close()
    except:
        pass

    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)

# =========================================================
# START GUI
# =========================================================
log("Connected to ESP32")
root.mainloop()