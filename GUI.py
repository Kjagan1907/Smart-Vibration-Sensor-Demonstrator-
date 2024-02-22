import serial
import sounddevice as sd
import tkinter as tk
from tkinter import Label, Button, StringVar
from threading import Thread
import numpy as np
import time

# Variable to keep track of the currently playing audio
current_audio = None

# Function to play a sine wave using sounddevice
def play_sine(frequency, duration):
    sample_rate = 44100
    samples = np.arange(0, duration, 1 / sample_rate)
    sine_wave = 0.4 * np.sin(2 * np.pi * frequency * samples)

    global current_audio
    if current_audio:
        stop_audio()
    current_audio = sd.play(sine_wave, sample_rate, loop=True)

# Function to play white noise using sounddevice
def play_white_noise(duration):
    sample_rate = 44100
    white_noise = np.random.normal(0, 1.5, int(duration * sample_rate))

    global current_audio
    if current_audio:
        stop_audio()
    current_audio = sd.play(white_noise, sample_rate, loop=True)

# Function to stop playing audio
def stop_audio():
    global current_audio
    if current_audio:
        sd.stop()
        current_audio = None

# Function to handle button clicks
def button_click(audio_type_var, serial_data_var):
    try:
        ser = serial.Serial('COM9', 115200, timeout=1)

        while True:
            # Check if data is available from the serial port
            if ser.in_waiting > 0:
                # Read data from Arduino
                arduino_data = ser.readline().decode('utf-8').rstrip()
                print(f"Received data from Arduino: {arduino_data}")

                # Update the serial data label
                serial_data_var.set(f"Serial Data: {arduino_data}")

            # Sleep for a short duration to reduce CPU usage
            time.sleep(0.1)

    except serial.SerialException as e:
        print(f"Serial Communication Error: {e}")
    finally:
        stop_audio()

# Main GUI window
root = tk.Tk()
root.title("Audio Player")

# Set the GUI window size to be equal to the screen size
screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()
root.geometry(f"{screen_width}x{screen_height}")

# Variables for GUI elements
audio_type_var = StringVar()
audio_type_var.set("Select Audio")

serial_data_var = StringVar()
serial_data_var.set("Serial Data: ")

# Function to update audio type and start playing audio
def start_audio(audio_type):
    audio_type_var.set(f"Selected Audio: {audio_type.capitalize()}")

    # Stop playing previous audio
    stop_audio()

    # Play new audio
    if audio_type == "sine":
        play_sine(100, 2)  # Adjust the frequency and duration as needed
    elif audio_type == "random":
        play_white_noise(2)  # Adjust the duration as needed

# Create buttons for sine and random noise
sine_button = Button(root, text="Play Sine", command=lambda: start_audio("sine"), font=("Helvetica", 16, "bold"))
sine_button.pack(pady=10)

random_button = Button(root, text="Play Random Noise", command=lambda: start_audio("random"), font=("Helvetica", 16, "bold"))
random_button.pack(pady=10)

# Label to display selected audio type
audio_type_label = Label(root, textvariable=audio_type_var, font=("Helvetica", 18, "bold"))
audio_type_label.pack()

# Label to display serial data
serial_data_label = Label(root, textvariable=serial_data_var, font=("Helvetica", 10, "bold"))
serial_data_label.pack()

# Start the thread to handle serial data
thread = Thread(target=button_click, args=(audio_type_var, serial_data_var))
thread.start()

# Start the GUI main loop
root.mainloop()
