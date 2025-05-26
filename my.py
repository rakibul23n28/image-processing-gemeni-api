import tkinter as tk
from tkinter import ttk
import cv2
from PIL import Image, ImageTk
from google.genai import types
from google import genai
import numpy as np
import os
from datetime import datetime
from io import BytesIO
import pyttsx3
import speech_recognition as sr
import threading
from gtts import gTTS
import playsound
import tempfile
from dotenv import load_dotenv

load_dotenv()
# Get the API key
api_key = os.getenv("GEMINI_API_KEY")

# === CONFIGURE GEMINI API ===
client = genai.Client(api_key=api_key)

# === GLOBAL VARIABLES ===
cap = None
video_label = None
last_response_text = ""
selected_language = "English"

# === TTS ENGINE SETUP ===
tts_engine = pyttsx3.init()
tts_engine.setProperty('rate', 150)

def speak_text(text):
    global selected_language

    if selected_language == "Bangla":
        try:
            tts = gTTS(text=text, lang='bn')
            with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as fp:
                temp_path = fp.name
                tts.save(temp_path)

            playsound.playsound(temp_path)
            os.remove(temp_path)  # Clean up after playback
        except Exception as e:
            print(f"Bangla TTS error: {e}")
            tts_engine.say("Could not speak Bangla.")
            tts_engine.runAndWait()
    else:
        tts_engine.say(text)
        tts_engine.runAndWait()


# === START VIDEO STREAM ===
def start_video():
    global cap
    cap = cv2.VideoCapture(0)
    update_frame()

def update_frame():
    global cap, video_label
    if cap is not None and cap.isOpened():
        ret, frame = cap.read()
        if ret:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb)
            imgtk = ImageTk.PhotoImage(image=img)
            video_label.imgtk = imgtk
            video_label.configure(image=imgtk)
    video_label.after(10, update_frame)

# === IMAGE CAPTURE ===
def capture_image():
    global cap
    if cap is not None and cap.isOpened():
        ret, frame = cap.read()
        if ret:
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pil_img = Image.fromarray(rgb_frame)

            save_dir = "captured_images"
            os.makedirs(save_dir, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            image_path = os.path.join(save_dir, f"capture_{timestamp}.jpg")
            pil_img.save(image_path)

            cv2.imshow("Captured Image", frame)
            cv2.waitKey(1000)
            cv2.destroyAllWindows()

            send_to_gemini(pil_img, timestamp)

# === GEMINI API CALL ===
def send_to_gemini(pil_img, timestamp):
    global last_response_text, selected_language

    try:
        img_byte_arr = BytesIO()
        pil_img.save(img_byte_arr, format="JPEG")
        img_bytes = img_byte_arr.getvalue()

        language_prompts = {
            "English": "I am a blind person. Please describe the scene briefly in English.",
            "Spanish": "Soy una persona ciega. Describe brevemente la escena en español.",
            "Hindi": "मैं एक नेत्रहीन व्यक्ति हूँ। कृपया दृश्य का संक्षिप्त वर्णन हिंदी में करें।",
            "French": "Je suis une personne aveugle. Veuillez décrire brièvement la scène en français.",
            "German": "Ich bin eine blinde Person. Bitte beschreiben Sie die Szene kurz auf Deutsch.",
            "Bangla": "আমি একজন অন্ধ ব্যক্তি। দয়া করে দৃশ্যটি সংক্ষেপে বাংলায় বর্ণনা করুন।"
        }

        prompt = language_prompts.get(selected_language, language_prompts["English"])

        response = client.models.generate_content(
            model='gemini-2.0-flash',
            contents=[
                types.Part.from_bytes(data=img_bytes, mime_type='image/jpeg'),
                prompt
            ]
        )

        result_text = response.text
        last_response_text = result_text
        show_response(result_text)
        save_response(timestamp, result_text)
        speak_text(result_text)

    except Exception as e:
        error_msg = f"Error: {e}"
        show_response(error_msg)
        speak_text("Sorry, an error occurred while describing the image.")

# === DISPLAY RESPONSE ===
def show_response(text):
    response_window = tk.Toplevel(root)
    response_window.title("Gemini Response")
    tk.Label(response_window, text=text, wraplength=400, justify="left").pack(padx=10, pady=10)

# === SAVE TO FILE ===
def save_response(timestamp, text):
    save_dir = "captured_images"
    response_path = os.path.join(save_dir, f"response_{timestamp}.txt")
    with open(response_path, "w", encoding="utf-8") as f:
        f.write(text)

def speak_last_response():
    if last_response_text:
        speak_text(last_response_text)
    else:
        speak_text("No description available yet. Please capture an image first.")

# === VOICE COMMAND LISTENER ===
def listen_for_command():
    recognizer = sr.Recognizer()
    mic = sr.Microphone()
    speak_text("Voice command mode activated. Say 'capture' to take a picture.")

    while True:
        try:
            with mic as source:
                recognizer.adjust_for_ambient_noise(source)
                audio = recognizer.listen(source, timeout=5)

            command = recognizer.recognize_google(audio).lower()
            print("Voice command received:", command)

            if "capture" in command:
                speak_text("Capturing image.")
                capture_image()

        except sr.WaitTimeoutError:
            continue
        except sr.UnknownValueError:
            speak_text("Sorry, I did not understand. Please try again.")
        except Exception as e:
            print(f"Voice command error: {e}")
            speak_text("Error in voice command system.")

# === START VOICE LISTENING IN THREAD ===
def start_voice_thread():
    threading.Thread(target=listen_for_command, daemon=True).start()

# === LANGUAGE DROPDOWN ===
def on_language_change(event):
    global selected_language
    selected_language = language_var.get()

# === GUI SETUP ===
root = tk.Tk()
root.title("Live Camera & Gemini AI for Blind Users")

tk.Label(root, text="Live Camera Feed:").pack()

video_label = tk.Label(root)
video_label.pack()

tk.Button(root, text="Capture Image", command=capture_image).pack(pady=5)
tk.Button(root, text="Replay Description", command=speak_last_response).pack(pady=5)
tk.Button(root, text="Start Voice Command Mode", command=start_voice_thread).pack(pady=5)

# Language dropdown
tk.Label(root, text="Select Language for Description:").pack(pady=5)
language_var = tk.StringVar(value="English")
languages = ["English", "Spanish", "Hindi", "French", "German", "Bangla"]
language_menu = ttk.Combobox(root, textvariable=language_var, values=languages, state="readonly")
language_menu.pack()
language_menu.bind("<<ComboboxSelected>>", on_language_change)

# === START VIDEO STREAM ===
start_video()

# === MAIN LOOP ===
root.mainloop()
