import os
from flask import Flask, request
from dotenv import load_dotenv
from google.genai import types
from google import genai
import speech_recognition as sr
import tempfile

load_dotenv()

app = Flask(__name__)
# IzaSyAB7mcHdimWfhJJHEMXEDR8doQjOaU07hM


# Create upload directory if not exists
UPLOAD_FOLDER = './uploads'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

api_key = os.getenv("GEMINI_API_KEY")
if not api_key:
    raise ValueError("GEMINI_API_KEY not set in environment")

client = genai.Client(api_key=api_key)
recognizer = sr.Recognizer()

@app.route('/upload', methods=['POST'])
def upload():
    if 'audio' not in request.files or 'image' not in request.files:
        return "Please upload both 'audio' and 'image' files", 400

    audio_file = request.files['audio']
    image_file = request.files['image']

    print(f"Received audio file: {audio_file.filename}, size: {audio_file.content_length}")
    print(f"Received image file: {image_file.filename}, size: {image_file.content_length}")

    # Save image to disk to verify capture
    image_path = os.path.join(UPLOAD_FOLDER, image_file.filename)
    image_file.save(image_path)
    print(f"Image saved to {image_path}")

    try:
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=True) as temp_audio_file:
            audio_file.save(temp_audio_file.name)
            temp_audio_file.flush()

            with sr.AudioFile(temp_audio_file.name) as source:
                audio = recognizer.record(source)

            try:
                transcript = recognizer.recognize_google(audio, language='en-US')
                print("Transcribed text:", transcript)
            except sr.UnknownValueError:
                transcript = ""
                print("Speech recognition could not understand audio")
            except sr.RequestError as e:
                print("Speech recognition error:", e)
                return "Speech recognition failed", 500

        response = client.models.generate_content(
            model="gemini-2.0-flash",
            contents=[
                types.Part.from_bytes(data=open(image_path, 'rb').read(), mime_type='image/jpeg'),
                f"Please analyze and improve the following transcription:\n{transcript}"
            ]
        )

        gemini_text = response.text if hasattr(response, 'text') else str(response)
        print("Gemini API response:", gemini_text)

        return gemini_text, 200

    except Exception as e:
        print("Server error:", e)
        return "Server error", 500


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
