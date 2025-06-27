import os
from flask import Flask, request, send_file
from dotenv import load_dotenv
from google.genai import types
from google import genai
import speech_recognition as sr
import tempfile
from gtts import gTTS
from pydub import AudioSegment
from io import BytesIO

# Load .env variables
load_dotenv()

app = Flask(__name__)

# Upload directory
UPLOAD_FOLDER = './uploads'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

# Load Gemini API key
api_key = os.getenv("GEMINI_API_KEY")
if not api_key:
    raise ValueError("GEMINI_API_KEY not set in .env")

# Gemini and speech recognizer setup
client = genai.Client(api_key=api_key)
recognizer = sr.Recognizer()

@app.route('/upload', methods=['POST'])
def upload():
    if 'audio' not in request.files or 'image' not in request.files:
        return "Please upload both 'audio' and 'image' files", 400

    audio_file = request.files['audio']
    image_file = request.files['image']

    print(f"Received audio file: {audio_file.filename}")
    print(f"Received image file: {image_file.filename}")

    # Save the image to disk
    image_path = os.path.join(UPLOAD_FOLDER, image_file.filename)
    image_file.save(image_path)
    image = types.Part.from_bytes(data=open(image_path, 'rb').read(), mime_type='image/jpeg')
    print("Image loaded into Gemini Part object")
    print(f"Image saved to {image_path}")

    try:
        # Save audio to temp file for recognition
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_audio_file:
            temp_audio_path = temp_audio_file.name
            audio_file.save(temp_audio_path)

        with sr.AudioFile(temp_audio_path) as source:
            audio = recognizer.record(source)

        os.remove(temp_audio_path)

        # Transcribe audio
        try:
            transcript = recognizer.recognize_google(audio, language='en-US')
            print("Transcribed text:", transcript)
        except sr.UnknownValueError:
            transcript = ""
            print("Speech recognition could not understand audio")
            return "Could not understand audio", 400
        except sr.RequestError as e:
            print("Speech recognition error:", e)
            return "Speech recognition failed", 500

        response = client.models.generate_content(
            model="gemini-2.5-flash",
            contents=[
            f"""
            You are assisting a blind person.

            ONLY do ONE task per request. Choose the task based on this transcript:

            '{transcript}'

            Supported tasks (choose ONE):

            1. Emotion detection → Respond with: Gender, Age, Emotion, Description (in 10 words)
            2. Text recognition → Respond with: Exact text found in the image
            3. Image captioning → Respond with: A scene description (based on the transcript)

            If no command is clearly detected, just describe the image in less than 50 words.
            """,
                    image
                ]
            )


        gemini_text = response.text if hasattr(response, 'text') else str(response)
        print("Gemini API response:", gemini_text)

        # Convert text to MP3 using gTTS
        tts = gTTS(text=gemini_text, lang='en')
        temp_mp3_path = os.path.join(UPLOAD_FOLDER, "temp.mp3")
        tts.save(temp_mp3_path)

        # Convert MP3 to WAV in-memory
        sound = AudioSegment.from_mp3(temp_mp3_path)
        wav_io = BytesIO()
        sound.export(wav_io, format="wav")
        wav_io.seek(0)

        print(f"WAV audio length: {len(wav_io.getvalue())} bytes")

        # Return as attachment
        return send_file(wav_io, mimetype='audio/wav', as_attachment=True, download_name="response.wav")

    except Exception as e:
        print("Server error:", e)
        return "Server error", 500



if __name__ == '__main__':
    app.run(host='0.0.0.0', port=80)
