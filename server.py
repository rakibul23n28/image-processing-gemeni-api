from email.mime import audio
import os
import tempfile
import pyttsx3
import speech_recognition as sr
from flask import Flask, request, send_file
from dotenv import load_dotenv
from google.genai import types
from google import genai
from pydub import AudioSegment

# Load environment variables from .env
load_dotenv()

app = Flask(__name__)
UPLOAD_FOLDER = './uploads'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

# Load Gemini API key
api_key = os.getenv("GEMINI_API_KEY")
if not api_key:
    raise ValueError("GEMINI_API_KEY not set in .env")

# Setup Gemini client and recognizer
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

    # Save image
    image_path = os.path.join(UPLOAD_FOLDER, image_file.filename)
    image_file.save(image_path)
    image = types.Part.from_bytes(data=open(image_path, 'rb').read(), mime_type='image/jpeg')

    try:
        # Save audio to temporary file for transcription
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
            return "Could not understand audio", 400
        except sr.RequestError as e:
            print("Speech recognition error:", e)
            return "Speech recognition failed", 500

        # Generate Gemini response
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

        # Convert Gemini response text to speech with pyttsx3
        engine = pyttsx3.init()
        engine.setProperty('rate', 150)  # Adjust speech rate if needed

        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tf:
            wav_path = tf.name

        engine.save_to_file(gemini_text, wav_path)
        engine.runAndWait()

        # Force mono 16-bit PCM @ 16kHz using pydub
        audio = AudioSegment.from_wav(wav_path)
        audio = audio.set_channels(1).set_sample_width(2).set_frame_rate(16000)
        
        print(f"Channels: {audio.channels}")
        print(f"Sample Width: {audio.sample_width * 8} bits")
        print(f"Frame Rate: {audio.frame_rate} Hz")


        mono_path = wav_path.replace(".wav", "_mono.wav")
        audio.export(mono_path, format="wav")

        print(f"Mono WAV file saved at: {mono_path}")
        return send_file(mono_path, mimetype='audio/wav', as_attachment=True, download_name="response.wav")

    except Exception as e:
        print("Server error:", e)
        return "Server error", 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=80)
