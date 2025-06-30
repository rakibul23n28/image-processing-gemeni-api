from flask import Flask, send_from_directory

app = Flask(__name__)

# Route for home page
@app.route("/")
def home():
    return "ESP32 Audio Server is running."

# Route to serve audio from 'upload/' folder
@app.route("/audio")
def serve_audio():
    filename = "temp.mp3"  # Change this to your audio file name
    return send_from_directory("upload", filename, mimetype="audio/mpeg")

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
