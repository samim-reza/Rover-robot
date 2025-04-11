from flask import Flask, send_from_directory
from flask_cors import CORS
import os

app = Flask(__name__)
CORS(app)  # Enable CORS globally

# Folder containing STL files
STL_FOLDER = os.path.join(os.getcwd(), "stl_files")

@app.route("/stl/rover")  # Static route for specific file
def serve_stl():
    try:
        # Serve the STL file from the stl_files folder
        return send_from_directory(STL_FOLDER, "letest_rover.STL")
    except Exception as e:
        print("Error:", e)
        return f"File not found", 500

if __name__ == "__main__":
    app.run(debug=True, port=5000)
