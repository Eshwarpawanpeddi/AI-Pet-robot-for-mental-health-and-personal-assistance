import os
from pathlib import Path
from dotenv import load_dotenv
from google import genai

# Load env
script_dir = Path(__file__).parent
load_dotenv(script_dir / ".env")
api_key = os.getenv("GEMINI_API_KEY")

client = genai.Client(api_key=api_key)

print("--- Listing Available Models ---")
try:
    # This will print every model your key can access
    for model in client.models.list():
        print(f"Name: {model.name}")
except Exception as e:
    print(f"Error listing models: {e}")