import os
from pathlib import Path
from dotenv import load_dotenv
from google import genai

# --- 1. Load the Environment Variables Correctly ---
# This ensures Python finds .env even if you run the script from a different folder
script_dir = Path(__file__).parent
load_dotenv(script_dir / ".env")

# --- 2. Get the Key ---
api_key = os.getenv("GEMINI_API_KEY")

if not api_key:
    print("Error: GEMINI_API_KEY not found. Check your .env file!")
    exit()

# --- 3. Initialize Client ---
# We pass the variable 'api_key' (no quotes), not the string name
client = genai.Client(api_key=api_key)

# Start the chat
# Note: "gemini-2.5-flash-lite" might not exist yet.
# If this crashes, try "gemini-1.5-flash" or "gemini-2.0-flash-exp"
try:
    chat = client.chats.create(model="gemini-2.5-flash") # Safest bet for now
    print("--- Robot Online ---")

    while True:
        user_input = input("You: ")
        if user_input.lower() in ["quit", "exit"]:
            break

        try:
            response = chat.send_message(user_input)
            print(f"Robot: {response.text}\n")
        except Exception as e:
            print(f"Error during send: {e}")

except Exception as e:
    print(f"Error initializing chat: {e}")