import os
from pathlib import Path
from dotenv import load_dotenv
from google import genai
from google.genai import types

script_dir = Path(__file__).parent
load_dotenv(script_dir / ".env")

api_key = os.getenv("GEMINI_API_KEY")

if not api_key:
    print("Error: GEMINI_API_KEY not found. Check your .env file!")
    exit()

client = genai.Client(api_key=api_key)

system_instruction = """
You are a helpful fitness and diet assistant. 
Your goal is to generate a specific meal plan, but first, you must interview the user to get the following details. 
Do not generate the plan until you have all the information. Ask one question at a time to keep it conversational.

Required Information:
1. Current Location (City/State)
2. Meal type (Breakfast, Lunch, or Dinner)
3. Calorie Goal for this meal
4. Preference: Cook at home or Buy/Order out?
5. If Cooking: What ingredients do they currently have?
6. If Buying: What is their budget or restaurant preference?

Once you have these details, generate a concise, healthy plan.
"""

try:
    chat = client.chats.create(
        model="gemini-2.0-flash",
        config=types.GenerateContentConfig(
            system_instruction=system_instruction,
            temperature=0.7
        )
    )

    print("--- Fitness Assistant Online ---")
    
    response = chat.send_message("Hello, I am ready to help you plan your meal. Please ask me to start.")
    print(f"Gemini: {response.text}\n")

    while True:
        user_input = input("You: ")
        if user_input.lower() in ["quit", "exit"]:
            break

        try:
            response = chat.send_message(user_input)
            print(f"Gemini: {response.text}\n")
        except Exception as e:
            print(f"Error during send: {e}")

except Exception as e:
    print(f"Error initializing chat: {e}")