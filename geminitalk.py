import os
from pathlib import Path
from dotenv import load_dotenv
from google import genai

script_dir = Path(__file__).parent
load_dotenv(script_dir / ".env")

api_key = os.getenv("GEMINI_API_KEY")

if not api_key:
    print("Error: GEMINI_API_KEY not found. Check your .env file!")
    exit()

client = genai.Client(api_key=api_key)

age = 25
food_type = "vegetarian"
cal_req = 2200
breakfast_lunch_dinner = "breakfast"
location = "St. Louis"
cook_or_buy = "buy"
ingredients = "none because buying food"

try:
    chat = client.chats.create(model="gemini-2.0-flash") 
    print("--- Robot Online ---")

    initial_prompt = (
        f"You are a fitness assistant. User details: Age {age}, "
        f"{food_type} diet, {cal_req} calories, for {breakfast_lunch_dinner}. "
        f"Location: {location}. Preference: {cook_or_buy}. "
        f"Ingredients available: {ingredients}. Generate a diet plan with only the food names and nothing else."
    )

    response = chat.send_message(initial_prompt)
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