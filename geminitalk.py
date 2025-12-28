'''
from google import genai

client = genai.Client(api_key="GEMINI_API_KEY")

# Start a chat session
chat = client.chats.create(model="gemini-1.5-flash")

print("--- Chat Started (Type 'quit' to stop) ---")

while True:
    user_input = input("You: ")
    if user_input.lower() in ["quit", "exit", "bye"]:
        break

    # Send message and get response
    response = chat.send_message(user_input)
    print(f"Gemini: {response.text}\n")

'''

from google import genai

# Use your actual API Key here
client = genai.Client(api_key="GEMINI_API_KEY")

# Start the chat
chat = client.chats.create(model="gemini-2.5-flash-lite")

print("--- Robot Online ---")

while True:
    user_input = input("You: ")
    if user_input.lower() in ["quit", "exit"]:
        break

    try:
        # Note: Ensure the model name matches exactly "gemini-1.5-flash"
        response = chat.send_message(user_input)
        print(f"Robot: {response.text}\n")
    except Exception as e:
        print(f"Error: {e}")
'''
from google import genai

client = genai.Client(api_key="GEMINI_API_KEY")

print("Listing available models...")
for model in client.models.list():
    print(f"Model Name: {model.name} | Supported Methods: {model.supported_actions}")
'''