import asyncio
import base64
from google.generativeai.types import LiveStreamingConfig
import google.generativeai as genai

class GeminiLiveIntegration:
    def __init__(self, api_key: str):
        genai.configure(api_key=api_key)
        self.session = None
        self.model_name = "gemini-2.5-flash-native-audio-preview-12-2025"
        
    async def initialize_session(self):
        """Initialize Gemini Live API session"""
        try:
            model = genai.GenerativeModel(
                model_name=self.model_name,
                system_instruction="""You are a friendly, emotionally intelligent pet robot companion.
                Keep responses concise and charming.
                Express emotions in your tone.
                Respond to commands and engage in meaningful conversation.""",
                generation_config=genai.types.GenerationConfig(
                    response_modalities=["audio"],
                    realtime_input_config=genai.types.RealtimeInputConfig(
                        automatic_activity_detection={"disabled": False}
                    )
                )
            )
            self.session = await model.aio.start_chat()
            print("Gemini Live session initialized")
            return True
        except Exception as e:
            print(f"Failed to initialize Gemini session: {e}")
            return False
    
    async def send_audio(self, audio_bytes: bytes, sample_rate: int = 16000) -> str:
        """Send audio to Gemini Live API and get response"""
        try:
            # Send audio to model
            await self.session.send_message(
                genai.types.Blob(
                    mime_type=f"audio/pcm;rate={sample_rate}",
                    data=audio_bytes
                )
            )
            
            # Receive response
            response_text = ""
            response_audio = None
            
            async for response in self.session.receive():
                if hasattr(response, 'text') and response.text:
                    response_text += response.text
                if hasattr(response, 'data') and response.data:
                    response_audio = response.data
            
            return {
                'text': response_text,
                'audio': response_audio
            }
        except Exception as e:
            print(f"Error sending audio to Gemini: {e}")
            return None
    
    async def close(self):
        """Close Gemini session"""
        if self.session:
            await self.session.close()