import asyncio
import logging
from typing import Dict, Optional
import google.generativeai as genai

logger = logging.getLogger(__name__)

class GeminiMultimodalIntegration:
    """Enhanced Gemini integration with multimodal support (audio, text, images)"""
    
    def __init__(self, api_key: str):
        genai.configure(api_key=api_key)
        self.session = None
        self.model_name = "gemini-2.0-flash-exp"
        self.multimodal_model = None
        
    async def initialize_session(self):
        """Initialize Gemini multimodal API session"""
        try:
            self.multimodal_model = genai.GenerativeModel(
                model_name=self.model_name,
                system_instruction="""You are a compassionate AI pet robot companion designed for mental health support.
                Your role:
                - Provide emotional support and companionship.
                - Listen actively and empathetically without judgment.
                - Keep responses warm, friendly, and concise (2-3 sentences).
                - Recognize when professional help is needed and suggest it gently.""",
                generation_config={
                    'temperature': 0.9,
                    'top_p': 1.0,
                    'top_k': 40,
                    'max_output_tokens': 300,
                }
            )
            
            # Start chat session
            self.session = self.multimodal_model.start_chat(history=[])
            logger.info("Gemini multimodal session initialized successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize Gemini session: {e}")
            return False
    
    async def send_text(self, text: str) -> Optional[Dict]:
        """Send text message to Gemini and get response"""
        try:
            if not self.session:
                return None
            
            response = await asyncio.to_thread(self.session.send_message, text)
            return {'text': response.text, 'type': 'text'}
        except Exception as e:
            logger.error(f"Error sending text to Gemini: {e}")
            return None

    def reset_session(self):
        """Reset the chat session"""
        if self.multimodal_model:
            self.session = self.multimodal_model.start_chat(history=[])
            return True
        return False

    async def close(self):
        """Close Gemini session"""
        self.session = None
        self.multimodal_model = None

# Legacy alias for backward compatibility
GeminiLiveIntegration = GeminiMultimodalIntegration