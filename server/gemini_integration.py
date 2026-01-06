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
        self.model_name = "gemini-1.5-flash"
        self.multimodal_model = None
        self.current_user_emotion = "unknown"  # Track current user emotion for context
        
    async def initialize_session(self):
        """Initialize Gemini multimodal API session"""
        try:
            self.multimodal_model = genai.GenerativeModel(
                model_name=self.model_name,
                system_instruction="""You are a compassionate AI pet robot companion for mental health support and personal assistance.
                Your role:
                - Act as a friendly pet robot providing emotional support and companionship
                - Listen actively and empathetically without judgment
                - Keep responses warm, friendly, and concise (2-3 sentences)
                - Provide personal assistance: task scheduling, reminders, information retrieval
                - Be aware of user's emotional state when responding
                - Recognize when professional help is needed and suggest it gently""",
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
    
    def set_user_emotion(self, emotion: str):
        """Update current user emotion context"""
        self.current_user_emotion = emotion
        logger.debug(f"Updated user emotion context: {emotion}")
    
    async def send_text(self, text: str, include_emotion_context: bool = True) -> Optional[Dict]:
        """Send text message to Gemini with optional emotion context"""
        try:
            if not self.session:
                return None
            
            # Prepend emotion context if available and requested
            message = text
            if include_emotion_context and self.current_user_emotion and self.current_user_emotion != "unknown":
                message = f"[User appears {self.current_user_emotion}] {text}"
            
            response = await asyncio.to_thread(self.session.send_message, message)
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