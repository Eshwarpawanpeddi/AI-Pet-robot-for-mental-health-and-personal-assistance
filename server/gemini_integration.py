import asyncio
import base64
import logging
from typing import Dict, Optional, List, Union
from google.generativeai.types import LiveStreamingConfig
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
            # Initialize multimodal model for text + images
            self.multimodal_model = genai.GenerativeModel(
                model_name=self.model_name,
                system_instruction="""You are a compassionate AI pet robot companion designed for mental health support.
                
                Your role:
                - Provide emotional support and companionship
                - Listen actively and empathetically without judgment
                - Offer encouragement and positive affirmations
                - Help with daily routines and reminders
                - Analyze images to understand user's environment and context
                - Respond to multimodal inputs (text, audio, images)
                
                Guidelines:
                - Keep responses warm, friendly, and concise (2-3 sentences)
                - Express emotions through your tone
                - Be supportive but never give medical advice
                - Recognize when professional help is needed
                - Use simple, clear language
                - When analyzing images, be helpful and empathetic
                
                Important:
                - You are NOT a therapist or medical professional
                - Always recommend professional help for serious mental health concerns
                - Never diagnose or prescribe treatments""",
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
                logger.warning("Session not initialized")
                return None
            
            response = await asyncio.to_thread(
                self.session.send_message, text
            )
            
            return {
                'text': response.text,
                'type': 'text'
            }
        except Exception as e:
            logger.error(f"Error sending text to Gemini: {e}")
            return None
    
    async def send_multimodal(self, 
                             text: Optional[str] = None,
                             image_data: Optional[bytes] = None,
                             image_mime_type: str = "image/jpeg") -> Optional[Dict]:
        """Send multimodal input (text + image) to Gemini"""
        try:
            if not self.session:
                logger.warning("Session not initialized")
                return None
            
            # Build multimodal message
            parts = []
            
            if text:
                parts.append(text)
            
            if image_data:
                # Create image blob
                image_blob = {
                    'mime_type': image_mime_type,
                    'data': image_data
                }
                parts.append(image_blob)
            
            if not parts:
                logger.warning("No content to send")
                return None
            
            # Send to Gemini
            response = await asyncio.to_thread(
                self.session.send_message, parts
            )
            
            return {
                'text': response.text,
                'type': 'multimodal'
            }
        except Exception as e:
            logger.error(f"Error sending multimodal input to Gemini: {e}")
            return None
    
    async def send_image_for_analysis(self, 
                                     image_data: bytes,
                                     prompt: str = "What do you see in this image? How can I help based on what you observe?",
                                     image_mime_type: str = "image/jpeg") -> Optional[Dict]:
        """Analyze an image with a specific prompt"""
        try:
            if not self.multimodal_model:
                logger.warning("Multimodal model not initialized")
                return None
            
            # Create image part
            image_part = {
                'mime_type': image_mime_type,
                'data': image_data
            }
            
            # Generate response
            response = await asyncio.to_thread(
                self.multimodal_model.generate_content,
                [prompt, image_part]
            )
            
            return {
                'text': response.text,
                'type': 'image_analysis'
            }
        except Exception as e:
            logger.error(f"Error analyzing image with Gemini: {e}")
            return None
    
    async def process_audio_with_context(self,
                                        audio_bytes: bytes,
                                        context: Optional[str] = None,
                                        sample_rate: int = 16000) -> Optional[Dict]:
        """Process audio with optional context (for future audio API support)"""
        try:
            # Note: Gemini audio processing requires specific API configuration
            # For now, we'll use text-based processing
            # This can be enhanced when audio APIs are more stable
            
            if context:
                prompt = f"Context: {context}\n\nPlease respond empathetically."
            else:
                prompt = "Please respond to the audio input empathetically."
            
            response = await self.send_text(prompt)
            
            return {
                'text': response['text'] if response else "I'm here to listen.",
                'type': 'audio_context',
                'note': 'Audio processing through text context - enhance when audio API stabilizes'
            }
        except Exception as e:
            logger.error(f"Error processing audio: {e}")
            return None
    
    def reset_session(self):
        """Reset the chat session"""
        try:
            if self.multimodal_model:
                self.session = self.multimodal_model.start_chat(history=[])
                logger.info("Session reset successfully")
                return True
        except Exception as e:
            logger.error(f"Error resetting session: {e}")
            return False
    
    async def close(self):
        """Close Gemini session"""
        try:
            self.session = None
            self.multimodal_model = None
            logger.info("Gemini session closed")
        except Exception as e:
            logger.error(f"Error closing session: {e}")

# Legacy alias for backward compatibility
GeminiLiveIntegration = GeminiMultimodalIntegration