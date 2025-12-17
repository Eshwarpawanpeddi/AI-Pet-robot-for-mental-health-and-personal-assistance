class AppConfig {
  // Server Configuration
  static const String defaultServerUrl = '192.168.1.100';
  static const int defaultServerPort = 8000;
  static const String websocketPath = '/ws/control';
  
  // Get WebSocket URL
  static String getWebSocketUrl(String? customUrl) {
    final url = customUrl ?? defaultServerUrl;
    return 'ws://$url:$defaultServerPort$websocketPath';
  }
  
  // Get HTTP API URL
  static String getApiUrl(String? customUrl) {
    final url = customUrl ?? defaultServerUrl;
    return 'http://$url:$defaultServerPort/api';
  }
  
  // App Configuration
  static const String appName = 'AI Pet Robot';
  static const String appVersion = '1.0.0';
  
  // Mental Health Resources
  static const Map<String, String> crisisResources = {
    'USA Suicide Prevention': '988',
    'Crisis Text Line': 'Text HOME to 741741',
    'International': 'https://www.iasp.info/resources/Crisis_Centres/',
  };
  
  // Emotion colors
  static const Map<String, Color> emotionColors = {
    'happy': Color(0xFFFFD700),      // Gold
    'sad': Color(0xFF4A90E2),        // Blue
    'anxious': Color(0xFFFFA500),    // Orange
    'stressed': Color(0xFFFF6B6B),   // Red
    'tired': Color(0xFF9370DB),      // Purple
    'angry': Color(0xFFDC143C),      // Crimson
    'neutral': Color(0xFF64C8FF),    // Cyan
    'crisis': Color(0xFFFF0000),     // Red
  };
  
  // Storage Keys
  static const String keyServerUrl = 'server_url';
  static const String keyMoodLogs = 'mood_logs';
  static const String keyUserConsent = 'user_consent';
}
