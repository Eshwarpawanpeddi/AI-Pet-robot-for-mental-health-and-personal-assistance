# Pet Robot Mobile App

A Flutter-based mobile application for the AI Pet Robot mental health companion.

## Features

- 🤖 Real-time robot control via WebSocket
- 🧠 Mental health support features
  - Mood tracking
  - Positive affirmations
  - Breathing exercises
  - Crisis resources
- 🎙️ Voice interaction
- 😊 Animated robot face
- 📊 Emotion monitoring
- 🔒 Privacy-first design

## Requirements

- Flutter SDK 3.0+
- Android Studio / Xcode
- Android 6.0+ / iOS 12.0+

## Setup

### 1. Install Flutter

```bash
# Download Flutter SDK
git clone https://github.com/flutter/flutter.git -b stable
export PATH="$PATH:`pwd`/flutter/bin"

# Verify installation
flutter doctor
```

### 2. Install Dependencies

```bash
cd mobile_app
flutter pub get
```

### 3. Configure Server Connection

Edit `lib/config/app_config.dart`:
```dart
static const String serverUrl = 'ws://YOUR_SERVER_IP:8000/ws/control';
```

### 4. Run the App

```bash
# Run on connected device
flutter run

# Build APK
flutter build apk

# Build for iOS
flutter build ios
```

## Project Structure

```
mobile_app/
├── lib/
│   ├── main.dart                 # App entry point
│   ├── config/
│   │   └── app_config.dart       # Configuration
│   ├── models/
│   │   ├── robot_state.dart      # Robot state model
│   │   └── mood_log.dart         # Mood tracking model
│   ├── services/
│   │   ├── websocket_service.dart # WebSocket connection
│   │   └── api_service.dart       # HTTP API calls
│   ├── providers/
│   │   └── robot_provider.dart    # State management
│   ├── screens/
│   │   ├── home_screen.dart       # Main screen
│   │   ├── control_screen.dart    # Robot control
│   │   ├── mood_screen.dart       # Mood tracking
│   │   └── settings_screen.dart   # Settings
│   └── widgets/
│       ├── robot_face.dart        # Animated face
│       ├── control_pad.dart       # Movement controls
│       └── emotion_card.dart      # Emotion display
├── android/                       # Android config
├── ios/                          # iOS config
└── pubspec.yaml                  # Dependencies
```

## Usage

### Connect to Robot

1. Open the app
2. Go to Settings
3. Enter your server IP address
4. Tap "Connect"

### Control Robot

- Use on-screen joystick for movement
- Tap emotion buttons to change robot's mood
- Use voice button for voice commands

### Mental Health Features

- **Mood Log**: Track your daily emotions
- **Affirmations**: Get positive encouragement
- **Breathing**: Guided breathing exercises
- **Crisis Help**: Access emergency resources

## Building for Release

### Android

```bash
# Generate signing key
keytool -genkey -v -keystore ~/key.jks -keyalg RSA -keysize 2048 -validity 10000 -alias key

# Build release APK
flutter build apk --release

# Build App Bundle
flutter build appbundle --release
```

### iOS

```bash
# Build for iOS
flutter build ios --release

# Or open in Xcode
open ios/Runner.xcworkspace
```

## Permissions

The app requires the following permissions:
- Internet access (for server connection)
- Microphone (for voice commands)
- Storage (for mood logs)

## Privacy

- All data stored locally on device
- No third-party analytics
- User consent required for data collection
- Compliant with HIPAA guidelines for mental health apps

## Support

For issues or questions:
- GitHub: [Repository Issues](https://github.com/Eshwarpawanpeddi/AI-Pet-robot-for-mental-health-and-personal-assistance/issues)
- Documentation: See main README.md

## License

MIT License - See LICENSE file
