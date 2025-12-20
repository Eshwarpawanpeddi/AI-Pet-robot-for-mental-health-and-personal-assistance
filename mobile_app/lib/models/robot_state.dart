class RobotState {
  final String emotion;
  final bool isSpeaking;
  final bool isListening;
  final int batteryLevel;
  final SensorData? sensorData;
  final DateTime timestamp;
  final String? controlMode;
  final bool? espConnected;
  final bool? raspberryPiConnected;
  final String? lastTranscript;

  RobotState({
    required this.emotion,
    required this.isSpeaking,
    required this.isListening,
    required this.batteryLevel,
    this.sensorData,
    DateTime? timestamp,
    this.controlMode,
    this.espConnected,
    this.raspberryPiConnected,
    this.lastTranscript,
  }) : timestamp = timestamp ?? DateTime.now();

  factory RobotState.fromJson(Map<String, dynamic> json) {
    return RobotState(
      emotion: json['emotion'] ?? 'neutral',
      isSpeaking: json['is_speaking'] ?? false,
      isListening: json['is_listening'] ?? false,
      batteryLevel: json['battery_level'] ?? 100,
      sensorData: json['sensor_data'] != null
          ? SensorData.fromJson(json['sensor_data'])
          : null,
      controlMode: json['control_mode'],
      espConnected: json['esp_connected'],
      raspberryPiConnected: json['raspberry_pi_connected'],
      lastTranscript: json['last_transcript'],
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'emotion': emotion,
      'is_speaking': isSpeaking,
      'is_listening': isListening,
      'battery_level': batteryLevel,
      'sensor_data': sensorData?.toJson(),
      'control_mode': controlMode,
      'esp_connected': espConnected,
      'raspberry_pi_connected': raspberryPiConnected,
      'last_transcript': lastTranscript,
    };
  }

  RobotState copyWith({
    String? emotion,
    bool? isSpeaking,
    bool? isListening,
    int? batteryLevel,
    SensorData? sensorData,
    String? controlMode,
    bool? espConnected,
    bool? raspberryPiConnected,
    String? lastTranscript,
  }) {
    return RobotState(
      emotion: emotion ?? this.emotion,
      isSpeaking: isSpeaking ?? this.isSpeaking,
      isListening: isListening ?? this.isListening,
      batteryLevel: batteryLevel ?? this.batteryLevel,
      sensorData: sensorData ?? this.sensorData,
      controlMode: controlMode ?? this.controlMode,
      espConnected: espConnected ?? this.espConnected,
      raspberryPiConnected: raspberryPiConnected ?? this.raspberryPiConnected,
      lastTranscript: lastTranscript ?? this.lastTranscript,
    );
  }
}

class SensorData {
  final bool touchDetected;
  final double distance;
  final double temperature;

  SensorData({
    required this.touchDetected,
    required this.distance,
    required this.temperature,
  });

  factory SensorData.fromJson(Map<String, dynamic> json) {
    return SensorData(
      touchDetected: json['touch_detected'] ?? false,
      distance: (json['distance'] ?? 0).toDouble(),
      temperature: (json['temperature'] ?? 25.0).toDouble(),
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'touch_detected': touchDetected,
      'distance': distance,
      'temperature': temperature,
    };
  }
}
