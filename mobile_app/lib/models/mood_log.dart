class MoodLog {
  final String mood;
  final int intensity;
  final String notes;
  final DateTime timestamp;
  final String? response;
  final List<String>? suggestions;

  MoodLog({
    required this.mood,
    required this.intensity,
    required this.notes,
    DateTime? timestamp,
    this.response,
    this.suggestions,
  }) : timestamp = timestamp ?? DateTime.now();

  factory MoodLog.fromJson(Map<String, dynamic> json) {
    return MoodLog(
      mood: json['mood'] ?? 'neutral',
      intensity: json['intensity'] ?? 5,
      notes: json['notes'] ?? '',
      timestamp: json['timestamp'] != null
          ? DateTime.parse(json['timestamp'])
          : DateTime.now(),
      response: json['response'],
      suggestions: json['suggestions'] != null
          ? List<String>.from(json['suggestions'])
          : null,
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'mood': mood,
      'intensity': intensity,
      'notes': notes,
      'timestamp': timestamp.toIso8601String(),
      if (response != null) 'response': response,
      if (suggestions != null) 'suggestions': suggestions,
    };
  }
}
