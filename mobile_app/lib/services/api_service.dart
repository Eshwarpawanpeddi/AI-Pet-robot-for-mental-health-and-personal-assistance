import 'dart:convert';
import 'package:http/http.dart' as http;
import '../models/mood_log.dart';

class ApiService {
  final String baseUrl;

  ApiService(this.baseUrl);

  Future<Map<String, dynamic>> getMoodResponse(
    String mood,
    int intensity,
    String notes,
  ) async {
    try {
      final response = await http.post(
        Uri.parse('$baseUrl/mood'),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode({
          'mood': mood,
          'intensity': intensity,
          'notes': notes,
        }),
      );

      if (response.statusCode == 200) {
        return jsonDecode(response.body);
      } else {
        throw Exception('Failed to log mood');
      }
    } catch (e) {
      print('Error logging mood: $e');
      rethrow;
    }
  }

  Future<String> getAffirmation() async {
    try {
      final response = await http.post(
        Uri.parse('$baseUrl/affirmation'),
        headers: {'Content-Type': 'application/json'},
      );

      if (response.statusCode == 200) {
        final data = jsonDecode(response.body);
        return data['affirmation'] ?? '';
      } else {
        throw Exception('Failed to get affirmation');
      }
    } catch (e) {
      print('Error getting affirmation: $e');
      rethrow;
    }
  }

  Future<Map<String, dynamic>> getBreathingExercise() async {
    try {
      final response = await http.post(
        Uri.parse('$baseUrl/breathing'),
        headers: {'Content-Type': 'application/json'},
      );

      if (response.statusCode == 200) {
        return jsonDecode(response.body);
      } else {
        throw Exception('Failed to get breathing exercise');
      }
    } catch (e) {
      print('Error getting breathing exercise: $e');
      rethrow;
    }
  }

  Future<Map<String, dynamic>> getCrisisResources() async {
    try {
      final response = await http.get(
        Uri.parse('$baseUrl/crisis_resources'),
        headers: {'Content-Type': 'application/json'},
      );

      if (response.statusCode == 200) {
        return jsonDecode(response.body);
      } else {
        throw Exception('Failed to get crisis resources');
      }
    } catch (e) {
      print('Error getting crisis resources: $e');
      rethrow;
    }
  }

  Future<Map<String, dynamic>> getRobotState() async {
    try {
      final response = await http.get(
        Uri.parse('$baseUrl/state'),
        headers: {'Content-Type': 'application/json'},
      );

      if (response.statusCode == 200) {
        return jsonDecode(response.body);
      } else {
        throw Exception('Failed to get robot state');
      }
    } catch (e) {
      print('Error getting robot state: $e');
      rethrow;
    }
  }

  Future<Map<String, dynamic>> setControlMode(String mode) async {
    try {
      final response = await http.post(
        Uri.parse('$baseUrl/control_mode'),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode({'mode': mode}),
      );

      if (response.statusCode == 200) {
        return jsonDecode(response.body);
      } else {
        throw Exception('Failed to set control mode');
      }
    } catch (e) {
      print('Error setting control mode: $e');
      rethrow;
    }
  }

  Future<String> getControlMode() async {
    try {
      final response = await http.get(
        Uri.parse('$baseUrl/state'),
        headers: {'Content-Type': 'application/json'},
      );

      if (response.statusCode == 200) {
        final data = jsonDecode(response.body);
        return data['control_mode'] ?? 'manual';
      } else {
        throw Exception('Failed to get control mode');
      }
    } catch (e) {
      print('Error getting control mode: $e');
      rethrow;
    }
  }
  
  Future<void> speak(String text) async {
    try {
      await http.post(
        Uri.parse('$baseUrl/speak'),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode({'text': text}),
      );
    } catch (e) {
      print('Error sending speak command: $e');
      rethrow;
    }
  }
}
