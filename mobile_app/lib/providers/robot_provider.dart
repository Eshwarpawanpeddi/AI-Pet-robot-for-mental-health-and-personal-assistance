import 'package:flutter/foundation.dart';
import 'package:shared_preferences/shared_preferences.dart';
import '../models/robot_state.dart';
import '../models/mood_log.dart';
import '../services/websocket_service.dart';
import '../services/api_service.dart';
import '../config/app_config.dart';

class RobotProvider with ChangeNotifier {
  final WebSocketService _wsService = WebSocketService();
  ApiService? _apiService;
  
  RobotState _robotState = RobotState(
    emotion: 'neutral',
    isSpeaking: false,
    isListening: false,
    batteryLevel: 100,
  );
  
  bool _isConnected = false;
  String? _serverUrl;
  List<MoodLog> _moodLogs = [];
  String? _lastAffirmation;
  String? _errorMessage;
  String _controlMode = 'manual'; // manual or autonomous
  bool _espConnected = false;
  bool _raspberryPiConnected = false;

  // Getters
  RobotState get robotState => _robotState;
  bool get isConnected => _isConnected;
  String? get serverUrl => _serverUrl;
  List<MoodLog> get moodLogs => _moodLogs;
  String? get lastAffirmation => _lastAffirmation;
  String? get errorMessage => _errorMessage;
  String get controlMode => _controlMode;
  bool get espConnected => _espConnected;
  bool get raspberryPiConnected => _raspberryPiConnected;

  RobotProvider() {
    _init();
  }

  Future<void> _init() async {
    // Load saved server URL
    final prefs = await SharedPreferences.getInstance();
    _serverUrl = prefs.getString(AppConfig.keyServerUrl);
    
    // Load mood logs
    await _loadMoodLogs();
    
    // Listen to WebSocket streams
    _wsService.stateStream.listen((state) {
      _robotState = state;
      
      // Update connection status from state if available
      if (state.controlMode != null) {
        _controlMode = state.controlMode!;
      }
      if (state.espConnected != null) {
        _espConnected = state.espConnected!;
      }
      if (state.raspberryPiConnected != null) {
        _raspberryPiConnected = state.raspberryPiConnected!;
      }
      
      notifyListeners();
    });

    _wsService.connectionStream.listen((connected) {
      _isConnected = connected;
      notifyListeners();
    });
    
    // Auto-connect if server URL is set
    if (_serverUrl != null && _serverUrl!.isNotEmpty) {
      await connect(_serverUrl!);
    }
  }

  Future<void> connect(String url) async {
    try {
      _errorMessage = null;
      final wsUrl = AppConfig.getWebSocketUrl(url);
      final apiUrl = AppConfig.getApiUrl(url);
      
      await _wsService.connect(wsUrl);
      _apiService = ApiService(apiUrl);
      _serverUrl = url;
      
      // Save server URL
      final prefs = await SharedPreferences.getInstance();
      await prefs.setString(AppConfig.keyServerUrl, url);
      
      notifyListeners();
    } catch (e) {
      _errorMessage = 'Failed to connect: $e';
      notifyListeners();
      rethrow;
    }
  }

  Future<void> disconnect() async {
    await _wsService.disconnect();
    _apiService = null;
    notifyListeners();
  }

  // Robot Control Methods
  void moveForward() => _wsService.sendMoveCommand('forward', 200);
  void moveBackward() => _wsService.sendMoveCommand('backward', 200);
  void turnLeft() => _wsService.sendMoveCommand('left', 200);
  void turnRight() => _wsService.sendMoveCommand('right', 200);
  void stop() => _wsService.sendMoveCommand('stop', 0);

  void setEmotion(String emotion) {
    _wsService.sendEmotionCommand(emotion);
  }

  // Control Mode Methods
  Future<void> setControlMode(String mode) async {
    if (_apiService == null) {
      _errorMessage = 'Not connected to server';
      notifyListeners();
      return;
    }

    try {
      await _apiService!.setControlMode(mode);
      _controlMode = mode;
      notifyListeners();
    } catch (e) {
      _errorMessage = 'Failed to set control mode: $e';
      notifyListeners();
    }
  }

  void toggleControlMode() {
    final newMode = _controlMode == 'manual' ? 'autonomous' : 'manual';
    setControlMode(newMode);
  }

  // Mental Health Methods
  Future<MoodLog?> logMood(String mood, int intensity, String notes) async {
    if (_apiService == null) {
      _errorMessage = 'Not connected to server';
      notifyListeners();
      return null;
    }

    try {
      final response = await _apiService!.getMoodResponse(mood, intensity, notes);
      
      final moodLog = MoodLog(
        mood: mood,
        intensity: intensity,
        notes: notes,
        response: response['response'],
        suggestions: response['suggestions'] != null
            ? List<String>.from(response['suggestions'])
            : null,
      );
      
      _moodLogs.insert(0, moodLog);
      await _saveMoodLogs();
      notifyListeners();
      
      return moodLog;
    } catch (e) {
      _errorMessage = 'Failed to log mood: $e';
      notifyListeners();
      return null;
    }
  }

  Future<void> getAffirmation() async {
    if (_apiService == null) {
      _errorMessage = 'Not connected to server';
      notifyListeners();
      return;
    }

    try {
      _lastAffirmation = await _apiService!.getAffirmation();
      notifyListeners();
    } catch (e) {
      _errorMessage = 'Failed to get affirmation: $e';
      notifyListeners();
    }
  }

  Future<Map<String, dynamic>?> startBreathingExercise() async {
    if (_apiService == null) {
      _errorMessage = 'Not connected to server';
      notifyListeners();
      return null;
    }

    try {
      return await _apiService!.getBreathingExercise();
    } catch (e) {
      _errorMessage = 'Failed to get breathing exercise: $e';
      notifyListeners();
      return null;
    }
  }

  Future<Map<String, dynamic>?> getCrisisResources() async {
    if (_apiService == null) {
      _errorMessage = 'Not connected to server';
      notifyListeners();
      return null;
    }

    try {
      return await _apiService!.getCrisisResources();
    } catch (e) {
      _errorMessage = 'Failed to get crisis resources: $e';
      notifyListeners();
      return null;
    }
  }

  // Storage Methods
  Future<void> _loadMoodLogs() async {
    try {
      final prefs = await SharedPreferences.getInstance();
      final logsJson = prefs.getStringList(AppConfig.keyMoodLogs) ?? [];
      
      _moodLogs = logsJson
          .map((json) => MoodLog.fromJson(Map<String, dynamic>.from(
              const JsonDecoder().convert(json))))
          .toList();
    } catch (e) {
      print('Error loading mood logs: $e');
    }
  }

  Future<void> _saveMoodLogs() async {
    try {
      final prefs = await SharedPreferences.getInstance();
      final logsJson = _moodLogs
          .take(100) // Keep only last 100 logs
          .map((log) => const JsonEncoder().convert(log.toJson()))
          .toList();
      
      await prefs.setStringList(AppConfig.keyMoodLogs, logsJson);
    } catch (e) {
      print('Error saving mood logs: $e');
    }
  }

  void clearError() {
    _errorMessage = null;
    notifyListeners();
  }

  @override
  void dispose() {
    _wsService.dispose();
    super.dispose();
  }
}
