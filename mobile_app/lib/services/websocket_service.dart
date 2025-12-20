import 'dart:async';
import 'dart:convert';
import 'package:web_socket_channel/web_socket_channel.dart';
import '../models/robot_state.dart';

class WebSocketService {
  WebSocketChannel? _channel;
  final StreamController<RobotState> _stateController =
      StreamController<RobotState>.broadcast();
  final StreamController<bool> _connectionController =
      StreamController<bool>.broadcast();
  final StreamController<String> _transcriptController =
      StreamController<String>.broadcast();
  
  String? _currentUrl;
  String? _authToken;
  bool _isConnected = false;
  Timer? _heartbeatTimer;
  Timer? _reconnectTimer;
  int _reconnectAttempts = 0;
  static const int _maxReconnectDelay = 60; // Maximum 60 seconds

  Stream<RobotState> get stateStream => _stateController.stream;
  Stream<bool> get connectionStream => _connectionController.stream;
  Stream<String> get transcriptStream => _transcriptController.stream;
  bool get isConnected => _isConnected;

  Future<void> connect(String url, {String? token}) async {
    try {
      // Close existing connection
      await disconnect();

      _currentUrl = url;
      _authToken = token;
      
      // Add token to URL if provided
      String wsUrl = url;
      if (token != null && token.isNotEmpty) {
        wsUrl += '?token=$token';
      }
      
      _channel = WebSocketChannel.connect(Uri.parse(wsUrl));

      // Listen to messages
      _channel!.stream.listen(
        (message) {
          _handleMessage(message);
        },
        onError: (error) {
          print('WebSocket error: $error');
          _setConnectionStatus(false);
          _scheduleReconnect();
        },
        onDone: () {
          print('WebSocket connection closed');
          _setConnectionStatus(false);
          _scheduleReconnect();
        },
      );

      _setConnectionStatus(true);
      _startHeartbeat();
      _reconnectAttempts = 0; // Reset on successful connection
    } catch (e) {
      print('Failed to connect: $e');
      _setConnectionStatus(false);
      _scheduleReconnect();
      rethrow;
    }
  }

  void _handleMessage(dynamic message) {
    try {
      final data = jsonDecode(message);
      final messageType = data['type'];
      
      if (messageType == 'state' && data['data'] != null) {
        final state = RobotState.fromJson(data['data']);
        _stateController.add(state);
      } else if (messageType == 'response' && data['transcript'] != null) {
        _transcriptController.add(data['transcript']);
      } else if (messageType == 'connection_established') {
        print('Connection established with server');
        if (data['state'] != null) {
          final state = RobotState.fromJson(data['state']);
          _stateController.add(state);
        }
      } else if (messageType == 'heartbeat_ack') {
        // Server acknowledged heartbeat
      }
    } catch (e) {
      print('Error handling message: $e');
    }
  }

  void _setConnectionStatus(bool status) {
    _isConnected = status;
    _connectionController.add(status);
  }

  void _startHeartbeat() {
    _heartbeatTimer?.cancel();
    _heartbeatTimer = Timer.periodic(Duration(seconds: 10), (timer) {
      if (_isConnected && _channel != null) {
        try {
          _channel!.sink.add(jsonEncode({'type': 'heartbeat'}));
        } catch (e) {
          print('Heartbeat error: $e');
        }
      }
    });
  }

  void _scheduleReconnect() {
    _reconnectTimer?.cancel();
    
    // Exponential backoff: 5s, 10s, 20s, 40s, up to max of 60s
    _reconnectAttempts++;
    int delay = (5 * (1 << (_reconnectAttempts - 1))).clamp(5, _maxReconnectDelay);
    
    print('Scheduling reconnect attempt $_reconnectAttempts in ${delay}s');
    
    _reconnectTimer = Timer(Duration(seconds: delay), () {
      if (!_isConnected && _currentUrl != null) {
        print('Attempting to reconnect...');
        connect(_currentUrl!, token: _authToken);
      }
    });
  }

  void sendMoveCommand(String direction, int speed) {
    if (!_isConnected || _channel == null) return;

    final command = {
      'type': 'move',
      'direction': direction,
      'speed': speed,
    };

    _channel!.sink.add(jsonEncode(command));
  }

  void sendEmotionCommand(String emotion) {
    if (!_isConnected || _channel == null) return;

    final command = {
      'type': 'emotion',
      'emotion': emotion,
    };

    _channel!.sink.add(jsonEncode(command));
  }

  void sendTextCommand(String text) {
    if (!_isConnected || _channel == null) return;

    final command = {
      'type': 'text',
      'text': text,
    };

    _channel!.sink.add(jsonEncode(command));
  }

  void sendImageCommand(String base64Image, {String? prompt}) {
    if (!_isConnected || _channel == null) return;

    final command = {
      'type': 'image',
      'image': base64Image,
      if (prompt != null) 'prompt': prompt,
    };

    _channel!.sink.add(jsonEncode(command));
  }

  void sendMultimodalCommand(String? text, String? base64Image) {
    if (!_isConnected || _channel == null) return;

    final command = {
      'type': 'multimodal',
      if (text != null) 'text': text,
      if (base64Image != null) 'image': base64Image,
    };

    _channel!.sink.add(jsonEncode(command));
  }

  void sendVoiceCommand(String audioData, {String? context}) {
    if (!_isConnected || _channel == null) return;

    final command = {
      'type': 'voice',
      'audio': audioData,
      if (context != null) 'context': context,
    };

    _channel!.sink.add(jsonEncode(command));
  }

  void sendModeCommand(String mode) {
    if (!_isConnected || _channel == null) return;

    final command = {
      'type': 'set_mode',
      'mode': mode,
    };

    _channel!.sink.add(jsonEncode(command));
  }

  void requestState() {
    if (!_isConnected || _channel == null) return;

    final command = {
      'type': 'get_state',
    };

    _channel!.sink.add(jsonEncode(command));
  }

  Future<void> disconnect() async {
    _heartbeatTimer?.cancel();
    _reconnectTimer?.cancel();
    await _channel?.sink.close();
    _channel = null;
    _setConnectionStatus(false);
  }

  void dispose() {
    disconnect();
    _stateController.close();
    _connectionController.close();
    _transcriptController.close();
  }
}
