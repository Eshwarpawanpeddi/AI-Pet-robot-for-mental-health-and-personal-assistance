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
  
  String? _currentUrl;
  bool _isConnected = false;

  Stream<RobotState> get stateStream => _stateController.stream;
  Stream<bool> get connectionStream => _connectionController.stream;
  bool get isConnected => _isConnected;

  Future<void> connect(String url) async {
    try {
      // Close existing connection
      await disconnect();

      _currentUrl = url;
      _channel = WebSocketChannel.connect(Uri.parse(url));

      // Listen to messages
      _channel!.stream.listen(
        (message) {
          _handleMessage(message);
        },
        onError: (error) {
          print('WebSocket error: $error');
          _setConnectionStatus(false);
        },
        onDone: () {
          print('WebSocket connection closed');
          _setConnectionStatus(false);
        },
      );

      _setConnectionStatus(true);
    } catch (e) {
      print('Failed to connect: $e');
      _setConnectionStatus(false);
      rethrow;
    }
  }

  void _handleMessage(dynamic message) {
    try {
      final data = jsonDecode(message);
      
      if (data['type'] == 'state' && data['data'] != null) {
        final state = RobotState.fromJson(data['data']);
        _stateController.add(state);
      }
    } catch (e) {
      print('Error handling message: $e');
    }
  }

  void _setConnectionStatus(bool status) {
    _isConnected = status;
    _connectionController.add(status);
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

  void sendVoiceCommand(String audioData) {
    if (!_isConnected || _channel == null) return;

    final command = {
      'type': 'voice',
      'audio': audioData,
    };

    _channel!.sink.add(jsonEncode(command));
  }

  Future<void> disconnect() async {
    await _channel?.sink.close();
    _channel = null;
    _setConnectionStatus(false);
  }

  void dispose() {
    disconnect();
    _stateController.close();
    _connectionController.close();
  }
}
