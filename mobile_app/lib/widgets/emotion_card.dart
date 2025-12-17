import 'package:flutter/material.dart';
import '../config/app_config.dart';

class EmotionCard extends StatelessWidget {
  final String emotion;
  final int batteryLevel;
  final bool isListening;
  final bool isSpeaking;

  const EmotionCard({
    super.key,
    required this.emotion,
    required this.batteryLevel,
    required this.isListening,
    required this.isSpeaking,
  });

  @override
  Widget build(BuildContext context) {
    final color = AppConfig.emotionColors[emotion] ?? Colors.cyan;

    return Card(
      margin: const EdgeInsets.symmetric(horizontal: 16),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          children: [
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceAround,
              children: [
                _buildStatusItem(
                  context,
                  Icons.mood,
                  emotion.toUpperCase(),
                  color,
                ),
                _buildStatusItem(
                  context,
                  Icons.battery_std,
                  '$batteryLevel%',
                  _getBatteryColor(batteryLevel),
                ),
              ],
            ),
            const Divider(height: 24),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceAround,
              children: [
                _buildIndicator(
                  context,
                  isListening,
                  Icons.mic,
                  'Listening',
                ),
                _buildIndicator(
                  context,
                  isSpeaking,
                  Icons.volume_up,
                  'Speaking',
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildStatusItem(
    BuildContext context,
    IconData icon,
    String text,
    Color color,
  ) {
    return Column(
      children: [
        Icon(icon, size: 32, color: color),
        const SizedBox(height: 4),
        Text(
          text,
          style: TextStyle(
            fontWeight: FontWeight.bold,
            color: color,
          ),
        ),
      ],
    );
  }

  Widget _buildIndicator(
    BuildContext context,
    bool isActive,
    IconData icon,
    String label,
  ) {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Container(
          width: 12,
          height: 12,
          decoration: BoxDecoration(
            shape: BoxShape.circle,
            color: isActive ? Colors.green : Colors.grey,
            boxShadow: isActive
                ? [
                    BoxShadow(
                      color: Colors.green.withOpacity(0.5),
                      blurRadius: 8,
                      spreadRadius: 2,
                    ),
                  ]
                : null,
          ),
        ),
        const SizedBox(width: 8),
        Icon(icon, size: 20),
        const SizedBox(width: 4),
        Text(label),
      ],
    );
  }

  Color _getBatteryColor(int level) {
    if (level > 50) return Colors.green;
    if (level > 20) return Colors.orange;
    return Colors.red;
  }
}
