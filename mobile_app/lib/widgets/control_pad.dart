import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/robot_provider.dart';

class ControlPad extends StatelessWidget {
  const ControlPad({super.key});

  @override
  Widget build(BuildContext context) {
    final provider = Provider.of<RobotProvider>(context, listen: false);

    return Container(
      padding: const EdgeInsets.all(16),
      child: Column(
        children: [
          Text(
            'Robot Control',
            style: Theme.of(context).textTheme.titleLarge,
          ),
          const SizedBox(height: 16),
          Row(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              const Spacer(),
              _buildControlButton(
                icon: Icons.arrow_upward,
                onPressed: provider.moveForward,
                color: Colors.green,
              ),
              const Spacer(),
            ],
          ),
          const SizedBox(height: 8),
          Row(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              _buildControlButton(
                icon: Icons.arrow_back,
                onPressed: provider.turnLeft,
                color: Colors.blue,
              ),
              const SizedBox(width: 8),
              _buildControlButton(
                icon: Icons.stop,
                onPressed: provider.stop,
                color: Colors.red,
              ),
              const SizedBox(width: 8),
              _buildControlButton(
                icon: Icons.arrow_forward,
                onPressed: provider.turnRight,
                color: Colors.blue,
              ),
            ],
          ),
          const SizedBox(height: 8),
          Row(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              const Spacer(),
              _buildControlButton(
                icon: Icons.arrow_downward,
                onPressed: provider.moveBackward,
                color: Colors.orange,
              ),
              const Spacer(),
            ],
          ),
          const SizedBox(height: 24),
          // Emotion buttons
          Text(
            'Change Emotion',
            style: Theme.of(context).textTheme.titleMedium,
          ),
          const SizedBox(height: 8),
          Wrap(
            spacing: 8,
            runSpacing: 8,
            alignment: WrapAlignment.center,
            children: [
              _buildEmotionButton(
                context,
                'happy',
                Icons.sentiment_very_satisfied,
                Colors.yellow,
                provider,
              ),
              _buildEmotionButton(
                context,
                'sad',
                Icons.sentiment_very_dissatisfied,
                Colors.blue,
                provider,
              ),
              _buildEmotionButton(
                context,
                'anxious',
                Icons.sentiment_neutral,
                Colors.orange,
                provider,
              ),
              _buildEmotionButton(
                context,
                'neutral',
                Icons.sentiment_neutral,
                Colors.grey,
                provider,
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildControlButton({
    required IconData icon,
    required VoidCallback onPressed,
    required Color color,
  }) {
    return ElevatedButton(
      onPressed: onPressed,
      style: ElevatedButton.styleFrom(
        backgroundColor: color,
        foregroundColor: Colors.white,
        padding: const EdgeInsets.all(20),
        shape: const CircleBorder(),
      ),
      child: Icon(icon, size: 32),
    );
  }

  Widget _buildEmotionButton(
    BuildContext context,
    String emotion,
    IconData icon,
    Color color,
    RobotProvider provider,
  ) {
    return ElevatedButton.icon(
      onPressed: () => provider.setEmotion(emotion),
      icon: Icon(icon),
      label: Text(emotion.toUpperCase()),
      style: ElevatedButton.styleFrom(
        backgroundColor: color,
        foregroundColor: Colors.white,
      ),
    );
  }
}
