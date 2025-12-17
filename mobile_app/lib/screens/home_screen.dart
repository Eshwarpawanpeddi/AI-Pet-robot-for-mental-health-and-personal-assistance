import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/robot_provider.dart';
import '../widgets/robot_face.dart';
import '../widgets/control_pad.dart';
import '../widgets/emotion_card.dart';
import 'mood_screen.dart';
import 'settings_screen.dart';
import '../config/app_config.dart';

class HomeScreen extends StatelessWidget {
  const HomeScreen({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('AI Pet Robot'),
        actions: [
          Consumer<RobotProvider>(
            builder: (context, provider, child) {
              return IconButton(
                icon: Icon(
                  provider.isConnected ? Icons.cloud_done : Icons.cloud_off,
                  color: provider.isConnected ? Colors.green : Colors.red,
                ),
                onPressed: () {
                  Navigator.push(
                    context,
                    MaterialPageRoute(
                      builder: (context) => const SettingsScreen(),
                    ),
                  );
                },
              );
            },
          ),
        ],
      ),
      body: SafeArea(
        child: Consumer<RobotProvider>(
          builder: (context, provider, child) {
            if (!provider.isConnected) {
              return Center(
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    Icon(
                      Icons.cloud_off,
                      size: 64,
                      color: Colors.grey[400],
                    ),
                    const SizedBox(height: 16),
                    Text(
                      'Not Connected',
                      style: Theme.of(context).textTheme.headlineSmall,
                    ),
                    const SizedBox(height: 8),
                    Text(
                      'Go to Settings to connect',
                      style: Theme.of(context).textTheme.bodyMedium,
                    ),
                    const SizedBox(height: 24),
                    ElevatedButton.icon(
                      onPressed: () {
                        Navigator.push(
                          context,
                          MaterialPageRoute(
                            builder: (context) => const SettingsScreen(),
                          ),
                        );
                      },
                      icon: const Icon(Icons.settings),
                      label: const Text('Open Settings'),
                    ),
                  ],
                ),
              );
            }

            return SingleChildScrollView(
              child: Column(
                children: [
                  // Robot Face
                  const SizedBox(height: 20),
                  RobotFaceWidget(emotion: provider.robotState.emotion),
                  
                  const SizedBox(height: 20),
                  
                  // Emotion Card
                  EmotionCard(
                    emotion: provider.robotState.emotion,
                    batteryLevel: provider.robotState.batteryLevel,
                    isListening: provider.robotState.isListening,
                    isSpeaking: provider.robotState.isSpeaking,
                  ),
                  
                  const SizedBox(height: 20),
                  
                  // Control Pad
                  const ControlPad(),
                  
                  const SizedBox(height: 20),
                  
                  // Mental Health Quick Actions
                  Padding(
                    padding: const EdgeInsets.all(16.0),
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.stretch,
                      children: [
                        Text(
                          'Mental Health Support',
                          style: Theme.of(context).textTheme.titleLarge,
                        ),
                        const SizedBox(height: 12),
                        Row(
                          children: [
                            Expanded(
                              child: ElevatedButton.icon(
                                onPressed: () async {
                                  await provider.getAffirmation();
                                  if (provider.lastAffirmation != null) {
                                    _showAffirmationDialog(
                                      context,
                                      provider.lastAffirmation!,
                                    );
                                  }
                                },
                                icon: const Icon(Icons.favorite),
                                label: const Text('Affirmation'),
                                style: ElevatedButton.styleFrom(
                                  padding: const EdgeInsets.all(16),
                                ),
                              ),
                            ),
                            const SizedBox(width: 12),
                            Expanded(
                              child: ElevatedButton.icon(
                                onPressed: () {
                                  Navigator.push(
                                    context,
                                    MaterialPageRoute(
                                      builder: (context) => const MoodScreen(),
                                    ),
                                  );
                                },
                                icon: const Icon(Icons.mood),
                                label: const Text('Log Mood'),
                                style: ElevatedButton.styleFrom(
                                  padding: const EdgeInsets.all(16),
                                ),
                              ),
                            ),
                          ],
                        ),
                        const SizedBox(height: 12),
                        ElevatedButton.icon(
                          onPressed: () async {
                            final exercise = await provider.startBreathingExercise();
                            if (exercise != null) {
                              _showBreathingDialog(context, exercise);
                            }
                          },
                          icon: const Icon(Icons.air),
                          label: const Text('Breathing Exercise'),
                          style: ElevatedButton.styleFrom(
                            padding: const EdgeInsets.all(16),
                          ),
                        ),
                      ],
                    ),
                  ),
                  
                  const SizedBox(height: 20),
                ],
              ),
            );
          },
        ),
      ),
    );
  }

  void _showAffirmationDialog(BuildContext context, String affirmation) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: const Row(
          children: [
            Icon(Icons.favorite, color: Colors.pink),
            SizedBox(width: 8),
            Text('Positive Affirmation'),
          ],
        ),
        content: Text(
          affirmation,
          style: const TextStyle(fontSize: 18, fontStyle: FontStyle.italic),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: const Text('Close'),
          ),
        ],
      ),
    );
  }

  void _showBreathingDialog(BuildContext context, Map<String, dynamic> exercise) {
    final instructions = exercise['instructions'] as List<dynamic>?;
    
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: const Row(
          children: [
            Icon(Icons.air, color: Colors.blue),
            SizedBox(width: 8),
            Text('Breathing Exercise'),
          ],
        ),
        content: instructions != null
            ? Column(
                mainAxisSize: MainAxisSize.min,
                crossAxisAlignment: CrossAxisAlignment.start,
                children: instructions
                    .map((instruction) => Padding(
                          padding: const EdgeInsets.symmetric(vertical: 4),
                          child: Text('• $instruction'),
                        ))
                    .toList(),
              )
            : const Text('Follow the breathing pattern...'),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: const Text('Close'),
          ),
        ],
      ),
    );
  }
}
