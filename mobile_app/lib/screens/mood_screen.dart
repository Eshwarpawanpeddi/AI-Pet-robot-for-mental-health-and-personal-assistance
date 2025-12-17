import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/robot_provider.dart';
import '../config/app_config.dart';

class MoodScreen extends StatefulWidget {
  const MoodScreen({super.key});

  @override
  State<MoodScreen> createState() => _MoodScreenState();
}

class _MoodScreenState extends State<MoodScreen> {
  String _selectedMood = 'neutral';
  double _intensity = 5.0;
  final TextEditingController _notesController = TextEditingController();

  final List<String> _moods = [
    'happy',
    'sad',
    'anxious',
    'stressed',
    'tired',
    'angry',
    'neutral',
  ];

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Log Your Mood'),
      ),
      body: Consumer<RobotProvider>(
        builder: (context, provider, child) {
          return SingleChildScrollView(
            padding: const EdgeInsets.all(16),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                Text(
                  'How are you feeling?',
                  style: Theme.of(context).textTheme.headlineSmall,
                ),
                const SizedBox(height: 20),
                
                // Mood Selection
                Wrap(
                  spacing: 8,
                  runSpacing: 8,
                  children: _moods.map((mood) {
                    final isSelected = _selectedMood == mood;
                    return ChoiceChip(
                      label: Text(
                        mood.toUpperCase(),
                        style: TextStyle(
                          color: isSelected ? Colors.white : null,
                        ),
                      ),
                      selected: isSelected,
                      selectedColor: AppConfig.emotionColors[mood],
                      onSelected: (selected) {
                        if (selected) {
                          setState(() => _selectedMood = mood);
                        }
                      },
                    );
                  }).toList(),
                ),
                
                const SizedBox(height: 24),
                
                // Intensity Slider
                Text(
                  'Intensity: ${_intensity.round()}/10',
                  style: Theme.of(context).textTheme.titleMedium,
                ),
                Slider(
                  value: _intensity,
                  min: 1,
                  max: 10,
                  divisions: 9,
                  label: _intensity.round().toString(),
                  onChanged: (value) {
                    setState(() => _intensity = value);
                  },
                ),
                
                const SizedBox(height: 24),
                
                // Notes TextField
                TextField(
                  controller: _notesController,
                  decoration: const InputDecoration(
                    labelText: 'Notes (optional)',
                    hintText: 'What\'s on your mind?',
                    border: OutlineInputBorder(),
                  ),
                  maxLines: 4,
                ),
                
                const SizedBox(height: 24),
                
                // Submit Button
                ElevatedButton(
                  onPressed: () async {
                    final moodLog = await provider.logMood(
                      _selectedMood,
                      _intensity.round(),
                      _notesController.text,
                    );
                    
                    if (moodLog != null && mounted) {
                      _showResponseDialog(
                        context,
                        moodLog.response ?? 'Mood logged successfully',
                        moodLog.suggestions ?? [],
                      );
                    }
                  },
                  style: ElevatedButton.styleFrom(
                    padding: const EdgeInsets.all(16),
                  ),
                  child: const Text('Log Mood'),
                ),
                
                const SizedBox(height: 32),
                
                // Mood History
                Text(
                  'Recent Mood Logs',
                  style: Theme.of(context).textTheme.titleLarge,
                ),
                const SizedBox(height: 12),
                
                ...provider.moodLogs.take(5).map((log) {
                  return Card(
                    child: ListTile(
                      leading: CircleAvatar(
                        backgroundColor: AppConfig.emotionColors[log.mood],
                        child: Text(
                          log.intensity.toString(),
                          style: const TextStyle(color: Colors.white),
                        ),
                      ),
                      title: Text(log.mood.toUpperCase()),
                      subtitle: Text(
                        '${log.notes}\n${_formatDate(log.timestamp)}',
                      ),
                      isThreeLine: true,
                    ),
                  );
                }),
              ],
            ),
          );
        },
      ),
    );
  }

  String _formatDate(DateTime date) {
    return '${date.day}/${date.month}/${date.year} ${date.hour}:${date.minute.toString().padLeft(2, '0')}';
  }

  void _showResponseDialog(
    BuildContext context,
    String response,
    List<String> suggestions,
  ) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text('Support Response'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(response),
            if (suggestions.isNotEmpty) ...[
              const SizedBox(height: 16),
              const Text(
                'Suggestions:',
                style: TextStyle(fontWeight: FontWeight.bold),
              ),
              const SizedBox(height: 8),
              ...suggestions.map((suggestion) => Text('• $suggestion')),
            ],
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: const Text('Close'),
          ),
          TextButton(
            onPressed: () {
              Navigator.pop(context);
              Navigator.pop(context);
            },
            child: const Text('Back to Home'),
          ),
        ],
      ),
    );
  }

  @override
  void dispose() {
    _notesController.dispose();
    super.dispose();
  }
}
