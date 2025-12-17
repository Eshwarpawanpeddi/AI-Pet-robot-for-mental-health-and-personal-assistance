import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/robot_provider.dart';
import '../config/app_config.dart';

class SettingsScreen extends StatefulWidget {
  const SettingsScreen({super.key});

  @override
  State<SettingsScreen> createState() => _SettingsScreenState();
}

class _SettingsScreenState extends State<SettingsScreen> {
  final TextEditingController _urlController = TextEditingController();
  bool _isConnecting = false;

  @override
  void initState() {
    super.initState();
    final provider = Provider.of<RobotProvider>(context, listen: false);
    _urlController.text = provider.serverUrl ?? AppConfig.defaultServerUrl;
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Settings'),
      ),
      body: Consumer<RobotProvider>(
        builder: (context, provider, child) {
          return ListView(
            padding: const EdgeInsets.all(16),
            children: [
              // Connection Section
              Text(
                'Server Connection',
                style: Theme.of(context).textTheme.titleLarge,
              ),
              const SizedBox(height: 16),
              
              TextField(
                controller: _urlController,
                decoration: const InputDecoration(
                  labelText: 'Server IP Address',
                  hintText: '192.168.1.100',
                  border: OutlineInputBorder(),
                  prefixIcon: Icon(Icons.computer),
                ),
                keyboardType: TextInputType.url,
              ),
              
              const SizedBox(height: 16),
              
              Row(
                children: [
                  Expanded(
                    child: ElevatedButton.icon(
                      onPressed: _isConnecting
                          ? null
                          : () async {
                              setState(() => _isConnecting = true);
                              try {
                                await provider.connect(_urlController.text);
                                if (mounted) {
                                  ScaffoldMessenger.of(context).showSnackBar(
                                    const SnackBar(
                                      content: Text('Connected successfully!'),
                                      backgroundColor: Colors.green,
                                    ),
                                  );
                                }
                              } catch (e) {
                                if (mounted) {
                                  ScaffoldMessenger.of(context).showSnackBar(
                                    SnackBar(
                                      content: Text('Connection failed: $e'),
                                      backgroundColor: Colors.red,
                                    ),
                                  );
                                }
                              } finally {
                                setState(() => _isConnecting = false);
                              }
                            },
                      icon: _isConnecting
                          ? const SizedBox(
                              width: 16,
                              height: 16,
                              child: CircularProgressIndicator(strokeWidth: 2),
                            )
                          : const Icon(Icons.link),
                      label: Text(_isConnecting ? 'Connecting...' : 'Connect'),
                      style: ElevatedButton.styleFrom(
                        padding: const EdgeInsets.all(16),
                      ),
                    ),
                  ),
                  const SizedBox(width: 12),
                  ElevatedButton.icon(
                    onPressed: provider.isConnected
                        ? () async {
                            await provider.disconnect();
                            if (mounted) {
                              ScaffoldMessenger.of(context).showSnackBar(
                                const SnackBar(
                                  content: Text('Disconnected'),
                                ),
                              );
                            }
                          }
                        : null,
                    icon: const Icon(Icons.link_off),
                    label: const Text('Disconnect'),
                    style: ElevatedButton.styleFrom(
                      padding: const EdgeInsets.all(16),
                    ),
                  ),
                ],
              ),
              
              const SizedBox(height: 16),
              
              Card(
                child: ListTile(
                  leading: Icon(
                    provider.isConnected ? Icons.check_circle : Icons.cancel,
                    color: provider.isConnected ? Colors.green : Colors.red,
                  ),
                  title: Text(
                    provider.isConnected ? 'Connected' : 'Disconnected',
                  ),
                  subtitle: provider.isConnected
                      ? Text('Server: ${provider.serverUrl}')
                      : const Text('Not connected to server'),
                ),
              ),
              
              const Divider(height: 32),
              
              // Crisis Resources
              Text(
                'Crisis Resources',
                style: Theme.of(context).textTheme.titleLarge,
              ),
              const SizedBox(height: 16),
              
              ...AppConfig.crisisResources.entries.map((entry) {
                return Card(
                  color: Colors.red.shade50,
                  child: ListTile(
                    leading: const Icon(Icons.emergency, color: Colors.red),
                    title: Text(entry.key),
                    subtitle: Text(entry.value),
                    trailing: const Icon(Icons.arrow_forward_ios),
                    onTap: () {
                      // Could implement phone dialer or browser
                      ScaffoldMessenger.of(context).showSnackBar(
                        SnackBar(content: Text('${entry.key}: ${entry.value}')),
                      );
                    },
                  ),
                );
              }),
              
              const Divider(height: 32),
              
              // About Section
              Text(
                'About',
                style: Theme.of(context).textTheme.titleLarge,
              ),
              const SizedBox(height: 16),
              
              const Card(
                child: Padding(
                  padding: EdgeInsets.all(16),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        'AI Pet Robot',
                        style: TextStyle(
                          fontSize: 20,
                          fontWeight: FontWeight.bold,
                        ),
                      ),
                      SizedBox(height: 8),
                      Text('Version: ${AppConfig.appVersion}'),
                      SizedBox(height: 16),
                      Text(
                        'A companion robot for mental health support and personal assistance.',
                        style: TextStyle(fontSize: 14),
                      ),
                      SizedBox(height: 16),
                      Text(
                        '⚠️ Important: This robot is a supportive companion, NOT a replacement for professional mental health care.',
                        style: TextStyle(
                          fontSize: 12,
                          fontStyle: FontStyle.italic,
                        ),
                      ),
                    ],
                  ),
                ),
              ),
            ],
          );
        },
      ),
    );
  }

  @override
  void dispose() {
    _urlController.dispose();
    super.dispose();
  }
}
