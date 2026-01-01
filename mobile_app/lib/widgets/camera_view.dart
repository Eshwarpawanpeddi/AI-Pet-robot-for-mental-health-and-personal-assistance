import 'package:flutter/material.dart';
import 'dart:convert';
import 'dart:typed_data';

class CameraView extends StatefulWidget {
  final String? cameraFrame;
  final bool isEnabled;
  final VoidCallback onToggle;

  const CameraView({
    super.key,
    this.cameraFrame,
    required this.isEnabled,
    required this.onToggle,
  });

  @override
  State<CameraView> createState() => _CameraViewState();
}

class _CameraViewState extends State<CameraView> {
  @override
  Widget build(BuildContext context) {
    return Card(
      elevation: 4,
      child: Column(
        children: [
          // Camera header
          Container(
            padding: const EdgeInsets.all(12),
            decoration: BoxDecoration(
              color: Theme.of(context).colorScheme.primaryContainer,
              borderRadius: const BorderRadius.only(
                topLeft: Radius.circular(12),
                topRight: Radius.circular(12),
              ),
            ),
            child: Row(
              children: [
                Icon(
                  Icons.videocam,
                  color: Theme.of(context).colorScheme.onPrimaryContainer,
                ),
                const SizedBox(width: 8),
                Text(
                  'Robot Camera',
                  style: TextStyle(
                    fontSize: 16,
                    fontWeight: FontWeight.bold,
                    color: Theme.of(context).colorScheme.onPrimaryContainer,
                  ),
                ),
                const Spacer(),
                Switch(
                  value: widget.isEnabled,
                  onChanged: (value) => widget.onToggle(),
                ),
              ],
            ),
          ),
          
          // Camera view
          Container(
            height: 200,
            width: double.infinity,
            color: Colors.black,
            child: widget.isEnabled
                ? widget.cameraFrame != null
                    ? _buildCameraImage()
                    : const Center(
                        child: CircularProgressIndicator(),
                      )
                : Center(
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        Icon(
                          Icons.videocam_off,
                          size: 48,
                          color: Colors.grey[600],
                        ),
                        const SizedBox(height: 8),
                        Text(
                          'Camera Off',
                          style: TextStyle(
                            color: Colors.grey[600],
                            fontSize: 14,
                          ),
                        ),
                      ],
                    ),
                  ),
          ),
        ],
      ),
    );
  }

  Widget _buildCameraImage() {
    try {
      final imageBytes = base64Decode(widget.cameraFrame!);
      return Image.memory(
        imageBytes,
        fit: BoxFit.contain,
        gaplessPlayback: true,
      );
    } catch (e) {
      return Center(
        child: Text(
          'Error displaying camera',
          style: TextStyle(color: Colors.grey[600]),
        ),
      );
    }
  }
}
