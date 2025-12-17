import 'package:flutter/material.dart';
import '../config/app_config.dart';

class RobotFaceWidget extends StatefulWidget {
  final String emotion;

  const RobotFaceWidget({
    super.key,
    required this.emotion,
  });

  @override
  State<RobotFaceWidget> createState() => _RobotFaceWidgetState();
}

class _RobotFaceWidgetState extends State<RobotFaceWidget>
    with SingleTickerProviderStateMixin {
  late AnimationController _blinkController;
  late Animation<double> _blinkAnimation;

  @override
  void initState() {
    super.initState();
    _blinkController = AnimationController(
      duration: const Duration(milliseconds: 200),
      vsync: this,
    );

    _blinkAnimation = Tween<double>(begin: 1.0, end: 0.0).animate(
      CurvedAnimation(parent: _blinkController, curve: Curves.easeInOut),
    );

    // Blink every 3 seconds
    _startBlinking();
  }

  void _startBlinking() {
    Future.delayed(const Duration(seconds: 3), () {
      if (mounted) {
        _blinkController.forward().then((_) {
          _blinkController.reverse();
          _startBlinking();
        });
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    final color = AppConfig.emotionColors[widget.emotion] ?? Colors.cyan;

    return Container(
      width: 300,
      height: 300,
      decoration: BoxDecoration(
        shape: BoxShape.circle,
        color: Colors.grey.shade900,
        boxShadow: [
          BoxShadow(
            color: color.withOpacity(0.5),
            blurRadius: 30,
            spreadRadius: 5,
          ),
        ],
      ),
      child: AnimatedBuilder(
        animation: _blinkAnimation,
        builder: (context, child) {
          return CustomPaint(
            painter: RobotFacePainter(
              emotion: widget.emotion,
              color: color,
              eyeOpen: _blinkAnimation.value,
            ),
          );
        },
      ),
    );
  }

  @override
  void dispose() {
    _blinkController.dispose();
    super.dispose();
  }
}

class RobotFacePainter extends CustomPainter {
  final String emotion;
  final Color color;
  final double eyeOpen;

  RobotFacePainter({
    required this.emotion,
    required this.color,
    required this.eyeOpen,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final centerX = size.width / 2;
    final centerY = size.height / 2;

    // Draw eyes
    _drawEyes(canvas, centerX, centerY);

    // Draw mouth based on emotion
    _drawMouth(canvas, centerX, centerY);
  }

  void _drawEyes(Canvas canvas, double centerX, double centerY) {
    final eyePaint = Paint()
      ..color = Colors.white
      ..style = PaintingStyle.fill;

    final pupilPaint = Paint()
      ..color = Colors.black
      ..style = PaintingStyle.fill;

    final eyeSpacing = 60.0;
    final eyeRadius = 35.0;
    final pupilRadius = 15.0;

    // Left eye
    canvas.drawOval(
      Rect.fromCenter(
        center: Offset(centerX - eyeSpacing, centerY - 40),
        width: eyeRadius * 2,
        height: eyeRadius * 2 * eyeOpen,
      ),
      eyePaint,
    );

    if (eyeOpen > 0.3) {
      canvas.drawCircle(
        Offset(centerX - eyeSpacing, centerY - 40),
        pupilRadius * eyeOpen,
        pupilPaint,
      );
    }

    // Right eye
    canvas.drawOval(
      Rect.fromCenter(
        center: Offset(centerX + eyeSpacing, centerY - 40),
        width: eyeRadius * 2,
        height: eyeRadius * 2 * eyeOpen,
      ),
      eyePaint,
    );

    if (eyeOpen > 0.3) {
      canvas.drawCircle(
        Offset(centerX + eyeSpacing, centerY - 40),
        pupilRadius * eyeOpen,
        pupilPaint,
      );
    }
  }

  void _drawMouth(Canvas canvas, double centerX, double centerY) {
    final mouthPaint = Paint()
      ..color = color
      ..style = PaintingStyle.stroke
      ..strokeWidth = 4;

    final mouthY = centerY + 60;
    final mouthWidth = 40.0;

    switch (emotion) {
      case 'happy':
        // Smiling mouth
        canvas.drawArc(
          Rect.fromCenter(
            center: Offset(centerX, mouthY),
            width: mouthWidth * 2,
            height: mouthWidth,
          ),
          0,
          3.14159, // π
          false,
          mouthPaint,
        );
        break;

      case 'sad':
        // Sad mouth (upside down)
        canvas.drawArc(
          Rect.fromCenter(
            center: Offset(centerX, mouthY - 10),
            width: mouthWidth * 2,
            height: mouthWidth,
          ),
          3.14159, // π
          3.14159, // π
          false,
          mouthPaint,
        );
        break;

      case 'anxious':
      case 'stressed':
        // Wavy mouth
        final path = Path();
        path.moveTo(centerX - mouthWidth, mouthY);
        path.quadraticBezierTo(
          centerX - mouthWidth / 2,
          mouthY + 10,
          centerX,
          mouthY,
        );
        path.quadraticBezierTo(
          centerX + mouthWidth / 2,
          mouthY - 10,
          centerX + mouthWidth,
          mouthY,
        );
        canvas.drawPath(path, mouthPaint);
        break;

      case 'angry':
        // Straight line
        canvas.drawLine(
          Offset(centerX - mouthWidth, mouthY),
          Offset(centerX + mouthWidth, mouthY),
          mouthPaint,
        );
        break;

      default:
        // Neutral mouth (O shape)
        canvas.drawOval(
          Rect.fromCenter(
            center: Offset(centerX, mouthY),
            width: mouthWidth / 2,
            height: 20,
          ),
          mouthPaint,
        );
    }
  }

  @override
  bool shouldRepaint(RobotFacePainter oldDelegate) {
    return oldDelegate.emotion != emotion || oldDelegate.eyeOpen != eyeOpen;
  }
}
