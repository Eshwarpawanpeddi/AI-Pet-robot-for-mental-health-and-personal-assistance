# Mental Health & Personal Assistance Features

## Overview

This AI Pet Robot is specifically designed to provide mental health support and personal assistance through:

1. **Emotional Companionship**: Recognizes and responds to user emotions
2. **Conversation Support**: Engaging, empathetic conversations
3. **Routine Assistance**: Reminders, scheduling, and daily check-ins
4. **Mood Tracking**: Monitors user mood patterns over time
5. **Stress Relief**: Calming interactions and activities

## Mental Health Features

### 1. Emotion Recognition & Response

The robot detects user emotions through:
- Voice tone analysis (via Gemini AI)
- Interaction patterns
- User-reported mood

**Supported Emotions:**
- Happy
- Sad
- Anxious
- Stressed
- Neutral
- Excited
- Tired

### 2. Empathetic Conversations

**Conversation Modes:**
- **Active Listening**: Non-judgmental, supportive responses
- **Encouragement**: Positive affirmations and motivation
- **Distraction**: Engaging stories or activities
- **Grounding**: Helps users focus during anxiety

**Example Interactions:**
```
User: "I'm feeling overwhelmed today"
Robot: "I'm here for you. Would you like to talk about what's making you feel this way, or would you prefer a distraction?"

User: "I accomplished my goal today!"
Robot: "That's wonderful! I'm so proud of you. Tell me more about it!"
```

### 3. Daily Wellness Check-ins

**Morning Check-in:**
- Greets user with positive affirmation
- Asks about sleep quality
- Suggests daily intentions

**Evening Reflection:**
- Asks about the day's highlights
- Encourages gratitude practice
- Offers relaxation techniques

### 4. Mood Tracking

The robot logs mood data (with user consent) to identify patterns:
- Time of day mood variations
- Correlation with activities
- Weekly/monthly trends
- Trigger identification

### 5. Crisis Support

**Important**: This robot provides support but is NOT a replacement for professional help.

**Crisis Detection:**
- Recognizes distress signals
- Provides immediate supportive response
- Suggests professional resources
- Can alert emergency contacts (if configured)

**Emergency Resources:**
- National Suicide Prevention Lifeline: 988
- Crisis Text Line: Text HOME to 741741
- International Association for Suicide Prevention: https://www.iasp.info/resources/Crisis_Centres/

## Personal Assistance Features

### 1. Reminder System

**Types of Reminders:**
- Medication reminders
- Hydration reminders
- Break reminders (for extended work)
- Exercise/movement prompts
- Social connection reminders

**Example:**
```
Robot: "It's time for your afternoon medication. Would you like me to set a 5-minute reminder if you're busy?"
```

### 2. Routine Building

Helps establish healthy routines:
- Morning routines
- Evening wind-down
- Self-care activities
- Social activities

### 3. Mindfulness & Relaxation

**Guided Activities:**
- Breathing exercises
- Progressive muscle relaxation
- Grounding techniques (5-4-3-2-1 method)
- Meditation guidance

**Example:**
```
Robot: "Let's do a quick breathing exercise together. Breathe in for 4... hold for 4... breathe out for 4..."
```

### 4. Social Engagement

- Suggests activities to combat isolation
- Encourages social connections
- Provides conversation starters
- Celebrates social achievements

### 5. Goal Setting & Tracking

- Helps set achievable goals
- Breaks down large goals into steps
- Celebrates progress
- Provides gentle accountability

## Privacy & Data Protection

### Data Collected (Optional, User-Controlled)

1. **Conversation History**: For context and continuity
2. **Mood Logs**: For pattern recognition
3. **Routine Completion**: For habit tracking
4. **Interaction Frequency**: For engagement analysis

### User Privacy Controls

- **Opt-in**: All data collection requires explicit consent
- **Data Deletion**: Users can delete all data at any time
- **Local Storage**: Data stored locally on device (not cloud)
- **No Sharing**: Data never shared with third parties
- **Anonymization**: No personally identifiable information stored

### Configuration

Edit `server/config/mental_health.yaml`:
```yaml
mental_health:
  features:
    mood_tracking: true
    conversation_history: true
    reminders: true
    crisis_detection: true
  
  privacy:
    data_retention_days: 90
    require_consent: true
    allow_data_export: true
  
  reminders:
    medication: []
    hydration_interval_hours: 2
    movement_interval_hours: 1
  
  crisis_support:
    emergency_contacts: []
    enable_crisis_detection: true
```

## Usage Examples

### Setting Up Daily Routine

```python
# Via API
POST /api/routine
{
  "type": "morning",
  "time": "08:00",
  "activities": [
    "greeting",
    "mood_check",
    "medication_reminder",
    "daily_affirmation"
  ]
}
```

### Mood Tracking

```python
# Via API
POST /api/mood
{
  "mood": "anxious",
  "intensity": 7,
  "notes": "Work presentation today",
  "timestamp": "2024-12-17T10:30:00Z"
}

# Response includes:
{
  "mood": "anxious",
  "response": "I understand presentations can be stressful. Would you like to practice some breathing exercises together?",
  "suggestions": ["breathing_exercise", "positive_affirmation", "talk_it_out"]
}
```

### Emergency Contact Configuration

```python
# Via API
POST /api/emergency_contacts
{
  "contacts": [
    {
      "name": "Therapist",
      "phone": "+1-555-0123",
      "relationship": "professional"
    },
    {
      "name": "Best Friend",
      "phone": "+1-555-0456",
      "relationship": "friend"
    }
  ]
}
```

## Therapeutic Approaches

The robot incorporates principles from:

1. **Cognitive Behavioral Therapy (CBT)**
   - Helps identify negative thought patterns
   - Suggests cognitive reframing

2. **Dialectical Behavior Therapy (DBT)**
   - Teaches distress tolerance
   - Emotion regulation techniques

3. **Mindfulness-Based Stress Reduction (MBSR)**
   - Present-moment awareness
   - Non-judgmental observation

4. **Positive Psychology**
   - Gratitude practices
   - Strength identification
   - Positive affirmations

## Customization

### Personality Settings

Users can customize the robot's personality:
- **Tone**: Formal, casual, playful
- **Verbosity**: Concise, moderate, detailed
- **Proactivity**: Passive (responds only), moderate, active (initiates check-ins)

### Voice & Appearance

- Custom voice settings (pitch, speed)
- Face expression intensity
- Color themes for mood states

## Integration with Professional Care

### Therapist Dashboard (Future Feature)

With user consent, therapists can:
- View mood trends (anonymized)
- Suggest conversation topics
- Configure therapeutic exercises
- Monitor engagement levels

### Data Export for Clinical Use

Users can export data for sharing with healthcare providers:
```bash
# Export mood data
GET /api/export/mood?format=csv&start_date=2024-01-01&end_date=2024-12-31

# Export conversation summaries
GET /api/export/conversations?format=pdf&anonymize=true
```

## Safety Features

1. **Crisis Detection**: Identifies concerning language patterns
2. **Professional Referral**: Always recommends professional help when appropriate
3. **Boundaries**: Clear that robot is a tool, not a therapist
4. **Resource Directory**: Built-in mental health resources

## Limitations

**Important Disclaimers:**

⚠️ This robot is NOT a replacement for professional mental health care
⚠️ Does not diagnose mental health conditions
⚠️ Does not prescribe treatments
⚠️ Not for emergency situations - call 911 or local emergency services

**Recommended Use:**
- Supplemental support between therapy sessions
- Daily wellness companion
- Routine and habit assistance
- Emotional support for everyday stress

## Research & Evidence Base

This design is informed by:
- Companion robot research (PARO, ElliQ)
- Digital mental health interventions
- Human-Computer Interaction studies
- Clinical psychology best practices

## Future Enhancements

1. **Biometric Integration**: Heart rate, sleep tracking
2. **Group Support**: Connect users with similar experiences (anonymously)
3. **Professional Integration**: Secure therapist portal
4. **AI Emotion Recognition**: Computer vision for facial expression analysis
5. **Adaptive Therapy**: Personalized therapeutic approaches

## Getting Started

See [MENTAL_HEALTH_SETUP.md](MENTAL_HEALTH_SETUP.md) for detailed setup instructions specific to mental health and personal assistance features.

## Support & Resources

- **User Guide**: [docs/user_guide.md](../docs/user_guide.md)
- **Safety Information**: [SAFETY.md](SAFETY.md)
- **Privacy Policy**: [PRIVACY.md](PRIVACY.md)
- **Crisis Resources**: Built into the robot interface

## Feedback & Improvement

Users can provide feedback to help improve mental health features:
```bash
POST /api/feedback
{
  "feature": "mood_tracking",
  "rating": 5,
  "comments": "Very helpful for identifying patterns"
}
```

---

**Remember**: Mental health is a journey. This robot is here to support you, but professional help is invaluable. Please reach out to qualified mental health professionals when needed.
