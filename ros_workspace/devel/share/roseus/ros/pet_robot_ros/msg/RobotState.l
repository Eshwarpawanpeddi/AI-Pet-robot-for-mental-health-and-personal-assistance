;; Auto-generated. Do not edit!


(when (boundp 'pet_robot_ros::RobotState)
  (if (not (find-package "PET_ROBOT_ROS"))
    (make-package "PET_ROBOT_ROS"))
  (shadow 'RobotState (find-package "PET_ROBOT_ROS")))
(unless (find-package "PET_ROBOT_ROS::ROBOTSTATE")
  (make-package "PET_ROBOT_ROS::ROBOTSTATE"))

(in-package "ROS")
;;//! \htmlinclude RobotState.msg.html


(defclass pet_robot_ros::RobotState
  :super ros::object
  :slots (_emotion _is_speaking _is_listening _battery_level _sensor_data ))

(defmethod pet_robot_ros::RobotState
  (:init
   (&key
    ((:emotion __emotion) (instance pet_robot_ros::Emotion :init))
    ((:is_speaking __is_speaking) nil)
    ((:is_listening __is_listening) nil)
    ((:battery_level __battery_level) 0)
    ((:sensor_data __sensor_data) (instance pet_robot_ros::SensorData :init))
    )
   (send-super :init)
   (setq _emotion __emotion)
   (setq _is_speaking __is_speaking)
   (setq _is_listening __is_listening)
   (setq _battery_level (round __battery_level))
   (setq _sensor_data __sensor_data)
   self)
  (:emotion
   (&rest __emotion)
   (if (keywordp (car __emotion))
       (send* _emotion __emotion)
     (progn
       (if __emotion (setq _emotion (car __emotion)))
       _emotion)))
  (:is_speaking
   (&optional __is_speaking)
   (if __is_speaking (setq _is_speaking __is_speaking)) _is_speaking)
  (:is_listening
   (&optional __is_listening)
   (if __is_listening (setq _is_listening __is_listening)) _is_listening)
  (:battery_level
   (&optional __battery_level)
   (if __battery_level (setq _battery_level __battery_level)) _battery_level)
  (:sensor_data
   (&rest __sensor_data)
   (if (keywordp (car __sensor_data))
       (send* _sensor_data __sensor_data)
     (progn
       (if __sensor_data (setq _sensor_data (car __sensor_data)))
       _sensor_data)))
  (:serialization-length
   ()
   (+
    ;; pet_robot_ros/Emotion _emotion
    (send _emotion :serialization-length)
    ;; bool _is_speaking
    1
    ;; bool _is_listening
    1
    ;; int32 _battery_level
    4
    ;; pet_robot_ros/SensorData _sensor_data
    (send _sensor_data :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; pet_robot_ros/Emotion _emotion
       (send _emotion :serialize s)
     ;; bool _is_speaking
       (if _is_speaking (write-byte -1 s) (write-byte 0 s))
     ;; bool _is_listening
       (if _is_listening (write-byte -1 s) (write-byte 0 s))
     ;; int32 _battery_level
       (write-long _battery_level s)
     ;; pet_robot_ros/SensorData _sensor_data
       (send _sensor_data :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; pet_robot_ros/Emotion _emotion
     (send _emotion :deserialize buf ptr-) (incf ptr- (send _emotion :serialization-length))
   ;; bool _is_speaking
     (setq _is_speaking (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _is_listening
     (setq _is_listening (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _battery_level
     (setq _battery_level (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; pet_robot_ros/SensorData _sensor_data
     (send _sensor_data :deserialize buf ptr-) (incf ptr- (send _sensor_data :serialization-length))
   ;;
   self)
  )

(setf (get pet_robot_ros::RobotState :md5sum-) "23f43ffc60bb2cce618de7427a61ab01")
(setf (get pet_robot_ros::RobotState :datatype-) "pet_robot_ros/RobotState")
(setf (get pet_robot_ros::RobotState :definition-)
      "# Complete robot state
Emotion emotion
bool is_speaking
bool is_listening
int32 battery_level  # 0-100
SensorData sensor_data

================================================================================
MSG: pet_robot_ros/Emotion
# Emotion message for robot's current emotional state
string emotion  # happy, sad, anxious, stressed, tired, angry, neutral, crisis
float32 intensity  # 0.0 to 1.0
time timestamp

================================================================================
MSG: pet_robot_ros/SensorData
# Sensor data from robot
bool touch_detected
float32 distance  # cm
float32 temperature  # Celsius
time timestamp

")



(provide :pet_robot_ros/RobotState "23f43ffc60bb2cce618de7427a61ab01")


