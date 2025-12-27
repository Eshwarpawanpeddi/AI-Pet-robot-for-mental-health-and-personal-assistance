;; Auto-generated. Do not edit!


(when (boundp 'pet_robot_ros::SensorData)
  (if (not (find-package "PET_ROBOT_ROS"))
    (make-package "PET_ROBOT_ROS"))
  (shadow 'SensorData (find-package "PET_ROBOT_ROS")))
(unless (find-package "PET_ROBOT_ROS::SENSORDATA")
  (make-package "PET_ROBOT_ROS::SENSORDATA"))

(in-package "ROS")
;;//! \htmlinclude SensorData.msg.html


(defclass pet_robot_ros::SensorData
  :super ros::object
  :slots (_touch_detected _distance _temperature _timestamp ))

(defmethod pet_robot_ros::SensorData
  (:init
   (&key
    ((:touch_detected __touch_detected) nil)
    ((:distance __distance) 0.0)
    ((:temperature __temperature) 0.0)
    ((:timestamp __timestamp) (instance ros::time :init))
    )
   (send-super :init)
   (setq _touch_detected __touch_detected)
   (setq _distance (float __distance))
   (setq _temperature (float __temperature))
   (setq _timestamp __timestamp)
   self)
  (:touch_detected
   (&optional __touch_detected)
   (if __touch_detected (setq _touch_detected __touch_detected)) _touch_detected)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:temperature
   (&optional __temperature)
   (if __temperature (setq _temperature __temperature)) _temperature)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:serialization-length
   ()
   (+
    ;; bool _touch_detected
    1
    ;; float32 _distance
    4
    ;; float32 _temperature
    4
    ;; time _timestamp
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _touch_detected
       (if _touch_detected (write-byte -1 s) (write-byte 0 s))
     ;; float32 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _temperature
       (sys::poke _temperature (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; time _timestamp
       (write-long (send _timestamp :sec) s) (write-long (send _timestamp :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _touch_detected
     (setq _touch_detected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _distance
     (setq _distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _temperature
     (setq _temperature (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; time _timestamp
     (send _timestamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _timestamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get pet_robot_ros::SensorData :md5sum-) "5a89da8b484867ca2bc71805d9719a4d")
(setf (get pet_robot_ros::SensorData :datatype-) "pet_robot_ros/SensorData")
(setf (get pet_robot_ros::SensorData :definition-)
      "# Sensor data from robot
bool touch_detected
float32 distance  # cm
float32 temperature  # Celsius
time timestamp

")



(provide :pet_robot_ros/SensorData "5a89da8b484867ca2bc71805d9719a4d")


