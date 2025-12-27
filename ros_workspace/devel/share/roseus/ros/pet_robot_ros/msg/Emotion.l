;; Auto-generated. Do not edit!


(when (boundp 'pet_robot_ros::Emotion)
  (if (not (find-package "PET_ROBOT_ROS"))
    (make-package "PET_ROBOT_ROS"))
  (shadow 'Emotion (find-package "PET_ROBOT_ROS")))
(unless (find-package "PET_ROBOT_ROS::EMOTION")
  (make-package "PET_ROBOT_ROS::EMOTION"))

(in-package "ROS")
;;//! \htmlinclude Emotion.msg.html


(defclass pet_robot_ros::Emotion
  :super ros::object
  :slots (_emotion _intensity _timestamp ))

(defmethod pet_robot_ros::Emotion
  (:init
   (&key
    ((:emotion __emotion) "")
    ((:intensity __intensity) 0.0)
    ((:timestamp __timestamp) (instance ros::time :init))
    )
   (send-super :init)
   (setq _emotion (string __emotion))
   (setq _intensity (float __intensity))
   (setq _timestamp __timestamp)
   self)
  (:emotion
   (&optional __emotion)
   (if __emotion (setq _emotion __emotion)) _emotion)
  (:intensity
   (&optional __intensity)
   (if __intensity (setq _intensity __intensity)) _intensity)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:serialization-length
   ()
   (+
    ;; string _emotion
    4 (length _emotion)
    ;; float32 _intensity
    4
    ;; time _timestamp
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _emotion
       (write-long (length _emotion) s) (princ _emotion s)
     ;; float32 _intensity
       (sys::poke _intensity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; time _timestamp
       (write-long (send _timestamp :sec) s) (write-long (send _timestamp :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _emotion
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _emotion (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _intensity
     (setq _intensity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; time _timestamp
     (send _timestamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _timestamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get pet_robot_ros::Emotion :md5sum-) "e7af84fb54cd74beec900c75b72aa6ef")
(setf (get pet_robot_ros::Emotion :datatype-) "pet_robot_ros/Emotion")
(setf (get pet_robot_ros::Emotion :definition-)
      "# Emotion message for robot's current emotional state
string emotion  # happy, sad, anxious, stressed, tired, angry, neutral, crisis
float32 intensity  # 0.0 to 1.0
time timestamp

")



(provide :pet_robot_ros/Emotion "e7af84fb54cd74beec900c75b72aa6ef")


