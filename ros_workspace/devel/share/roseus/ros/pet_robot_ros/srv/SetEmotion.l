;; Auto-generated. Do not edit!


(when (boundp 'pet_robot_ros::SetEmotion)
  (if (not (find-package "PET_ROBOT_ROS"))
    (make-package "PET_ROBOT_ROS"))
  (shadow 'SetEmotion (find-package "PET_ROBOT_ROS")))
(unless (find-package "PET_ROBOT_ROS::SETEMOTION")
  (make-package "PET_ROBOT_ROS::SETEMOTION"))
(unless (find-package "PET_ROBOT_ROS::SETEMOTIONREQUEST")
  (make-package "PET_ROBOT_ROS::SETEMOTIONREQUEST"))
(unless (find-package "PET_ROBOT_ROS::SETEMOTIONRESPONSE")
  (make-package "PET_ROBOT_ROS::SETEMOTIONRESPONSE"))

(in-package "ROS")





(defclass pet_robot_ros::SetEmotionRequest
  :super ros::object
  :slots (_emotion _intensity ))

(defmethod pet_robot_ros::SetEmotionRequest
  (:init
   (&key
    ((:emotion __emotion) "")
    ((:intensity __intensity) 0.0)
    )
   (send-super :init)
   (setq _emotion (string __emotion))
   (setq _intensity (float __intensity))
   self)
  (:emotion
   (&optional __emotion)
   (if __emotion (setq _emotion __emotion)) _emotion)
  (:intensity
   (&optional __intensity)
   (if __intensity (setq _intensity __intensity)) _intensity)
  (:serialization-length
   ()
   (+
    ;; string _emotion
    4 (length _emotion)
    ;; float32 _intensity
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _emotion
       (write-long (length _emotion) s) (princ _emotion s)
     ;; float32 _intensity
       (sys::poke _intensity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _emotion
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _emotion (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _intensity
     (setq _intensity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass pet_robot_ros::SetEmotionResponse
  :super ros::object
  :slots (_success _message ))

(defmethod pet_robot_ros::SetEmotionResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass pet_robot_ros::SetEmotion
  :super ros::object
  :slots ())

(setf (get pet_robot_ros::SetEmotion :md5sum-) "b4511e9a0d7a1edc71467ea13a9b50a5")
(setf (get pet_robot_ros::SetEmotion :datatype-) "pet_robot_ros/SetEmotion")
(setf (get pet_robot_ros::SetEmotion :request) pet_robot_ros::SetEmotionRequest)
(setf (get pet_robot_ros::SetEmotion :response) pet_robot_ros::SetEmotionResponse)

(defmethod pet_robot_ros::SetEmotionRequest
  (:response () (instance pet_robot_ros::SetEmotionResponse :init)))

(setf (get pet_robot_ros::SetEmotionRequest :md5sum-) "b4511e9a0d7a1edc71467ea13a9b50a5")
(setf (get pet_robot_ros::SetEmotionRequest :datatype-) "pet_robot_ros/SetEmotionRequest")
(setf (get pet_robot_ros::SetEmotionRequest :definition-)
      "# Request to set robot's emotion
string emotion  # happy, sad, anxious, stressed, tired, angry, neutral
float32 intensity  # 0.0 to 1.0
---
# Response
bool success
string message

")

(setf (get pet_robot_ros::SetEmotionResponse :md5sum-) "b4511e9a0d7a1edc71467ea13a9b50a5")
(setf (get pet_robot_ros::SetEmotionResponse :datatype-) "pet_robot_ros/SetEmotionResponse")
(setf (get pet_robot_ros::SetEmotionResponse :definition-)
      "# Request to set robot's emotion
string emotion  # happy, sad, anxious, stressed, tired, angry, neutral
float32 intensity  # 0.0 to 1.0
---
# Response
bool success
string message

")



(provide :pet_robot_ros/SetEmotion "b4511e9a0d7a1edc71467ea13a9b50a5")


