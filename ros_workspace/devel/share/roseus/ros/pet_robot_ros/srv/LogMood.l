;; Auto-generated. Do not edit!


(when (boundp 'pet_robot_ros::LogMood)
  (if (not (find-package "PET_ROBOT_ROS"))
    (make-package "PET_ROBOT_ROS"))
  (shadow 'LogMood (find-package "PET_ROBOT_ROS")))
(unless (find-package "PET_ROBOT_ROS::LOGMOOD")
  (make-package "PET_ROBOT_ROS::LOGMOOD"))
(unless (find-package "PET_ROBOT_ROS::LOGMOODREQUEST")
  (make-package "PET_ROBOT_ROS::LOGMOODREQUEST"))
(unless (find-package "PET_ROBOT_ROS::LOGMOODRESPONSE")
  (make-package "PET_ROBOT_ROS::LOGMOODRESPONSE"))

(in-package "ROS")





(defclass pet_robot_ros::LogMoodRequest
  :super ros::object
  :slots (_mood _intensity _notes ))

(defmethod pet_robot_ros::LogMoodRequest
  (:init
   (&key
    ((:mood __mood) "")
    ((:intensity __intensity) 0)
    ((:notes __notes) "")
    )
   (send-super :init)
   (setq _mood (string __mood))
   (setq _intensity (round __intensity))
   (setq _notes (string __notes))
   self)
  (:mood
   (&optional __mood)
   (if __mood (setq _mood __mood)) _mood)
  (:intensity
   (&optional __intensity)
   (if __intensity (setq _intensity __intensity)) _intensity)
  (:notes
   (&optional __notes)
   (if __notes (setq _notes __notes)) _notes)
  (:serialization-length
   ()
   (+
    ;; string _mood
    4 (length _mood)
    ;; int32 _intensity
    4
    ;; string _notes
    4 (length _notes)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _mood
       (write-long (length _mood) s) (princ _mood s)
     ;; int32 _intensity
       (write-long _intensity s)
     ;; string _notes
       (write-long (length _notes) s) (princ _notes s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _mood
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _mood (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _intensity
     (setq _intensity (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _notes
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _notes (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass pet_robot_ros::LogMoodResponse
  :super ros::object
  :slots (_success _response _suggestions ))

(defmethod pet_robot_ros::LogMoodResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:response __response) "")
    ((:suggestions __suggestions) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _success __success)
   (setq _response (string __response))
   (setq _suggestions __suggestions)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:response
   (&optional __response)
   (if __response (setq _response __response)) _response)
  (:suggestions
   (&optional __suggestions)
   (if __suggestions (setq _suggestions __suggestions)) _suggestions)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _response
    4 (length _response)
    ;; string[] _suggestions
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _suggestions)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _response
       (write-long (length _response) s) (princ _response s)
     ;; string[] _suggestions
     (write-long (length _suggestions) s)
     (dolist (elem _suggestions)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _response
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _response (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string[] _suggestions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _suggestions (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _suggestions i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass pet_robot_ros::LogMood
  :super ros::object
  :slots ())

(setf (get pet_robot_ros::LogMood :md5sum-) "b43c763a2a7bf67b61c6235ef921779c")
(setf (get pet_robot_ros::LogMood :datatype-) "pet_robot_ros/LogMood")
(setf (get pet_robot_ros::LogMood :request) pet_robot_ros::LogMoodRequest)
(setf (get pet_robot_ros::LogMood :response) pet_robot_ros::LogMoodResponse)

(defmethod pet_robot_ros::LogMoodRequest
  (:response () (instance pet_robot_ros::LogMoodResponse :init)))

(setf (get pet_robot_ros::LogMoodRequest :md5sum-) "b43c763a2a7bf67b61c6235ef921779c")
(setf (get pet_robot_ros::LogMoodRequest :datatype-) "pet_robot_ros/LogMoodRequest")
(setf (get pet_robot_ros::LogMoodRequest :definition-)
      "# Request to log mood
string mood  # anxious, sad, happy, stressed, tired, angry, neutral
int32 intensity  # 1-10 scale
string notes
---
# Response
bool success
string response  # Supportive response from robot
string[] suggestions  # Suggested coping activities

")

(setf (get pet_robot_ros::LogMoodResponse :md5sum-) "b43c763a2a7bf67b61c6235ef921779c")
(setf (get pet_robot_ros::LogMoodResponse :datatype-) "pet_robot_ros/LogMoodResponse")
(setf (get pet_robot_ros::LogMoodResponse :definition-)
      "# Request to log mood
string mood  # anxious, sad, happy, stressed, tired, angry, neutral
int32 intensity  # 1-10 scale
string notes
---
# Response
bool success
string response  # Supportive response from robot
string[] suggestions  # Suggested coping activities

")



(provide :pet_robot_ros/LogMood "b43c763a2a7bf67b61c6235ef921779c")


