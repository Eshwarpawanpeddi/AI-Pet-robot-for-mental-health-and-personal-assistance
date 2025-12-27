;; Auto-generated. Do not edit!


(when (boundp 'pet_robot_ros::GetAffirmation)
  (if (not (find-package "PET_ROBOT_ROS"))
    (make-package "PET_ROBOT_ROS"))
  (shadow 'GetAffirmation (find-package "PET_ROBOT_ROS")))
(unless (find-package "PET_ROBOT_ROS::GETAFFIRMATION")
  (make-package "PET_ROBOT_ROS::GETAFFIRMATION"))
(unless (find-package "PET_ROBOT_ROS::GETAFFIRMATIONREQUEST")
  (make-package "PET_ROBOT_ROS::GETAFFIRMATIONREQUEST"))
(unless (find-package "PET_ROBOT_ROS::GETAFFIRMATIONRESPONSE")
  (make-package "PET_ROBOT_ROS::GETAFFIRMATIONRESPONSE"))

(in-package "ROS")





(defclass pet_robot_ros::GetAffirmationRequest
  :super ros::object
  :slots ())

(defmethod pet_robot_ros::GetAffirmationRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass pet_robot_ros::GetAffirmationResponse
  :super ros::object
  :slots (_affirmation _timestamp ))

(defmethod pet_robot_ros::GetAffirmationResponse
  (:init
   (&key
    ((:affirmation __affirmation) "")
    ((:timestamp __timestamp) (instance ros::time :init))
    )
   (send-super :init)
   (setq _affirmation (string __affirmation))
   (setq _timestamp __timestamp)
   self)
  (:affirmation
   (&optional __affirmation)
   (if __affirmation (setq _affirmation __affirmation)) _affirmation)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:serialization-length
   ()
   (+
    ;; string _affirmation
    4 (length _affirmation)
    ;; time _timestamp
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _affirmation
       (write-long (length _affirmation) s) (princ _affirmation s)
     ;; time _timestamp
       (write-long (send _timestamp :sec) s) (write-long (send _timestamp :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _affirmation
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _affirmation (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; time _timestamp
     (send _timestamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _timestamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass pet_robot_ros::GetAffirmation
  :super ros::object
  :slots ())

(setf (get pet_robot_ros::GetAffirmation :md5sum-) "176b3ebe98603e26ffc647c2f65b7d30")
(setf (get pet_robot_ros::GetAffirmation :datatype-) "pet_robot_ros/GetAffirmation")
(setf (get pet_robot_ros::GetAffirmation :request) pet_robot_ros::GetAffirmationRequest)
(setf (get pet_robot_ros::GetAffirmation :response) pet_robot_ros::GetAffirmationResponse)

(defmethod pet_robot_ros::GetAffirmationRequest
  (:response () (instance pet_robot_ros::GetAffirmationResponse :init)))

(setf (get pet_robot_ros::GetAffirmationRequest :md5sum-) "176b3ebe98603e26ffc647c2f65b7d30")
(setf (get pet_robot_ros::GetAffirmationRequest :datatype-) "pet_robot_ros/GetAffirmationRequest")
(setf (get pet_robot_ros::GetAffirmationRequest :definition-)
      "# Request a positive affirmation
---
# Response
string affirmation
time timestamp

")

(setf (get pet_robot_ros::GetAffirmationResponse :md5sum-) "176b3ebe98603e26ffc647c2f65b7d30")
(setf (get pet_robot_ros::GetAffirmationResponse :datatype-) "pet_robot_ros/GetAffirmationResponse")
(setf (get pet_robot_ros::GetAffirmationResponse :definition-)
      "# Request a positive affirmation
---
# Response
string affirmation
time timestamp

")



(provide :pet_robot_ros/GetAffirmation "176b3ebe98603e26ffc647c2f65b7d30")


