;; Auto-generated. Do not edit!


(when (boundp 'pet_robot_ros::MoodLog)
  (if (not (find-package "PET_ROBOT_ROS"))
    (make-package "PET_ROBOT_ROS"))
  (shadow 'MoodLog (find-package "PET_ROBOT_ROS")))
(unless (find-package "PET_ROBOT_ROS::MOODLOG")
  (make-package "PET_ROBOT_ROS::MOODLOG"))

(in-package "ROS")
;;//! \htmlinclude MoodLog.msg.html


(defclass pet_robot_ros::MoodLog
  :super ros::object
  :slots (_mood _intensity _notes _timestamp ))

(defmethod pet_robot_ros::MoodLog
  (:init
   (&key
    ((:mood __mood) "")
    ((:intensity __intensity) 0)
    ((:notes __notes) "")
    ((:timestamp __timestamp) (instance ros::time :init))
    )
   (send-super :init)
   (setq _mood (string __mood))
   (setq _intensity (round __intensity))
   (setq _notes (string __notes))
   (setq _timestamp __timestamp)
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
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:serialization-length
   ()
   (+
    ;; string _mood
    4 (length _mood)
    ;; int32 _intensity
    4
    ;; string _notes
    4 (length _notes)
    ;; time _timestamp
    8
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
     ;; time _timestamp
       (write-long (send _timestamp :sec) s) (write-long (send _timestamp :nsec) s)
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
   ;; time _timestamp
     (send _timestamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _timestamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get pet_robot_ros::MoodLog :md5sum-) "4584f6b7d371bdba788c5cd148510811")
(setf (get pet_robot_ros::MoodLog :datatype-) "pet_robot_ros/MoodLog")
(setf (get pet_robot_ros::MoodLog :definition-)
      "# Mood log entry for mental health tracking
string mood  # anxious, sad, happy, stressed, tired, angry, neutral
int32 intensity  # 1-10 scale
string notes
time timestamp

")



(provide :pet_robot_ros/MoodLog "4584f6b7d371bdba788c5cd148510811")


