
(cl:in-package :asdf)

(defsystem "pet_robot_ros-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GetAffirmation" :depends-on ("_package_GetAffirmation"))
    (:file "_package_GetAffirmation" :depends-on ("_package"))
    (:file "LogMood" :depends-on ("_package_LogMood"))
    (:file "_package_LogMood" :depends-on ("_package"))
    (:file "SetEmotion" :depends-on ("_package_SetEmotion"))
    (:file "_package_SetEmotion" :depends-on ("_package"))
  ))