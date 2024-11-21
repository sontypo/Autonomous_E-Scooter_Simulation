
(cl:in-package :asdf)

(defsystem "escooter_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "Joints" :depends-on ("_package_Joints"))
    (:file "_package_Joints" :depends-on ("_package"))
    (:file "Pose" :depends-on ("_package_Pose"))
    (:file "_package_Pose" :depends-on ("_package"))
    (:file "RPY" :depends-on ("_package_RPY"))
    (:file "_package_RPY" :depends-on ("_package"))
  ))