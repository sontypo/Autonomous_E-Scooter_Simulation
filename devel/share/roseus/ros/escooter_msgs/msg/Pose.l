;; Auto-generated. Do not edit!


(when (boundp 'escooter_msgs::Pose)
  (if (not (find-package "ESCOOTER_MSGS"))
    (make-package "ESCOOTER_MSGS"))
  (shadow 'Pose (find-package "ESCOOTER_MSGS")))
(unless (find-package "ESCOOTER_MSGS::POSE")
  (make-package "ESCOOTER_MSGS::POSE"))

(in-package "ROS")
;;//! \htmlinclude Pose.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass escooter_msgs::Pose
  :super ros::object
  :slots (_position _orientation ))

(defmethod escooter_msgs::Pose
  (:init
   (&key
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:orientation __orientation) (instance escooter_msgs::RPY :init))
    )
   (send-super :init)
   (setq _position __position)
   (setq _orientation __orientation)
   self)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:orientation
   (&rest __orientation)
   (if (keywordp (car __orientation))
       (send* _orientation __orientation)
     (progn
       (if __orientation (setq _orientation (car __orientation)))
       _orientation)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; escooter_msgs/RPY _orientation
    (send _orientation :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; escooter_msgs/RPY _orientation
       (send _orientation :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; escooter_msgs/RPY _orientation
     (send _orientation :deserialize buf ptr-) (incf ptr- (send _orientation :serialization-length))
   ;;
   self)
  )

(setf (get escooter_msgs::Pose :md5sum-) "f1b4a886328450f637af10145259080b")
(setf (get escooter_msgs::Pose :datatype-) "escooter_msgs/Pose")
(setf (get escooter_msgs::Pose :definition-)
      "geometry_msgs/Point position
escooter_msgs/RPY orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: escooter_msgs/RPY
float64 roll
float64 pitch
float64 yaw

")



(provide :escooter_msgs/Pose "f1b4a886328450f637af10145259080b")


