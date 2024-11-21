;; Auto-generated. Do not edit!


(when (boundp 'escooter_msgs::Joints)
  (if (not (find-package "ESCOOTER_MSGS"))
    (make-package "ESCOOTER_MSGS"))
  (shadow 'Joints (find-package "ESCOOTER_MSGS")))
(unless (find-package "ESCOOTER_MSGS::JOINTS")
  (make-package "ESCOOTER_MSGS::JOINTS"))

(in-package "ROS")
;;//! \htmlinclude Joints.msg.html


(defclass escooter_msgs::Joints
  :super ros::object
  :slots (_handle_bar _rotational_plate _front_wheel ))

(defmethod escooter_msgs::Joints
  (:init
   (&key
    ((:handle_bar __handle_bar) 0.0)
    ((:rotational_plate __rotational_plate) 0.0)
    ((:front_wheel __front_wheel) 0.0)
    )
   (send-super :init)
   (setq _handle_bar (float __handle_bar))
   (setq _rotational_plate (float __rotational_plate))
   (setq _front_wheel (float __front_wheel))
   self)
  (:handle_bar
   (&optional __handle_bar)
   (if __handle_bar (setq _handle_bar __handle_bar)) _handle_bar)
  (:rotational_plate
   (&optional __rotational_plate)
   (if __rotational_plate (setq _rotational_plate __rotational_plate)) _rotational_plate)
  (:front_wheel
   (&optional __front_wheel)
   (if __front_wheel (setq _front_wheel __front_wheel)) _front_wheel)
  (:serialization-length
   ()
   (+
    ;; float64 _handle_bar
    8
    ;; float64 _rotational_plate
    8
    ;; float64 _front_wheel
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _handle_bar
       (sys::poke _handle_bar (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rotational_plate
       (sys::poke _rotational_plate (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _front_wheel
       (sys::poke _front_wheel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _handle_bar
     (setq _handle_bar (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rotational_plate
     (setq _rotational_plate (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _front_wheel
     (setq _front_wheel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get escooter_msgs::Joints :md5sum-) "d3e5fbcf0da70ecde669dea5a74f6f7b")
(setf (get escooter_msgs::Joints :datatype-) "escooter_msgs/Joints")
(setf (get escooter_msgs::Joints :definition-)
      "float64 handle_bar
float64 rotational_plate
float64 front_wheel

")



(provide :escooter_msgs/Joints "d3e5fbcf0da70ecde669dea5a74f6f7b")


