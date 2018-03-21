
(cl:in-package :asdf)

(defsystem "villa_surface_detectors-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :visualization_msgs-msg
)
  :components ((:file "_package")
    (:file "DetectHorizontalPlanes" :depends-on ("_package_DetectHorizontalPlanes"))
    (:file "_package_DetectHorizontalPlanes" :depends-on ("_package"))
  ))