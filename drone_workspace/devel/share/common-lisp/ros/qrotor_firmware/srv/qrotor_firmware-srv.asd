
(cl:in-package :asdf)

(defsystem "qrotor_firmware-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "FlatTrajectory" :depends-on ("_package_FlatTrajectory"))
    (:file "_package_FlatTrajectory" :depends-on ("_package"))
    (:file "Setpoint" :depends-on ("_package_Setpoint"))
    (:file "_package_Setpoint" :depends-on ("_package"))
  ))