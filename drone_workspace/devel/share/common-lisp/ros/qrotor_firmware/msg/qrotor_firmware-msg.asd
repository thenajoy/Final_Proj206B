
(cl:in-package :asdf)

(defsystem "qrotor_firmware-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AttitudeState" :depends-on ("_package_AttitudeState"))
    (:file "_package_AttitudeState" :depends-on ("_package"))
    (:file "Log" :depends-on ("_package_Log"))
    (:file "_package_Log" :depends-on ("_package"))
    (:file "RCRaw" :depends-on ("_package_RCRaw"))
    (:file "_package_RCRaw" :depends-on ("_package"))
  ))