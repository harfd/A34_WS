
(cl:in-package :asdf)

(defsystem "coordSystem-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "imu_state" :depends-on ("_package_imu_state"))
    (:file "_package_imu_state" :depends-on ("_package"))
  ))