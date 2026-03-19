
(cl:in-package :asdf)

(defsystem "lidar_cluster-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ClusterCone" :depends-on ("_package_ClusterCone"))
    (:file "_package_ClusterCone" :depends-on ("_package"))
    (:file "ClusterCones" :depends-on ("_package_ClusterCones"))
    (:file "_package_ClusterCones" :depends-on ("_package"))
  ))