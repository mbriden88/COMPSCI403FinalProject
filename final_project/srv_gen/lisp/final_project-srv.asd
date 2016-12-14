
(cl:in-package :asdf)

(defsystem "final_project-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "GetCommandVelSrv" :depends-on ("_package_GetCommandVelSrv"))
    (:file "_package_GetCommandVelSrv" :depends-on ("_package"))
  ))