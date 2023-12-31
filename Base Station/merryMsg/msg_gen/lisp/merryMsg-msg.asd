
(cl:in-package :asdf)

(defsystem "merryMsg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Msg" :depends-on ("_package_Msg"))
    (:file "_package_Msg" :depends-on ("_package"))
  ))