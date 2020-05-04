
;************************************************
;*                                              *
;*      ros_clips.clp                           *
;*                                              *
;*      Jesus Savage                            *
;*                                              *
;*              Bio-Robotics Laboratory         *
;*              UNAM, 2020                      *
;*                                              *
;*                                              *
;************************************************


(defrule clips-alive
        ?f <- (alive clips)
        =>
        (retract ?f)
        (printout t "ROS clips alive ROS")
)

(defrule send-ros
	(declare (salience 100))
	?f <-  (step ?n)
	?f1 <- (send-ROS ?system ?action ?command ?t ?num)
	=>
        (retract ?f ?f1)
        (printout t "ROS " ?system " " ?action " " ?command " " ?t " " ?num " ROS")
)

(defrule step-ros
	?f <- (step ?num)
	=>
	(retract ?f)
	(printout t "ROS step " ?num " ROS")
)

