
(defrule obs-dest-ros
        ?f <- (ros step ?num obs ?obs dest ?dest advance ?adv twist ?twist)
        =>
        (retract ?f)
	(assert (dest ?num ?obs ?dest ?adv ?twist))
	(bind ?num (* ?num 1))
        ;(printout t "Modified number " ?num)
)



(defrule no-obstacle-light-up-left-ros
	?f <- (dest ?num 0 1 ?adv ?twist)
	=>
	(retract ?f)
        (bind ?command (str-cat "" ?num " left_45 forward 0.5 " ?adv " " ?twist))
        (assert (send-ros ACT-PLN clips_ros ?command 4000 4))
	;(assert (movement ?num left_45 forward 0.5))
)


(defrule no-obstacle-light-down-left-ros
        ?f <- (dest ?num 0 2 ?adv ?twist)
        =>
        (retract ?f)
        (bind ?command (str-cat "" ?num " left_135 forward 0.5 " ?adv " " ?twist ))
        (assert (send-ros ACT-PLN clips_ros ?command 4000 4))
        ;(assert (movement ?num left_135 forward 0.5))
)

(defrule no-obstacle-light-down-right-ros
        ?f <- (dest ?num 0 3 ?adv ?twist)
        =>
        (retract ?f)
        (bind ?command (str-cat "" ?num " right_135 forward 0.5 " ?adv " " ?twist))
        (assert (send-ros ACT-PLN clips_ros ?command 4000 4))
        ;(assert (movement ?num right_135 forward 0.5))
)

(defrule no-obstacle-light-up-right-ros
	?f <- (dest ?num 0 4 ?adv ?twist)
	=>
	(retract ?f)
        (bind ?command (str-cat "" ?num " right_45 forward 0.5 " ?adv " " ?twist))
        (assert (send-ros ACT-PLN clips_ros ?command 4000 4))
	;(assert (movement ?num right_45 forward 0.5))
)

(defrule no-obstacle-light-front-ros
	?f <- (dest ?num 0 0 ?adv ?twist)
	=>
	(retract ?f)
        (bind ?command (str-cat "" ?num " none forward 0.5 " ?adv " " ?twist))
        (assert (send-ros ACT-PLN clips_ros ?command 4000 4))
	;(assert (movement ?num none forward 0.5))
)


;;;;;;;;;;;;; send message from CLIPS to ROS
(defrule send-to-ros
        ?f <- (send_data_to_ros ?data ?t)
	?f1 <- (send_data active)
        =>
        (bind ?command (str-cat "" ?data ""))
        (assert (send-ros ACT-PLN clips_ros ?command ?t 4))
)

(defrule sended-to-ros-succeded
        ?f <- (received ?sender command clips_ros $?spc 1)
	;?f1 <- (send_data active)
        =>
        (retract ?f)
        ;(retract ?f1)
	(printout t "ROS succeded response " $?spc " " crlf)
)

(defrule sended-to-ros-no-succeded
        ?f <- (received ?sender command spg_say $?spc 0)
        ?f1 <- (send_data active)
        =>
        (retract ?f)
	(retract ?f1)
	(printout t "ROS response " $?spc " " crlf)
        (assert (send_data active))
)
;;;;;;;;;;;;;;;;
