;************************************************
;*                                              *
;*      exe_plan.clp	                        *
;*                                              *
;*                                              *
;*                                              *
;*                      J.Savage, UNAM          *
;*                                              *
;*                      1/5/20                  *
;*                                              *
;************************************************




(defrule exe-plan
	(finish-planner ?name ?num_pln)
        ?f <- (plan (name ?name) (number ?num&:(neq num 0))(status inactive))
        (not (plan (name ?name) (number ?num1&:( < ?num1 ?num))(status ?status&:(or (eq ?status active) (eq ?status inactive  )))) )
        =>
	(modify ?f (status active))
)


(defrule exe-plan-find-object
        (plan (name ?name) (number ?num-pln)(status active)(actions find-object ?obj)(duration ?t))
 	;?f1 <- (item (name ?obj)(status ?x&:(neq ?x found)))
 	?f1 <- (item (name ?obj)(pose ?x ?y ?z))
        =>
	(bind ?command (str-cat "" ?obj " " ?x " " ?y " " ?z""))
        (assert (send-ROS ACT-PLN find_object ?command ?t 4))
)




(defrule exe-plan-found-object
        ?f <-  (answer ?sender command find_object ?block1 ?x ?y ?z ?arm 1)
 	?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find-object ?object))
	;?f3 <- (Arm (name ?arm))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        ;(modify ?f3 (status verify))
        ;(modify ?f1 (pose ?x ?y ?z) (status found));;;; modified for verify arm task		
        (modify ?f1 (pose ?x ?y ?z))		
)



(defrule exe-plan-goto
        (plan (name ?name) (number ?num-pln)(status active)(actions goto ?room ?zone)(duration ?t))
        ?f1 <- (Room (name ?room) (zone ?zone)(center ?x ?y ))
        =>
        (bind ?command (str-cat "" ?room " " ?zone " " ?x " " ?y ""))
        ;(bind ?command (str-cat "" ?room " " ?zone""))
	(printout t "goto " ?command  crlf)

        (assert (send-ROS ACT-PLN goto ?command ?t 4))
)




(defrule exe-plan-executed-goto
        ?f <-  (answer ?sender command goto ?room ?zone ?x ?y ?flg)
        ;?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions goto ?room ?zone))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
)


; The robot should go to an specif position
(defrule exe-plan-go
        (plan (name ?name) (number ?num-pln)(status active)(actions go ?obj)(duration ?t))
        ?f1 <- (item (name ?obj) (zone ?zone)(pose ?x ?y ?z))
        =>
        (bind ?command (str-cat "" ?zone " " ?x " " ?y " " ?z""))
        (printout t "go " ?command  crlf)
        (assert (send-ROS ACT-PLN go ?command ?t 4))
)


(defrule exe-plan-executed-go
        ?f <-  (answer ?sender command go ?x ?y ?z ?flg)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions go ?obj))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
)

; The robot moves the specified angle and distance
(defrule exe-plan-mv
        (plan (name ?name) (number ?num-pln)(status active)(actions mv ?obj)(duration ?t))
        ?f1 <- (item (name ?obj) (zone ?zone)(pose ?x ?y ?z))
        =>
        ;(bind ?command (str-cat "" ?distance " " ?angle""))
        (bind ?command (str-cat "" ?x " " ?y""))
        (printout t "mv " ?command  crlf)
        (assert (send-ROS ACT-PLN mv ?command ?t 4))
)


(defrule exe-plan-executed-mv
        ?f <-  (answer ?sender command mv ?distance ?angle ?flg)
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions mv ?obj))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
)




(defrule exe-plan-no-found-object
        ?f <-  (answer ?sender command find_object ?block1 ?x ?y ?z ?arm 0)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find-object ?object))
        =>
        (retract ?f)
        (modify ?f2 (status active))
)


(defrule exe-plan-move-actuator
        (plan (number ?num-pln)(status active)(actions move ?arm ?obj)(duration ?t))
 	(item (name ?obj) (pose ?x ?y ?z) )
        (Arm (name ?arm))
        =>
        ;(bind ?command (str-cat "" ?obj " " ?x " " ?y " " ?z ""))
        (bind ?command (str-cat "" ?obj ""))
        ;(bind ?command (str-cat "" ?obj " nil"""))
        (assert (send-ROS ACT-PLN move_actuator ?command ?t 4))
)

(defrule exe-plan-moved-actuator
        ?f <-  (answer ?sender command move_actuator ?object 1)
        ;?f <-  (answer ?sender command move_actuator ?object ?x ?y ?z ?id 1)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions move ?actuator ?object))
	;?f3 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        ;(retract ?f3)
)

;fix this later
(defrule exe-plan-no-moved-actuator
        ?f <-  (answer ?sender command move_actuator ?object ?x ?y ?z ?id 0)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions move ?actuator ?object))
        ;?f3 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f1 (name ?object))
)


(defrule exe-plan-grab-actuator
        (plan (name ?name) (number ?num-pln)(status active)(actions grab ?obj)(duration ?t))
        ?f1 <- (item (name ?obj))
        =>
        (bind ?command (str-cat "" ?obj ""))
        (assert (send-ROS ACT-PLN grab ?command ?t 4))
)


(defrule exe-plan-grabed-actuator
        ?f <-  (answer ?sender command grab ?object 1)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions grab ?object))
        ?f3 <- (Arm)
        =>
        (retract ?f)
	(modify ?f1 (status grabed)) 
        (modify ?f2 (status accomplished))
	(modify ?f3 (grasp ?object)) 
)


(defrule exe-plan-no-grabed-actuator
        ?f <-  (answer ?sender command grab ?actuator ?object 0)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions grab ?actuator ?object))
        ?f3 <- (item (name robot))
        ;?f4 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f1 (name ?object))
)


(defrule exe-plan-drop-actuator
        (plan (name ?name) (number ?num-pln)(status active)(actions drop ?obj)(duration ?t))
        ?f1 <- (item (name ?obj))
        ?f2 <- (Arm (grasp ?obj))
        =>
        (bind ?command (str-cat "" ?obj ""))
        (assert (send-ROS ACT-PLN drop ?command ?t 4))
)

( defrule exe-plan-droped-actuator
        ?f <-  (answer ?sender command drop ?object ?x ?y ?theta 1)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions drop ?object))
        ?f3 <- (Arm (grasp ?object))
        =>
        (retract ?f)
	(modify ?f1 (status droped)(pose ?x ?y ?theta))
        (modify ?f2 (status accomplished))
        (modify ?f3 (status nil) (grasp nil))
)

(defrule exe-plan-no-droped-actuator
        ?f <-  (answer ?sender command drop ?actuator ?object ?flag 0)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions drop ?actuator ?object))
        ?f3 <- (item (name robot))
        ;?f4 <- (wait plan ?name ?num-pln ?t)
        =>
        (retract ?f)
        (modify ?f1 (name ?object))
)

