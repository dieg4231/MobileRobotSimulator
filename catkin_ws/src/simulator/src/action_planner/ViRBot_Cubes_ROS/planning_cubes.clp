;********************************************************
;*                                                      *
;*      planning_cubes.clp                              *
;*                                                      *
;*                                                      *
;*                      University of Mexico            *
;*                      Jesus Savage-Carmona            *
;*                      Adrian Revueltas                *
;*                                                      *
;*                      6/4/20                          *
;*                                                      *
;********************************************************

(defglobal ?*plan_time* = 30000)


(defrule start
	?f <- (start action-planning)
        =>
        (retract ?f)
        (printout t "ROS Starting action planner ROS")
	(assert (action-planner active))
)




(defrule secondary-goals 
	(action-planner active)
        (goal-stack ?num ?room ?zone $? ?block1 ?block2 $?)
	(not (final-stack ?num ?room ?zone $? ?block1 $?))
	=>
	(assert (goal (room ?room)(zone ?zone)(move ?block1) (on ?block2)) )
)



(defrule move-directly-stack
	(action-planner active)
        ?stack-1 <- (stack ?room1 ?zone1 ?block1 $?rest1)
        ?goal <- (goal-stack ?num2 ?room ?zone $?rest-goal ?block1)
	(not (final-stack ?num2 ?room ?zone $?rest ?block1))
	(plan (name ?name) (number ?num))
	(not (plan (name ?name) (number ?num1&:( > ?num1 ?num))))
	?f1 <- (item (type Objects) (name ?block1))
	;?f1 <- (item (type Objects) (name ?block1)(room ?room1)(zone ?zone1))
        =>
        (retract ?stack-1)
        (assert (stack ?room1 ?zone1 $?rest1))
        (assert (final-stack ?num2 ?room ?zone ?block1))
        (printout t ?block1 " will be moved onto free space in room " ?room crlf)

        (assert (plan (name ?name) (number (+ 1 ?num))(actions goto ?room1 ?zone1)(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 2 ?num))(actions find-object ?block1)(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 3 ?num))(actions mv ?block1 )(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 4 ?num))(actions grab ?block1 )(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 5 ?num))(actions goto ?room ?zone)(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 6 ?num))(actions find-object freespace)(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 7 ?num))(actions go freespace )(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 8 ?num))(actions drop ?block1 )(duration ?*plan_time*)) )
        ;(modify ?f1 (room ?room)(zone ?zone))	
	(assert (attempt (move ?block1)(room ?room)(zone ?zone)(on freespace)(number (+ 8 ?num) )))
)



(defrule move-directly
	(action-planner active)
        ?goal <- (goal (room ?room)(zone ?zone)(move ?block1) (on ?block2&:(neq ?block2 freespace)))
	?f <- (final-stack ?num2 ?room ?zone ?block2 $?rest2)
        ?stack-1 <- (stack ?room1 ?zone1 ?block1 $?rest1)
        (plan (name ?name) (number ?num))
        (not (plan (name ?name) (number ?num1&:( > ?num1 ?num))))
	;?f1 <- (item (type Objects) (name ?block1)(room ?room1)(zone ?zone1))
        =>
        (retract ?goal ?stack-1 ?f)
        (assert (stack ?room1 ?zone1 $?rest1))
        (assert (final-stack ?num2 ?room ?zone ?block1 ?block2 $?rest2))
        (printout t ?block1 " will be moved in front of " ?block2 "." crlf)

        (assert (plan (name ?name) (number (+ 1 ?num))(actions goto ?room1 ?zone1)(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 2 ?num))(actions find-object ?block1)(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 3 ?num))(actions mv ?block1 )(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 4 ?num))(actions grab ?block1 )(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 5 ?num))(actions goto ?room ?zone)(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 6 ?num))(actions find-object ?block2)(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 7 ?num))(actions go ?block2 )(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 8 ?num))(actions drop ?block1 )(duration ?*plan_time*)) )
        ;(modify ?f1 (room ?room)(zone ?zone))	
        (assert (attempt (move ?block1)(room ?room)(zone ?zone)(on ?block2)(number (+ 8 ?num) )))
)



(defrule move-to-free-space
	(declare (salience 100))
	(action-planner active)
        ?goal <- (goal (move ?block1) (on freespace))
	(item (type Objects) (name ?block1)(room ?room1)(zone ?zone1))
	(plan (name ?name) (number ?num))
        (not (plan (name ?name) (number ?num1&:( > ?num1 ?num))))
        =>
        (retract ?goal)
        (assert (stack ?room1 ?zone1 ?block1))
        (printout t ?block1 " will be moved onto free space in room " ?room1 crlf)
        (assert (plan (name ?name) (number (+ 1 ?num))(actions goto ?room1 ?zone1)(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 2 ?num))(actions find-object ?block1)(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 3 ?num))(actions mv ?block1)(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 4 ?num))(actions grab ?block1 )(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 5 ?num))(actions find-object freespace)(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 6 ?num))(actions go freespace)(duration ?*plan_time*)) )
        (assert (plan (name ?name) (number (+ 7 ?num))(actions drop ?block1 )(duration ?*plan_time*)) )
	(assert (attempt (move ?block1)(room ?room1)(zone ?zone1)(on freespace)(number (+ 7 ?num)) ))
)




(defrule clear-upper-block
	(action-planner active)
        (goal-stack ?num ?room ?zone $? ?block1 ?block2 $?)
        ?f <- (stack ?r ?z ?top $?rest1 ?block1 $?rest2)
        =>
	(retract ?f)
        (assert (goal (move ?top)(room ?r)(zone ?z)(on freespace)))
	(assert (stack ?r ?z $?rest1 ?block1 $?rest2))
)


(defrule clear-lower-block
	(action-planner active)
        (goal-stack ?num ?room ?zone $? ?block1 ?block2 $?)
        ?f <- (stack ?r ?z ?top  $?rest1 ?block2 $?rest2)
        =>
	(retract ?f)
        (assert (goal (move ?top)(room ?r)(zone ?z)(on freespace)))
	(assert (stack ?r ?z $?rest1 ?block2 $?rest2))
)


(defrule finish-plan
	(declare (salience -10000))
        (plan (name ?name) (number ?num&:(neq ?num 0)))
        (not (plan (name ?name) (number ?num1&:( > ?num1 ?num))))
        ?f <- (plan (name ?name) (number 0))
        =>
	(retract ?f)
	(assert (finish-planner ?name ?num))
)


(defrule accomplish-plan
	(declare (salience 100))
	(plan (name ?name) (number ?num) (status accomplished))
	?f <- (attempt (move ?block1) (on ?block2) (room ?deposit) (zone ?zone) (number ?num)(status ?status&:(neq ?status finished)))
	?f1 <- (item (type Objects) (name ?block1)) 
        =>
        (printout t ?block1 " was moved on " ?block2 "." crlf)
	(modify ?f (room ?deposit) (zone ?zone)(status finished))
	(modify ?f1 (room ?deposit) (zone ?zone))
)


