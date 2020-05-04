
;************************************************
;*						*
;*	oracle.clp 				*
;*						*
;************************************************

(deffacts init-skts-blackboard
        (address BLACKBOARD "localhost" )
        (port_out BLACKBOARD  2300)

	;; Network definitions
	(open-network BLACKBOARD)
)

(defrule clips-alive
	?f <- (alive clips)
	=>
	(retract ?f)
	(printout t "clips alive")
)


(defrule obs-dest
        ?f <- (step ?num obs ?obs dest ?dest)
        =>
        (retract ?f)
	(assert (received ?num ?obs ?dest))
	(bind ?num (* ?num 1))
        ;(printout t "Modified number " ?num)
)



(defrule no-obstacle-light-up-left
	?f <- (received ?num 0 1)
	=>
	(retract ?f)
	(assert (movement ?num left_45 forward 0.5))
)


(defrule no-obstacle-light-down-left
        ?f <- (received ?num 0 2)
        =>
        (retract ?f)
        (assert (movement ?num left_135 forward 0.5))
)

(defrule no-obstacle-light-down-right
        ?f <- (received ?num 0 3)
        =>
        (retract ?f)
        (assert (movement ?num right_135 forward 0.5))
)

(defrule no-obstacle-light-up-right
	?f <- (received ?num 0 4)
	=>
	(retract ?f)
	(assert (movement ?num right_45 forward 0.5))
)

(defrule no-obstacle-light-front
	?f <- (received ?num 0 0)
	=>
	(retract ?f)
	(assert (movement ?num none forward 0.5))
)


(defrule arbiter
	?f <- (movement ?num ?rotation ?advance ?status)
	=>
	(retract ?f)
	(printout t "movement " ?num " " ?rotation " " ?advance " " ?status)
)



