
;************************************************
;*						*
;*	oracle.clp 				*
;*						*
;*	Jesus Savage				*
;*						*
;*		Bio-Robotics Laboratory		*
;*		UNAM, 2019			*
;*						*
;*						*
;************************************************


(defrule clips-alive
	?f <- (alive clips)
	=>
	(retract ?f)
	(printout t "ROS clips alive ROS")
	(printout t "clips very alive" crlf)
)


(defrule Max-Values
	(max-advance ?max-advance max-rotation ?max-rotation)
	=>
	(printout t "ROS received max advance rotation ROS")
	(printout t "received from max advance rotation" crlf)
)


(defrule obs-dest
        ?f <- (step ?num intensity ?int obs ?obs dest ?dest)
        =>
        (retract ?f)
	(assert (received ?num ?obs ?dest))
	(assert (intensity ?num ?int))
	(bind ?num (* ?num 1))
        (printout t "Modified number " ?num crlf)
)


(defrule no-obstacle-light-backward-right
        ?f <- (received ?num 0 0)
	(max-advance ?max-advance max-rotation ?max-rotation)
        =>
        (retract ?f)
	; rotate to the right 135 degrees and advance forward
	(bind ?right_135 (- 0 (* 3 ?max-rotation)))
	(bind ?forward ?max-advance)
        (assert (movement ?num ?right_135 ?forward 0.5))
)


(defrule no-obstacle-light-backward-left
        ?f <- (received ?num 0 1)
	(max-advance ?max-advance max-rotation ?max-rotation)
        =>
        (retract ?f)
	; rotate to the left 135 degrees and advance forward
	(bind ?left_135 (* 2 ?max-rotation))
	(bind ?forward ?max-advance)
	(assert (movement ?num ?left_135 ?forward 0.5))
)


(defrule no-obstacle-light-front-right
	?f <- (received ?num 0 2)
	(max-advance ?max-advance max-rotation ?max-rotation)
	=>
	(retract ?f)
	; rotate to the right 45 degrees and advance forward
        (bind ?right_45 (- 0 ?max-rotation))
        (bind ?forward ?max-advance)
	(assert (movement ?num ?right_45 ?forward 0.5))
)


(defrule no-obstacle-light-front-left
	?f <- (received ?num 0 3)
	(max-advance ?max-advance max-rotation ?max-rotation)
	=>
	(retract ?f)
	; rotate to the left 45 degrees and advance forward
	(bind ?left_45 ?max-rotation)
	(bind ?forward ?max-advance)
	(assert (movement ?num ?left_45 ?forward 0.5))
)


(defrule obstacle-right-light-backward-right
        ?f <- (received ?num 1 0)
	(max-advance ?max-advance max-rotation ?max-rotation)
        =>
        (retract ?f)
	(bind ?left_90 (* 2 ?max-rotation))
        (bind ?forward ?max-advance)
	; rotate to the 90 degrees and goes forward
        (assert (movement ?num ?left_90 ?forward 0.5))
)


(defrule obstacle-right-light-back-left
        ?f <- (received ?num 1 1)
        (max-advance ?max-advance max-rotation ?max-rotation)
        =>
        (retract ?f)
	(bind ?left_135 (* 3 ?max-rotation))
        (bind ?forward ?max-advance)
        ; rotate to the left 135 degrees and advance forward
        (assert (movement ?num ?left_135 ?forward 0.5))
)

(defrule obstacle-right-light-front-right
        ?f <- (received ?num 1 2)
	(max-advance ?max-advance max-rotation ?max-rotation)
        =>
        (retract ?f)
        (bind ?left_45 (- 0 ?max-rotation))
        (bind ?backward (- 0 ?max-advance))
        ; rotate to the left 45 degrees and advance forward
        (assert (movement ?num ?left_45 ?backward 0.5))
)


(defrule obstacle-right-light-front-left
        ?f <- (received ?num 1 3)
	(max-advance ?max-advance max-rotation ?max-rotation)
        =>
        (retract ?f)
	(bind ?left_45 ?max-rotation)
        (bind ?forward ?max-advance)
	; rotate to the left 45 degrees and advance forward
        (assert (movement ?num ?left_45 ?forward 0.5))
)


(defrule obstacle-left-light-backward-right
        ?f <- (received ?num 2 0)
        (max-advance ?max-advance max-rotation ?max-rotation)
        =>
        (retract ?f)
        (bind ?right_135 (- 0 (* 3 ?max-rotation)))
        (bind ?forward ?max-advance)
        ; rotate to the right 135 degrees and goes forward
        (assert (movement ?num ?right_135 ?forward 0.5))
)


(defrule obstacle-left-light-back-left
        ?f <- (received ?num 2 1)
        (max-advance ?max-advance max-rotation ?max-rotation)
        =>
        (retract ?f)
        (bind ?left_135 (* 3 ?max-rotation))
        (bind ?forward ?max-advance)
        ; rotate to the left 135 degrees and advance forward
        (assert (movement ?num ?left_135 ?forward 0.5))
)


(defrule obstacle-left-light-front-right
        ?f <- (received ?num 2 2)
        (max-advance ?max-advance max-rotation ?max-rotation)
        =>
        (retract ?f)
        (bind ?zero 0.0)
        (bind ?forward (- 0 ?max-advance))
        (assert (movement ?num ?zero ?forward 0.5))
)


(defrule obstacle-left-light-front-left
        ?f <- (received ?num 2 3)
        (max-advance ?max-advance max-rotation ?max-rotation)
        =>
        (retract ?f)
        (bind ?right_45 ?max-rotation)
        (bind ?forward ?max-advance)
        ; rotate to the right 45 degrees and advance forward
        (assert (movement ?num ?right_45 ?forward 0.5))
)


(defrule obstacle-front-light-backward-left
        ?f <- (received ?num 3 0)
        (max-advance ?max-advance max-rotation ?max-rotation)
        =>
        (retract ?f)
        (bind ?left_135 (* 3 ?max-rotation))
        (bind ?forward ?max-advance)
        ; rotate to the left 135 degrees and advance forward
        (assert (movement ?num ?left_135 ?forward 0.5))
)


(defrule obstacle-front-light-backward-right
        ?f <- (received ?num 3 1)
        (max-advance ?max-advance max-rotation ?max-rotation)
        =>
        (retract ?f)
        (bind ?right_135 (- 0 (* 3 ?max-rotation)))
        (bind ?forward ?max-advance)
        ; rotate to the right 135 degrees and goes forward
        (assert (movement ?num ?right_135 ?forward 0.5))
)


(defrule obstacle-front-light-front-right
        ?f <- (received ?num 3 2)
        (max-advance ?max-advance max-rotation ?max-rotation)
        =>
        (retract ?f)
        (bind ?right_90 (- 0 (* 3 ?max-rotation)))
        (bind ?forward ?max-advance)
        ; rotate to the right 90 degrees and advance forward
        (assert (movement ?num ?right_90 ?forward 0.5))
)


(defrule obstacle-front-light-front-left
        ?f <- (received ?num 3 3)
        (max-advance ?max-advance max-rotation ?max-rotation)
        =>
        (retract ?f)
        (bind ?left_90 (* 1.5 ?max-rotation))
        (bind ?forward ?max-advance)
        ; rotate to the left 90 degrees and advance forward
        (assert (movement ?num ?left_90 ?forward 0.5))
)


(defrule arbiter
	?f <- (movement ?num ?rotation ?advance ?status)
	?f1 <- (intensity ?num ?intensity)
	=>
	(retract ?f ?f1)
	(if (> ?intensity 30.0) then
		;(printout t "ROS movement " ?num " " ?rotation " " ?advance " " 1.0 " ROS")
		(printout t "ROS goto " ?num " " ?rotation " " ?advance " " 1.0 " ROS")
		(printout t "movement " ?num " " ?rotation " " ?advance " " 1.0 " " crlf)
	else
		;(printout t "ROS movement " ?num " " ?rotation " " ?advance " " ?status " ROS")
		(printout t "ROS goto " ?num " " ?rotation " " ?advance " " ?status " ROS")
		(printout t "movement " ?num " " ?rotation " " ?advance " " ?status " " crlf)
	)
)


(defrule delete-unused-intensities
	(declare (salience -1000))
        ?f <- (intensity ?num $?)
        =>
        (retract ?f)
)

(defrule delete-unused-received
	(declare (salience -1000))
        ?f <- (received ?num $?)
        =>
        (assert (movement ?num 0.0 0.0 0.5))
        (retract ?f)
)


