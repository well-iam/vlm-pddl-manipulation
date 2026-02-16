(define (domain objectsworld)
  (:requirements :strips :typing :equality :conditional-effects)

  ;; Types: manipulable (e.g., cubes) and static (table, pad, zones)
  (:types 
      manipulable static - object)

  (:constants
      table - static
  )

  ;; Predicates:
  ;; (on ?x ?y) -> x is on top of y (valid for Cube on Pad or Cube on Cube)
  ;; (clear ?x) -> x has nothing on top of it
  ;; (handempty) -> the hand is free
  ;; (holding ?x) -> currently holding x
  (:predicates 
    (on ?x - object ?y - object)
    (clear ?x - object)
    (handempty)
    (holding ?x - manipulable)
    (is_container ?x - object)
  )

  ;; Action: Pick an object from a support (be it the table or another object)
  (:action pick
   :parameters (?obj - manipulable ?support - object)
   :precondition (and (on ?obj ?support)      ;; The object is on the support
                      (clear ?obj)            ;; Nothing on top of the object
                      (handempty))            ;; Hand is free
   :effect (and (not (on ?obj ?support))      ;; No longer on the support
                (not (clear ?obj))            ;; No longer "clear" (it is in hand)
                (not (handempty))             ;; Hand is occupied
                (holding ?obj)                ;; Holding the object
                ;; If we remove something from a container, it was already clear and remains clear.
                ;; If we remove something from a normal block, it becomes clear.
                ;; In practice: (clear ?y) always becomes true.
                (clear ?support)))            ;; The support is now clear!
                

  ;; Action: Place an object on a support
  (:action place
   :parameters (?obj - manipulable ?support - object)
   :precondition (and (holding ?obj)          ;; Currently holding the object
                      (clear ?support)        ;; The support is clear
                      (not (= ?obj ?support))) ;; Cannot place it on itself
   :effect (and (not (holding ?obj))
                (clear ?obj)                  ;; The placed object is clear on top
                (handempty)
                (on ?obj ?support)
                ;; If ?y is NOT a container (and not the table), then it is no longer clear.
                ;; If ?y IS a container (or the table), this line is ignored and remains clear.
                (when (and (not (is_container ?support)) (not (= ?support table)))
                      (not (clear ?support))
                )
    )
  )
)