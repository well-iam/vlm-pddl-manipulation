(define (domain objectsworld)
  (:requirements :strips :typing :equality) ;;

  ;; Tipi: manipulable (cubi) e static (tavolo, pad, zone)
  (:types manipulable static - object)

  (:constants
      tavolo - static
  )

  ;; Predicates:
  ;; (on ?x ?y) -> x is on y (valid for both Cube on Pad and Cube on Cube)
  ;; (clear ?x) -> x has nothing on top
  ;; (handempty) -> the hand is free
  ;; (holding ?x) -> I am holding x
  (:predicates (on ?x - manipulable ?y - object)
               (clear ?x - object)
               (handempty)
               (holding ?x - manipulable))

  ;; Action: Pick an object from a support (whether it is a table or other)
  (:action pick
   :parameters (?obj - manipulable ?support - object)
   :precondition (and (on ?obj ?support)     ;; The object is on the support
                      (clear ?obj)           ;; Nothing on top of the object
                      (handempty))           ;; Hand free
   :effect (and (not (on ?obj ?support))     ;; It is no longer on the support
                (not (clear ?obj))           ; It is no longer "clear" (it is in hand)
                (not (handempty))            ;; Hand occupied
                (holding ?obj)               ;; Holding the object
                (clear ?support)))           ;; The support is now free!

  ;; Action: Place an object on a support
  (:action place
   :parameters (?obj - manipulable ?support - object)
   :precondition (and (holding ?obj)         ;; I am holding the object
                      (clear ?support)       ;; The support is free
                      (not (= ?obj ?support))) ;; I cannot place it on itself
   :effect (and (not (holding ?obj))
                (not (clear ?support))       ;; The support is no longer free
                (clear ?obj)                 ;; The placed object is free on top
                (handempty)
                (on ?obj ?support)))
)