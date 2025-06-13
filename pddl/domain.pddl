(define (domain robotaxi)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; TYPES ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
  robot
  room
  passenger
)

;; PREDICATES ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates
  (robot_at ?r - robot ?ro - room)
  (passenger_at ?p - passenger ?ro - room)
  (in_taxi ?p - passenger ?r - robot)
  (charging_point_at ?ro - room)
  (connected ?ro1 ?ro2 - room)
)

;; FUNCTIONS ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions
  (battery_level ?r - robot) ;; valori numerici
)

;; ACTIONS ;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; Movimento tra stanze
(:durative-action move
  :parameters (?r - robot ?from ?to - room)
  :duration (= ?duration 5)
  :condition (and
    (at start (robot_at ?r ?from))
    (at start (connected ?from ?to))
    (over all (> (battery_level ?r) 10))
  )
  :effect (and
    (at start (not (robot_at ?r ?from)))
    (at end (robot_at ?r ?to))
    (at end (decrease (battery_level ?r) 10))
  )
)

;; Ricarica
(:durative-action charge
  :parameters (?r - robot ?loc - room)
  :duration (= ?duration 10)
  :condition (and
    (at start (robot_at ?r ?loc))
    (at start (charging_point_at ?loc))
  )
  :effect (at end (increase (battery_level ?r) 50))
)

;; Caricare un passeggero
(:durative-action pickup
  :parameters (?r - robot ?p - passenger ?loc - room)
  :duration (= ?duration 2)
  :condition (and
    (at start (robot_at ?r ?loc))
    (at start (passenger_at ?p ?loc))
  )
  :effect (and
    (at end (not (passenger_at ?p ?loc)))
    (at end (in_taxi ?p ?r))
  )
)

;; Lasciare un passeggero
(:durative-action dropoff
  :parameters (?r - robot ?p - passenger ?loc - room)
  :duration (= ?duration 2)
  :condition (and
    (at start (robot_at ?r ?loc))
    (at start (in_taxi ?p ?r))
  )
  :effect (and
    (at end (not (in_taxi ?p ?r)))
    (at end (passenger_at ?p ?loc))
  )
)

)
