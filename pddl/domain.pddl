(define (domain urban_taxi_extended_temporal)
  (:requirements :strips :typing :durative-actions)

  (:types
    vehicle passenger location
  )

  (:predicates
    (at ?v - vehicle ?l - location)
    (at_passenger ?p - passenger ?l - location)
    (in ?p - passenger ?v - vehicle)
    (connected ?from - location ?to - location)
    (charging_station ?l - location)
    (energy_low ?v - vehicle)
    (busy ?v - vehicle)
    (traffic_heavy ?from - location ?to - location)
    (occupied ?v - vehicle) ;; nuovo predicato per occupazione taxi
  )

  (:functions
    (capacity ?v - vehicle)
  )

  (:durative-action drive_normal
    :parameters (?v - vehicle ?from - location ?to - location)
    :duration (= ?duration 5)
    :condition (and
      (at start (at ?v ?from))
      (at start (connected ?from ?to))
      (at start (not (busy ?v)))
      (at start (not (charging_station ?to)))
      (at start (not (traffic_heavy ?from ?to)))
    )
    :effect (and
      (at start (busy ?v))
      (at end (not (busy ?v)))
      (at end (not (at ?v ?from)))
      (at end (at ?v ?to))
      (at end (energy_low ?v))
    )
  )

  (:durative-action drive_normal_traffic
    :parameters (?v - vehicle ?from - location ?to - location)
    :duration (= ?duration 10)
    :condition (and
      (at start (at ?v ?from))
      (at start (connected ?from ?to))
      (at start (not (busy ?v)))
      (at start (not (charging_station ?to)))
      (at start (traffic_heavy ?from ?to))
    )
    :effect (and
      (at start (busy ?v))
      (at end (not (busy ?v)))
      (at end (not (at ?v ?from)))
      (at end (at ?v ?to))
      (at end (energy_low ?v))
    )
  )

  (:durative-action drive_to_charge
    :parameters (?v - vehicle ?from - location ?to - location)
    :duration (= ?duration 5)
    :condition (and
      (at start (at ?v ?from))
      (at start (connected ?from ?to))
      (at start (not (busy ?v)))
      (at start (charging_station ?to))
      (at start (not (traffic_heavy ?from ?to)))
    )
    :effect (and
      (at start (busy ?v))
      (at end (not (busy ?v)))
      (at end (not (at ?v ?from)))
      (at end (at ?v ?to))
    )
  )

  (:durative-action drive_to_charge_traffic
    :parameters (?v - vehicle ?from - location ?to - location)
    :duration (= ?duration 10)
    :condition (and
      (at start (at ?v ?from))
      (at start (connected ?from ?to))
      (at start (not (busy ?v)))
      (at start (charging_station ?to))
      (at start (traffic_heavy ?from ?to))
    )
    :effect (and
      (at start (busy ?v))
      (at end (not (busy ?v)))
      (at end (not (at ?v ?from)))
      (at end (at ?v ?to))
    )
  )

  (:durative-action pickup
    :parameters (?v - vehicle ?p - passenger ?l - location)
    :duration (= ?duration 2)
    :condition (and
      (at start (at ?v ?l))
      (at start (at_passenger ?p ?l))
      (at start (not (in ?p ?v)))
      (at start (not (busy ?v)))
      (at start (not (occupied ?v))) ;; qui controllo che il taxi non sia occupato
    )
    :effect (and
      (at start (busy ?v))
      (at end (not (busy ?v)))
      (at end (in ?p ?v))
      (at end (not (at_passenger ?p ?l)))
      (at end (occupied ?v)) ;; taxi ora occupato
    )
  )

  (:durative-action dropoff
    :parameters (?v - vehicle ?p - passenger ?l - location)
    :duration (= ?duration 2)
    :condition (and
      (at start (at ?v ?l))
      (at start (in ?p ?v))
      (at start (not (busy ?v)))
    )
    :effect (and
      (at start (busy ?v))
      (at end (not (busy ?v)))
      (at end (not (in ?p ?v)))
      (at end (at_passenger ?p ?l))
      (at end (not (occupied ?v))) ;; taxi ora libero
    )
  )

  (:durative-action charge
    :parameters (?v - vehicle ?l - location)
    :duration (= ?duration 3)
    :condition (and
      (at start (at ?v ?l))
      (at start (charging_station ?l))
      (at start (energy_low ?v))
      (at start (not (busy ?v)))
    )
    :effect (and
      (at start (busy ?v))
      (at end (not (busy ?v)))
      (at end (not (energy_low ?v)))
    )
  )
)

