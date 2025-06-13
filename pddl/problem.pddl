(define (problem robotaxi_problem)
  (:domain robotaxi)

  (:objects
    taxi1 - robot
    room1 room2 room3 - room
    alice - passenger
  )

  (:init
    ;; Posizioni iniziali
    (robot_at taxi1 room1)
    (passenger_at alice room2)

    ;; Connessioni tra stanze
    (connected room1 room2)
    (connected room2 room3)
    (connected room3 room2)
    (connected room2 room1)

    ;; Punto di ricarica
    (charging_point_at room3)

    ;; Livello batteria iniziale
    (= (battery_level taxi1) 50)
  )

  (:goal
    (and
      (passenger_at alice room3)
    )
  )
)
