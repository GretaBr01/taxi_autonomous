(define (problem multi_passenger_transport)
  (:domain urban_taxi_extended_temporal)

  (:objects
    taxi1 taxi2 - vehicle
    alice bob - passenger
    a b c d e f - location
  )

  (:init
    (at taxi1 a)
    (at taxi2 b)
    (at_passenger alice c)
    (at_passenger bob d)

    (connected a b)
    (connected b c)
    (connected c d)
    (connected d e)
    (connected e f)
    (connected b e)
    (connected b a)
    (connected c b)
    (connected d c)
    (connected e d)
    (connected f e)
    (connected e b)

    (charging_station f)
    (charging_station a)

    ;; === traffico ===
    (traffic_heavy b c)
    (traffic_heavy d e)
  )

  (:goal
    (and
      (at_passenger alice f)
      (at_passenger bob a)
    )
  )
)


