(define (domain stage_lights)
    (:requirements :typing)
    
    ;; one type of object in this domain, only the light
    (:types light)

    (:predicates
        ;; these are the properties of a light
        (off ?l - light)
        (on ?l - light)
        (left ?l1 ?l2 - light)
        (right ?l1 ?l2 - light)
    )

    ;; this action correspond to "light ON"
    (:action light_on
        :parameters (?l ?ll ?lr - light)
        :precondition (and
            (off ?l)
            (left ?ll ?l)
            (right ?lr ?l)
            (or
                (and 
                    (on ?ll)
                    (off ?lr)
                )
                (and 
                    (on ?lr)
                    (off ?ll)
                )
            )
        )
        :effect (and
            (not (off ?l))
            (on ?l)
        )
    )
    ;; this action correspond to "light OFF"
    (:action light_off
        :parameters (?l - light)
        :precondition (and
            (on ?l)
        )
        :effect (and
            (not (on ?l))
            (off ?l)
        )
    )
)
