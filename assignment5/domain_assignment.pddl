(define (domain stage_lights)
    (:requirements :typing)
    
    ;; one type of object in this domain, only the light
    (:types light)

    (:predicates
        ;; these are the properties of a light
        (off ?l - light)
        (on ?l - light)
        (left_on ?l - light)
        (left_off ?l -light)
        (right_on ?l - light)
        (right_off ?l -light)
    )

    ;; this action correspond to "light ON"
    (:action light_on
        :parameters (?l - light)
        :precondition (and
            (off ?l)
            (or
                (and 
                    (left_on ?l)
                    (right_off ?l)
                )
                (and 
                    (left_off ?l)
                    (right_on ?l)
                )
            )
        )
        :effect (and
            (not (off ?l))
            (on ?l)
        )
    )
)