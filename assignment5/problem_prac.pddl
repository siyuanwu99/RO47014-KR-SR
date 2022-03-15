(define (problem stage_lights)
(:domain stage_lights)
    ;; This is a sample problem with 5 lights
    (:objects
        light_1 light_2 light_3 light_4 light_5 - light
    )
    ;; Initial state of the lights
    (:init
        (left_off light_1)
        (off light_1)
        (right_on light_1)
        (left_off light_2)
        (on light_2)
        (right_off light_2)
        (left_on light_3)
        (off light_3)
        (right_off light_3)
        (left_off light_4)
        (off light_4)
        (right_on light_4)
        (left_off light_5)
        (on light_5)
        (right_off light_5)
    )
    
    (:goal (and
        ;; Turn on all 5 lights
        (on light_1)
        (on light_2)
        (on light_3)
        (on light_4)
        (on light_5)
    ))
)