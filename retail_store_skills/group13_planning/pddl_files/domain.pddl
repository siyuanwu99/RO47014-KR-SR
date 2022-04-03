(define (domain waypoint_following)

    (:requirements
        :typing
        :durative-actions
        )

    (:types
        waypoint
        robot
        object
        gripper
    )
    
    (:predicates
        (visited ?wp - waypoint)
        (can_place ?obj - object ?wp - waypoint)
        (can_discard ?wp - waypoint)
        (robot-at ?v - robot ?wp - waypoint)
        (object-at ?obj - object ?wp - waypoint)
        
        (can_pick ?g - gripper ?obj - object)
        (is_holding ?g - gripper ?obj - object)
        (free ?g - gripper)
        
        (not-working ?v - robot)
        
        (is_full ?obj - object)
        (is_empty ?obj - object)
        
        (is_classified ?obj - object)
    )
    
    (:durative-action move
        :parameters (?v - robot ?from ?to - waypoint)
        :duration (= ?duration 2)
        :condition (and
            (at start (robot-at ?v ?from))
            (at start (not-working ?v))
        )
        :effect (and
            (at end (visited ?to))
            (at end (robot-at ?v ?to))
            (at start (not
                (robot-at ?v ?from)
            ))
        )
    )

    (:durative-action pick
        :parameters (?v - robot ?wp - waypoint ?obj - object ?g - gripper)
        :duration (= ?duration 2)
        :condition (and
            (at start (robot-at ?v ?wp))
            (at start (object-at ?obj ?wp))
            (at start (can_pick ?g ?obj))
            (at start (free ?g))
            (at start (not-working ?v))
        )
        :effect (and
            (at end (robot-at ?v ?wp))
            (at end (is_holding ?g ?obj))
            (at start (not 
                (free ?g)
                )
            )
            (at end (not
                (object-at ?obj ?wp)
                )
            )
            (at start (not (not-working ?v)))
            (at end (not-working ?v))
        )
    )
    
    (:durative-action place
        :parameters (?v - robot ?wp - waypoint ?obj -object ?g - gripper)
        :duration (= ?duration 2)
        :condition (and
            (at start (robot-at ?v ?wp))
            (at start (is_holding ?g ?obj))
            (at start (not-working ?v))
            (at start (is_full ?obj))
            (at start (can_place ?obj ?wp))
        )
        :effect (and
            (at end (robot-at ?v ?wp))
            (at end (free ?g))
            (at end (not (is_holding ?g ?obj)))
            (at end (object-at ?obj ?wp))
            (at start (not (not-working ?v)))
            (at end (not-working ?v))
            (at end (is_classified ?obj))
        )
    )
    
    (:durative-action discard
        :parameters (?v - robot ?wp - waypoint ?obj -object ?g - gripper)
        :duration (= ?duration 2)
        :condition (and
            (at start (robot-at ?v ?wp))
            (at start (is_holding ?g ?obj))
            (at start (not-working ?v))
            (at start (is_empty ?obj))
            (at start (can_discard ?wp))
        )
        :effect (and
            (at end (robot-at ?v ?wp))
            (at end (free ?g))
            (at end (not (is_holding ?g ?obj)))
            (at end (object-at ?obj ?wp))
            (at start (not (not-working ?v)))
            (at end (not-working ?v))
            (at end (is_classified ?obj))
        )
    )
    
)