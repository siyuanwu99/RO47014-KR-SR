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
        :parameters (?v - robot ?wp - waypoint ?g1 - gripper ?obj1 - object ?g2 - gripper ?obj2 - object)
        :duration (= ?duration 5)
        :condition (and
            (at start (robot-at ?v ?wp))
            (at start (object-at ?obj1 ?wp))
            (at start (object-at ?obj2 ?wp))
            (at start (can_pick ?g1 ?obj1))
            (at start (can_pick ?g2 ?obj2))
            (at start (free ?g1))
            (at start (free ?g2))
            (at start (not-working ?v))
        )
        :effect (and
            (at end (robot-at ?v ?wp))
            (at end (is_holding ?g1 ?obj1))
            (at end (is_holding ?g2 ?obj2))
            (at end (not (free ?g1)))
            (at end (not (free ?g2)))
            (at end (not (object-at ?obj1 ?wp)))
            (at end (not (object-at ?obj2 ?wp)))
            (at start (not (not-working ?v)))
            (at end (not-working ?v))
        )
    )
    
    (:durative-action place
        :parameters (?v - robot ?wp - waypoint ?g1 - gripper ?obj1 - object ?g2 - gripper ?obj2 - object)
        :duration (= ?duration 2)
        :condition (and
            (at start (robot-at ?v ?wp))
            (at start (is_holding ?g1 ?obj1))
            (at start (is_holding ?g2 ?obj2))
            (at start (not-working ?v))
            (at start (is_full ?obj1))
            (at start (is_full ?obj2))
            (at start (can_place ?obj1 ?wp))
            (at start (can_place ?obj2 ?wp))
        )
        :effect (and
            (at end (robot-at ?v ?wp))
            (at end (free ?g1))
            (at end (free ?g2))
            (at end (not (is_holding ?g1 ?obj1)))
            (at end (not (is_holding ?g2 ?obj2)))
            (at end (object-at ?obj1 ?wp))
            (at end (object-at ?obj2 ?wp))
            (at start (not (not-working ?v)))
            (at end (not-working ?v))
            (at end (is_classified ?obj1))
            (at end (is_classified ?obj2))
        )
    )
    
    (:durative-action discard
        :parameters (?v - robot ?wp - waypoint ?g1 - gripper ?obj1 - object ?g2 - gripper ?obj2 - object)
        :duration (= ?duration 2)
        :condition (and
            (at start (robot-at ?v ?wp))
            (at start (is_holding ?g1 ?obj1))
            (at start (is_holding ?g2 ?obj2))
            (at start (not-working ?v))
            (at start (is_empty ?obj1))
            (at start (is_empty ?obj2))
            (at start (can_discard ?wp))
        )
        :effect (and
            (at end (robot-at ?v ?wp))
            (at end (free ?g1))
            (at end (free ?g2))
            (at end (not (is_holding ?g1 ?obj1)))
            (at end (not (is_holding ?g2 ?obj2)))
            (at end (object-at ?obj1 ?wp))
            (at end (object-at ?obj2 ?wp))
            (at start (not (not-working ?v)))
            (at end (not-working ?v))
            (at end (is_classified ?obj1))
            (at end (is_classified ?obj2))
        )
    )
    
)