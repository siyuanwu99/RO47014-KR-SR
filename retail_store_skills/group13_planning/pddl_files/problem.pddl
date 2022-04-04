(define (problem waypoint_following)
    (:domain waypoint_following)
    (:requirements :strips :typing)

    (:objects   tiago - robot
                wp0 wp_table_1 wp_table_2 wp_shelf_1 wp_shelf_2 wp_basket - waypoint
                leftgrip rightgrip - gripper
                AH_hagelslag_aruco_16 AH_hagelslag_aruco_17 AH_hagelslag_aruco_0 AH_hagelslag_aruco_1 - object
    )
    (:init
        ; GROUND TRUTH
        (is_full AH_hagelslag_aruco_16)
        (is_full AH_hagelslag_aruco_17)
        (is_empty AH_hagelslag_aruco_1)
        (is_empty AH_hagelslag_aruco_0)
        (can_place AH_hagelslag_aruco_16 wp_shelf_1)
        (can_place AH_hagelslag_aruco_17 wp_shelf_1)
        ; (can_place wp_table_2)
        (can_discard wp_basket)
        (can_pick leftgrip AH_hagelslag_aruco_17)
        (can_pick leftgrip AH_hagelslag_aruco_1)
        (can_pick rightgrip AH_hagelslag_aruco_16)
        (can_pick rightgrip AH_hagelslag_aruco_0)
        
        ; INIT STATES
        (visited wp0)
        (robot-at tiago wp0) 
        (not-working tiago)
        (free leftgrip)
        (free rightgrip)
        (object-at AH_hagelslag_aruco_16 wp_table_1)        
        (object-at AH_hagelslag_aruco_17 wp_table_1)        
        (object-at AH_hagelslag_aruco_0 wp_table_1)
        (object-at AH_hagelslag_aruco_1 wp_table_1)
    )
    
    (:goal (and
        ;(robot-at tiago wp_basket)
        (is_classified AH_hagelslag_aruco_0)
        (is_classified AH_hagelslag_aruco_1)
        (is_classified AH_hagelslag_aruco_17)
        (is_classified AH_hagelslag_aruco_16)
	)
	)

)
