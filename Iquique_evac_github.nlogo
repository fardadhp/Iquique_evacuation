;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Developed by: Fardad Haghpanah
;;;
;;; Vehicle Navigation Module + Pedestrian Navigation Module + walk-watch-cars for human-car interactions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

extensions [gis]

breed [people person]
breed [trappeds trapped]
;breed [rescuees rescuee]
breed [drivers driver]
breed [idrivers idriver]
breed [cars car]
people-own [
  follow-wall?
  target
  direction        ;; 1: follow right wall; -1: follow left wall (when reaching a barrier)
  wspeed           ;; walking speed
  H                ;; patch at which agent reaches an obstacle
  r                ;; radius of vision
  L                ;; patch along the boundary of the obstacle with minimum distance to target (Bug1)
  Q                ;; distance from current patch to target when agent is walking around an obstacle (Bug1)
  R2               ;; integrated length of the boundary starting at H (Bug1)
  R3               ;; integrated length of the boundary starting at L (Bug1)
  search-mode?     ;; boolean, to indicate whether agent has found the L-point or not yet (Bug1)
  m-line           ;; list containing patches on the straight line from start point to target (Bug2 & DistBug  & New Alg)
  step             ;; minimum(vision radius, minimal distance between obstacles) [ref. to the source paper] (DistBug & New Alg)
  d-min            ;; minimum distance to target when following obstacle boundary (DistBug & New Alg)
                   ;; minimum distance to target observed along the visible portion of the blocking obstacle (TangentBug)
  extension2?      ;; boolean; indicates if follow direction has been changed once according to the second proposed extension to the algorithm (DistBug)
  cn               ;; counter to allow agent pass through H point when direction is changed by extension-2 of DistBug (DistBug)
  extension3?      ;; boolean; indicate whether a virtual obstacle is already met when following an obstacle boundary (DistBug)
  vir-obs          ;; list containing patches of the virtual obstacles created according to the third proposed extension (DistBug)
  ltg-list         ;; list containing LTG points (TangentBug)
  transition1?     ;; boolean; indicate if agent is transiting from motion-toward-target to follow-boundary algorithm (TangentBug)
  transition2?     ;; boolean; indicate if agent is transiting from follow-boundary to motion-toward-target algorithm (TangentBug)
  node             ;; selected waypoint when an obstacle is found (KBug)
  edge             ;; edges of the identified onstacle (KBug)
  last-node        ;; last selected waypoint (stored to avoid going back to the last waypoint) (KBug)
  corner           ;; last corner the agent walked on (old alg)
  int-target       ;; intermediate target: patch on the other side of the barrier (target patch to pass the barrier) (old alg)
  br-dist          ;; max distance from int-target to target (old alg)
  tolerance        ;; number of time steps the agent waits behind another in a congestion before changing its position
  tol-count        ;; time step counter for tolerance
  step_forward?    ;; true: agent can step forward; false: otherwise (used for agent interaction) (KBug & TangentBug)
  car?             ;; boolean, true if person owns a car; otherwise 0.
  pre
]

globals [
  TL_period-scaled            ;; Traffic light period scaled to the model
  green_direction             ;; 0 or 90; indicating the direction in intersections for which traffic lights are green
  inf                         ;; a large number
  safety_dist                 ;; standard queueing distance [m/scale]
  buildings                   ;; patches corresponding to building blocks
  buildings_info              ;; List containing features of building blocks
  roads                       ;; patches corresponding to roads
  roads_info                  ;; List containing features of roads
  intersections               ;; patches corresponding to intersections
  target-set                  ;; set of gathering points
  target-line                 ;; border of safe zone
  exit-doors                  ;; patch-set of all main-gates of building blocks
  gen-pop                     ;; total generated population
  class                       ;; category of the exit doors corresponding to:
                                 ;; 1: low-rise buildings (1 - 2 stories)
                                 ;; 3: medium-rise buildings (3 - 7 stories)
                                 ;; 12: high-rise buildings (> 7 stories)
                                 ;; schools are considered as medium-rise.
  max-pop                     ;; maximum number of agents to occupy one patch = (scale/agent size)^2
  max-end-time                ;; time at which last person evacuate buildings
  max-speed-ped               ;; maximum speed of pedestrians (used to calculate timescale of the model = scale/max-speed-ped-ped)
  nodes_list                  ;; list containing nodes' IDs and corresponding patches [0 (patch x x) 1 (patch x x) 2 (patch x x) ...]
  nodes_roads                 ;; list containing nodes' information [id, patch, connected roads]
  nodes_connect               ;; list containing nodes' information [id, patch, connected nodes including connecting road's ID, and distance]
  roads_connect               ;; list containing roads' information [id, connected nodes' IDs]
  nodes_connect-updated       ;; nodes_connect, connectivity updated based on roads' directions; i.e. if a road is oneway, distance for conflicting direction is set to inf.
  shortest_distance_matrix    ;; outcome from Floyd–Warshall Algorithm: shortest distance for all pairs of nodes (not particularly in used)
  shortest_distance_nodes     ;; outcome from Floyd–Warshall Algorithm: nodes leading to shortest distance paths
  np                          ;; next patch (used for pedestrian navigation algorithms)
  headt                       ;; for pedestrian navigation algorithms (old alg)
  n                           ;; for pedestrian navigation algorithms (old alg)
  e1
  e2
  loc
  time
]

patches-own [
  id                          ;; ID of the corresponding building
  pop                         ;; total population of the building block
  pop_list                    ;; list containing number of people in each building type [low medium high]
  poprescom                   ;; population in residential and commercial buildings
  popsch                      ;; population in schools
  n_bld                       ;; # of buildings in the building block
  n_bld_list                  ;; list of number of buildings for each category [low medium high]
  low                         ;; # of low-rise buildings in the building block
  med                         ;; # of medium-rise buildings in the building block
  high                        ;; # of high-rise buildings in the building block
  sch                         ;; # of school buildings in the building block
  exits                       ;; exit patches of building blocks (count = n_bld)
  rate                        ;; flow rate of evacuees coming out of building
  rate-ct                     ;; counter for buildings with rate < 1
  main-gate                   ;; representative patch containing info of the corresponding building block
  pop-class                   ;; average population of each class of buildings in the building block [low medium high]
  ini                         ;; time at which first person evacuates the building for different classes [low medium high]
  endd                        ;; time at which last person evacuates the building for different classes [low medium high]
  lanes                       ;; number of lanes for road patches
  oneway                      ;; 1 if road is oneway; otherwise 0
  sidewalk                    ;; 1 if road has sidewalk; otherwise 0
  accessible                  ;; 1 if road is accessible for cars; otherwise 0
  ttype                       ;; type of the road: local or highway
  suggested                   ;; 1 if road is part of suggested routes for evacuation; otherwise 0
  direct                      ;; direction of the road: up, down, righ, left or bi for two-way roads
  speed_limit                 ;; speed limit for vehicles
  traffic_light               ;; 1 if intersection has a traffic light; 0 otherwise
  walkable                    ;; 1 if pedestrians can walk onto the patch; 0 otherwise
]

cars-own [
  ;acceleration                ;; acceleration of the car
  ;deceleration                ;; deceleration of the car
  speed                       ;; speed of the car adjusted with regard to surrounding cars and speed limit
  destination                 ;; final destination (emergency exit)
  target                      ;; intermediate target to reach final destinatotion
  riders                      ;; people riding the car
  car-size                    ;; size of the car
  ;at-intersection?            ;; boolean, true if car is right before an intersection (for traffic light maneuvers)
]

idrivers-own [car?]

;rescuees-own [loc time]

to load-gis-map
  clear-all
  gis:load-coordinate-system ("network5.prj")
  set buildings gis:load-dataset "test.shp"
  set roads gis:load-dataset "network5.shp"
  set intersections gis:load-dataset "nodes4.shp"
  set target-set gis:load-dataset "safe_points.shp"
  set target-line gis:load-dataset "safe_line.shp"

  let Gis-Map-Size gis:envelope-of buildings
  let w item 1 Gis-Map-Size - item 0 Gis-Map-Size
  let k item 3 Gis-Map-Size - item 2 Gis-Map-Size
  set w ((w * m_scale) / 2) ;ceiling
  set k ((k * m_scale) / 2) ;ceiling
  resize-world (0 - w) w (0 - k) k
  gis:set-world-envelope (Gis-Map-Size)
  set-patch-size 200 / w

  set buildings_info []
  let buildings_list gis:feature-list-of Buildings
  foreach buildings_list [
    set buildings_info lput (list (gis:property-value ? "ID") (gis:property-value ? "Population")
      (gis:property-value ? "Buildings") (gis:property-value ? "Low") (gis:property-value ? "Medium") (gis:property-value ? "High") (gis:property-value ? "Schools")) buildings_info  ;(gis:property-value ? "PopResComm") (gis:property-value ? "PopSchools")
    ask patches gis:intersecting ? [
      set id (gis:property-value ? "ID")
      set pop (gis:property-value ? "Population")
      set pop round (pop * 0.612)  ;; modify total population for different scenarios
      ;set poprescom (gis:property-value ? "PopResComm")
      ;set popsch (gis:property-value ? "PopSchools")
      set n_bld (gis:property-value ? "Buildings")
      set low (gis:property-value ? "Low")
      set med (gis:property-value ? "Medium")
      set high (gis:property-value ? "High")
      set sch (gis:property-value ? "Schools")
    ]
  ]

  set roads_info []
  let roads_list gis:feature-list-of roads
  foreach roads_list [
    set roads_info lput (list (gis:property-value ? "ID") (gis:property-value ? "LANES") (gis:property-value ? "ONEWAY")
       (gis:property-value ? "Type") (gis:property-value ? "direction")) roads_info  ;(gis:property-value ? "SIDEWALK") (gis:property-value ? "Accessible")  (gis:property-value ? "Suggested")
    ask patches gis:intersecting ? [
      set id (word "r" (gis:property-value ? "ID"))
      set lanes (gis:property-value ? "LANES")
      set oneway (gis:property-value ? "ONEWAY")
      ;set sidewalk (gis:property-value ? "SIDEWALK")
      ;set accessible (gis:property-value ? "Accessible")
      set ttype (gis:property-value ? "Type")
      ;set suggested (gis:property-value ? "Suggested")
      set direct (gis:property-value ? "direction")
    ]
  ]

  foreach gis:feature-list-of intersections [
    ask patches gis:intersecting ? [
    set traffic_light (gis:property-value ? "traf_light")
    ]
  ]

  set buildings patches gis:intersecting buildings
  set roads patches gis:intersecting roads
  set intersections patches gis:intersecting intersections
  set target-set patches gis:intersecting target-set
  set target-line patches gis:intersecting target-line
  let tset nobody ask target-set [let a one-of patches in-radius 2 with [member? self intersections] set tset (patch-set tset a)] set target-set tset
  ask patches [set pcolor black]
  ask buildings [set pcolor brown]
  ask roads [set pcolor red]
  ask intersections [set pcolor yellow]

end

to setup-cars
  if car? [
    let driver self
    ask min-one-of roads with [not member? self intersections and count cars-on self < [lanes] of self] [distance myself] [
      sprout-cars 1 [
        set color blue
        set shape "car"
        set size 15
        ;set pre patch-here
        set car-size one-of (list 4 5 6) / scale                     ;; typical size of cars = 4, 5, 6 meters
        ;set acceleration 3 * scale / (mu-speed + 2 * std-speed) ^ 2  ;; average acceleration of city cars =  3 m/ss
        ;set deceleration 5 * scale / (mu-speed + 2 * std-speed) ^ 2  ;; average deceleration of city cars = -5 m/ss
        set speed speed_limit
        set destination min-one-of target-set [distance myself]
        let elm item (read-from-string (remove "r" ([id] of patch-here))) roads_connect
        ;ifelse [direct] of patch-here != "bi" [
        ifelse [oneway] of patch-here = 1 [
         set heading [direct] of patch-here
         set target item (position (item 2 elm) nodes_list + 1) nodes_list
         face target
        ]
        [
          let n1 item (position (item 1 elm) nodes_list + 1) nodes_list
          let n2 item (position (item 2 elm) nodes_list + 1) nodes_list
          set target min-one-of (patch-set n1 n2) [distance myself]
          face target
        ]

        ask people with [car? = 0] [die]

        let x count people in-radius 10 with [not car?]
        ifelse x > 4 [set riders n-of 4 people in-radius 10 with [not car?]] [set riders n-of x people in-radius 10 with [not car?]]
        set riders (turtle-set riders driver)
        ask riders [set breed drivers set hidden? true]
      ]
    ]
  ]
end


to setup-icars
  let driver self
  ask min-one-of roads with [not member? self intersections and count cars-on self < [lanes] of self] [distance myself] [
    sprout-cars 1 [
      set color red
      set shape "car"
      set size 15
      ;set pre patch-here
      set car-size one-of (list 4 5 6) / scale                     ;; typical size of cars = 4, 5, 6 meters
      ;set acceleration 3 * scale / (mu-speed + 2 * std-speed) ^ 2  ;; average acceleration of city cars =  3 m/ss
      ;set deceleration 5 * scale / (mu-speed + 2 * std-speed) ^ 2  ;; average deceleration of city cars = -5 m/ss
      set speed speed_limit
      set destination min-one-of target-set [distance myself]
      let elm item (read-from-string (remove "r" ([id] of patch-here))) roads_connect
      ;ifelse [direct] of patch-here != "bi" [
      ifelse [oneway] of patch-here = 1 [
       set heading [direct] of patch-here
       set target item (position (item 2 elm) nodes_list + 1) nodes_list
       face target
      ]
      [
        let n1 item (position (item 1 elm) nodes_list + 1) nodes_list
        let n2 item (position (item 2 elm) nodes_list + 1) nodes_list
        set target min-one-of (patch-set n1 n2) [distance myself]
        face target
      ]

      ;ask people with [car? = 0] [die]
      ;let x count people in-radius 10 with [not car?]
      ;ifelse x > 4 [set riders n-of 4 people in-radius 10 with [not car?]] [set riders n-of x people in-radius 10 with [not car?]]
      set riders driver
      ask riders [set hidden? true]
    ]
  ]
end
to setup-road-directions
;  ask roads with [not member? self intersections] [
;    ifelse direct = "left" [
;      set direct towards (min-one-of neighbors with [member? self roads and pxcor < [pxcor] of myself] [distance myself])
;      if any? neighbors with [member? self intersections and pxcor < [pxcor] of myself] [set direct towards (one-of neighbors with [member? self intersections])]
;    ]
;    [
;      ifelse direct = "right" [
;        set direct towards (min-one-of neighbors with [member? self roads and pxcor > [pxcor] of myself] [distance myself])
;        if any? neighbors with [member? self intersections and pxcor > [pxcor] of myself] [set direct towards (one-of neighbors with [member? self intersections])]
;      ]
;      [
;        ifelse direct = "up" [
;          set direct towards (min-one-of neighbors with [member? self roads and pycor > [pycor] of myself] [distance myself])
;          if any? neighbors with [member? self intersections and pycor > [pycor] of myself] [set direct towards (one-of neighbors with [member? self intersections])]
;        ]
;        [
;          if direct = "down" [
;            set direct towards (min-one-of neighbors with [member? self roads and pycor <  [pycor] of myself] [distance myself])
;            if any? neighbors with [member? self intersections and pycor <  [pycor] of myself] [set direct towards (one-of neighbors with [member? self intersections])]
;          ]
;        ]
;      ]
;    ]
;  ]

  ask roads with [not member? self intersections] [
    if direct = "left" [set direct 270]
    if direct = "right" [set direct 90]
    if direct = "up" [set direct 0]
    if direct = "down" [set direct 180]
  ]
end


to setup-nodes
  set inf 1000000
  set safety_dist 1 / scale
  let nodes sort intersections
  set nodes_list []
  set nodes_roads []
  ask roads [set pcolor brown]
  ask intersections [set pcolor yellow]
  ask intersections [
    ask neighbors [
      set pcolor red
      if member? self roads [
        ;set pcolor red
        set traffic_light [traffic_light] of myself
      ]
    ]
  ]


  let i 0
  foreach nodes [
    set nodes_list lput i nodes_list
    set nodes_list lput ? nodes_list
    let lst1 []
    ask ? [
      let a roads in-radius 4
      set lst1 ([id] of a)
    ]
    set lst1 remove-duplicates lst1
    set lst1 fput ? lst1
    set lst1 fput i lst1
    set i i + 1
    set nodes_roads lput lst1 nodes_roads
  ]

  set nodes_connect []
  foreach nodes_roads [
    let j 0
    let temp2 ?
    let lst2 []
    set lst2 lput (item 0 temp2) lst2
    set lst2 lput (item 1 temp2) lst2
    while [j <= length nodes_roads - 1] [
      if j != position ? nodes_roads [
        foreach (item j nodes_roads) [
          if member? ? temp2 [
            let dist distance2v (item 1 (item j nodes_roads)) (item 1 temp2)
            set lst2 lput (list item 0 (item j nodes_roads) ? dist) lst2
          ]
        ]
      ]
      set j j + 1
    ]
    set nodes_connect lput lst2 nodes_connect
  ]

  set nodes_connect-updated nodes_connect
  foreach nodes_connect-updated [
    let temp ?
    let index position ? nodes_connect-updated
    let len length temp
    let index2 n-values (len - 2) [? + 2]
    ask (item 1 temp) [
      set i 2
      while [i < len] [
        let k item 0 (item i temp)
        let t item 1 (item k nodes_roads)
        let ang towards t
        if ang >= 315 [set ang 360 - ang]
        let ang2 [direct] of one-of roads with [id = item 1 (item i temp) and not member? self intersections]
        if ang2 != "bi" and abs (ang2 - ang) > 45 [
          let elm (item i temp)
          set elm (replace-item 2 elm inf)
          set temp (replace-item i temp elm)
        ]
        set i i + 1
      ]
      set nodes_connect-updated (replace-item index nodes_connect-updated temp)
    ]
  ]

  set i 0
  set roads_connect []
  while [i < length roads_info] [
    let elm []
    let rd word "r" i
    set elm lput rd elm
    let j 0
    while [j < length nodes_roads] [
      let temp (item j nodes_roads)
      if member? rd temp [set elm lput (item 0 temp) elm]
      set j j + 1
    ]
    set roads_connect lput elm roads_connect
    set i i + 1
  ]

  foreach roads_connect [
    let elm ?
    let ind position elm roads_connect
    let dir ([direct] of one-of roads with [id = item 0 elm])
    if dir != "bi" [
      let n1 item (position (item 1 elm) nodes_list + 1) nodes_list
      let n2 item (position (item 2 elm) nodes_list + 1) nodes_list
      ifelse dir = 0 [
        if [pycor] of n1 > [pycor] of n2 [
          set elm replace-item 1 elm (item (position n2 nodes_list - 1) nodes_list)
          set elm replace-item 2 elm (item (position n1 nodes_list - 1) nodes_list)
        ]
      ]
      [
        ifelse dir = 90 [
          if [pxcor] of n1 > [pxcor] of n2 [
            set elm replace-item 1 elm (item (position n2 nodes_list - 1) nodes_list)
            set elm replace-item 2 elm (item (position n1 nodes_list - 1) nodes_list)
          ]
        ]
        [
          ifelse dir = 180 [
            if [pycor] of n1 < [pycor] of n2 [
              set elm replace-item 1 elm (item (position n2 nodes_list - 1) nodes_list)
              set elm replace-item 2 elm (item (position n1 nodes_list - 1) nodes_list)
            ]
          ]
          [
            if [pxcor] of n1 < [pxcor] of n2 [
              set elm replace-item 1 elm (item (position n2 nodes_list - 1) nodes_list)
              set elm replace-item 2 elm (item (position n1 nodes_list - 1) nodes_list)
            ]
          ]
        ]
      ]
      set roads_connect replace-item ind roads_connect elm
    ]
  ]
end


to find-shortest-paths ;;;Floyd–Warshall Algorithm
  set inf 1000000
  set shortest_distance_matrix []
  set shortest_distance_nodes []
  foreach nodes_connect-updated [
    let temp []
    let temp2 []
    let i 0
    while [i < length nodes_connect-updated] [
      let k 2
      ifelse i = item 0 ? [
        set temp lput 0 temp
        set temp2 lput i temp2
      ]
      [
        let j k
        while [j < length ?] [
          ifelse i = item 0 (item j ?) [
            set temp lput (item 2 (item j ?)) temp
            set temp2 lput (item 0 ?) temp2
            set j 2 * length ?
            set k j
          ]
          [
            set j j + 1
          ]
        ]
        if j = length ? [
          set temp lput inf temp
          set temp2 lput "" temp2
        ]
      ]
      set i i + 1
    ]
    set shortest_distance_matrix lput temp shortest_distance_matrix
    set shortest_distance_nodes lput temp2 shortest_distance_nodes
  ]

  let k 0
  while [k < length nodes_connect-updated] [
    let i 0
    while [i < length nodes_connect-updated] [
      let j 0
      while [j < length nodes_connect-updated] [
        let temp3 (item i shortest_distance_matrix)
        let temp4 (item i shortest_distance_nodes)
        if (item k temp3) + (item j (item k shortest_distance_matrix)) < (item j temp3) [
          set temp3 (replace-item j temp3 ((item k temp3) + (item j (item k shortest_distance_matrix))))
          set temp4 (replace-item j temp4 k)
          set shortest_distance_matrix (replace-item i shortest_distance_matrix temp3)
          set shortest_distance_nodes (replace-item i shortest_distance_nodes temp4)
        ]
        set j j + 1
      ]
      set i i + 1
    ]
    set k k + 1
  ]

end

to go
  if movie? and ticks < 500 [movie-grab-interface]
  sprout-people-from-buildings

  ;;;;;;;;; set traffic lights
  if ticks mod TL_period = 0 [set green_direction (green_direction + 90) mod 180]
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  move-cars

  ask people [
    if algorithm = "Bug1" [walk-bug1]
    if algorithm = "Bug2" [
      if not follow-wall? [create-mline]
      walk-bug2
    ]
    if algorithm = "DistBug" [walk-distbug]
    if algorithm = "TangentBug" [
      ifelse transition1? [
        ifelse patch-here != H [walk-watch-cars] [
          set transition1? false
          set follow-wall? true
          while [wall? 0 wspeed != nobody] [
            lt direction * 44
          ]
          if (round (heading / 90) = 0) or (round (heading / 90) = 4) [set heading 0]
          if round (heading / 90) = 1 [set heading 90]
          if round (heading / 90) = 2 [set heading 180]
          if round (heading / 90) = 3 [set heading 270]
        ]
      ]
      [
        ifelse transition2? [
          ifelse distance target < d-min [
            face target
            set transition2? false
            set H nobody
          ]
          [walk-watch-cars]
        ]
        [
          ifelse follow-wall? [follow-boundary] [motion-toward-target]
        ]
      ]
    ]
    if algorithm = "K-Bug" [walk-kbug]
    if algorithm = "New Algorithm" [walk-new]
    if algorithm = "New Algorithm2" [walk-new2]
    if algorithm = "COPE0" [walk-cope0]
  ]
  tick
  ;if any? patches with [count cars-here > 0 and count people-here > 0] [stop]
  if movie? and ticks = 500 [movie-close]
  if count cars = 0 and count people = 0 and ticks > max-end-time [show timer reset-timer show count trappeds stop] ;
end

to sprout-people-from-buildings
  ask exit-doors [
    let i (list 0 1 2)
    let k (list 0 0 0)
    if sum pop_list != 0 [
      foreach i [
        if item ? pop_list != 0 [
          if ticks >= (item ? ini) [set k (replace-item ? k ((item ? rate) * (item ? n_bld_list)))]
          ifelse item ? k < item ? pop_list [
            let gen floor (item ? k + item ? rate-ct)
            set gen-pop gen-pop + gen
            set rate-ct (replace-item ? rate-ct (item ? k + item ? rate-ct - gen))
            set k (replace-item ? k gen)
            set pop_list (replace-item ? pop_list (item ? pop_list - gen))
          ]
          [
            let gen round (item ? pop_list)
            set k (replace-item ? k gen)
            set gen-pop gen-pop + gen
            set pop_list (replace-item ? pop_list 0)
          ]
        ]
      ]
    ]
    sprout-people (sum k) [setup-people setup-cars]
  ]
end


to move-cars
  ask cars [
    let stp 0
    while [stp < speed] [
;      ;ifelse count (cars-on patches with [id = [id] of ([patch-here] of myself)]) with [self != myself and distance myself < 3 * [speed] of myself + [car-size] of myself + safety_dist] < [lanes] of patch-here [
;      ifelse count (cars-on patches with [id = [id] of ([patch-here] of myself)]) with [self != myself and distance myself < ([speed] of myself) ^ 2 / 10 + [car-size] of myself + safety_dist] < [lanes] of patch-here [
;        set speed speed + acceleration
;        if speed > speed_limit [set speed speed_limit]
;      ]
;      [
;        set speed speed - deceleration
;        if speed < 0 [set speed 0]
;      ]

      ifelse patch-here = target [
        let current-id item ((position target nodes_list) - 1) nodes_list
        let dest-id item ((position destination nodes_list) - 1) nodes_list

        let next_node item dest-id (item current-id shortest_distance_nodes)
        let last_node dest-id
        while [next_node != current-id and next_node != last_node] [
          set last_node next_node
          set next_node item last_node (item current-id shortest_distance_nodes)
        ]

        let next-road nobody
        let i 2
        while [i < length (item current-id nodes_connect)] [
          if item 0 (item i (item current-id nodes_connect)) = last_node [set next-road item 1 (item i (item current-id nodes_connect))]
          set i i + 1
        ]

        let prev patch-here
        ;move-to min-one-of roads with [id = next-road and not member? self intersections] [distance myself]
        move-to min-one-of roads in-radius 2 with [id = next-road and not member? self intersections] [distance myself]
        set target item ((position last_node nodes_list) + 1) nodes_list
        set speed [speed_limit] of patch-here
        face target
        if (any? people-here) or (count (cars with [self != myself and not member? self cars-on prev and heading = [heading] of myself]) in-cone (car-size + safety_dist) 90 >= [lanes] of patch-here) [
          move-to prev
          set target prev
          set stp speed + 1
        ]
      ]
      [
        let nxt min-one-of neighbors4 with [member? self roads] [distance [target] of myself]
        ifelse not any? people-on nxt [
          ifelse nxt = target [
            ifelse (round (heading / 90) = 0) or (round (heading / 90) = 4) [set heading 0] [
              ifelse round (heading / 90) = 1 [set heading 90] [
                ifelse round (heading / 90) = 2 [set heading 180] [set heading 270]
              ]
            ]
            if (count cars-on nxt < [lanes] of nxt) [
              if ([traffic_light] of nxt = 0) or ([traffic_light] of nxt = 1 and (heading = green_direction or heading = green_direction + 180)) [move-to nxt]
            ]
          ]
          [
            let cur patch-here
            if count (cars with [self != myself and not member? self cars-on cur and heading = [heading] of myself]) in-cone (car-size + safety_dist) 90 < [lanes] of nxt [move-to nxt]
          ]
        ]
        [
          set stp speed + 1
        ]
      ]

      if patch-here = destination [
        ask riders [die]
        die
      ]
      set stp stp + 1
    ]
  ]
end


to setup-people-from-gis
  ;ask people [die]
  ;ask trappeds [die]
  ct
  ask patches [set plabel ""]
  output-print "NEW RUN:"

  set max-speed-ped mu-speed + 2 * std-speed

  ask roads [
    if ttype = "local" [set speed_limit 15.6 / max-speed-ped]    ;; 15.6 m/s = 35 mph
    if ttype = "highway" [set speed_limit 24.6 / max-speed-ped]  ;; 24.6 m/s = 55 mph
  ]

  ask patches [set walkable 1]
  ask patches with [pcolor = brown] [set walkable 0]
  ask patches with [pxcor = max-pxcor or pycor = max-pycor or pxcor = min-pxcor or pycor = min-pycor] [set walkable 1]
  ask patches [set suggested 0]
  ask patches with [pcolor = yellow or pcolor = red or pcolor = black] [set suggested 1]

  if walk-on-streets? [
    ask patches [set traffic_light 0]
    ask roads [set walkable 1]
    ;ask patches with [pcolor = brown] [set walkable 0]
    ask patches with [pxcor = max-pxcor or pycor = max-pycor or pxcor = min-pxcor or pycor = min-pycor] [set walkable 0]
  ]

  set loc []
  set time []
  set e1 0
  set e2 0
  set max-pop (scale / 0.5) ^ 2
  set gen-pop 0
  set exit-doors []
  set class (list 1 3 12)
  let max-id max [id] of patches
  let p 1
  while [p <= max-id] [
    let temp one-of patches with [id = p]
    ask patches with [id = p] [set main-gate temp]
    ask temp [
      let gate patches with [id = p and count patches in-radius 2 with [walkable = 1] >= 4]
      set exits ([neighbors4] of gate) set exits patch-set exits
      let avail-exits exits with [walkable = 1 and not member? self exit-doors]
      set exits avail-exits
      ;ifelse count avail-exits >= n_bld [set exits n-of n_bld avail-exits] [set exits avail-exits]
      set exit-doors (patch-set main-gate exit-doors)
      let tf (high * 12 + med * 3 + low * 1 + sch * 3)   ;; total number of floors in the building block
      let pf 0
      if tf != 0 [set pf pop / tf]                       ;; average population of each floor
      let i (list 0 1 2)
      set pop_list (list 0 0 0)
      set pop-class (list 0 0 0)
      set rate (list 0 0 0)
      set n_bld_list (list low (med + sch) high)
      set rate-ct (list 0 0 0)
      let ini_low random-normal 24.3 5.5
      let ini_med random-normal 24.7 6
      let ini_high random-normal 24.7 6.3
      let end_low random-normal 64.2 7.9
      let end_med random-normal 103.5 11.9
      let end_high random-normal 251.6 38.9
      set ini (list (ini_low * max-speed-ped / scale) (ini_med * max-speed-ped / scale) (ini_high * max-speed-ped / scale))
      set endd (list (end_low * max-speed-ped / scale) (end_med * max-speed-ped / scale) (end_high * max-speed-ped / scale))
      ;let min-ini min (list ini_low ini_med ini_high)
      ;set ini (list ((ini_low - min-ini) * 60 * max-speed-ped / scale) ((ini_med - min-ini) * 60 * max-speed-ped / scale) ((ini_high - min-ini) * 60 * max-speed-ped / scale))
      ;set endd (list ((end_low - min-ini) * 60 * max-speed-ped / scale) ((end_med - min-ini) * 60 * max-speed-ped / scale) ((end_high - min-ini) * 60 * max-speed-ped / scale))
      foreach i [
        set pop-class replace-item ? pop-class (pf * (item ? class))
        set rate replace-item ? rate ((item ? pop-class) / ((item ? endd) - (item ? ini)))
        set pop_list (replace-item ? pop_list ((item ? pop-class) * (item ? n_bld_list)))
      ]
    ]
    set p p + 1
  ]

  set max-end-time max ([item 2 endd] of exit-doors)
  set green_direction (random 2) * 90
  set TL_period-scaled round (TL_period * max-speed-ped / scale)
  reset-ticks
  reset-timer
end


to create-people-on-streets
  ;;pedestrian distribution zones
  ;; in clockwise direction
  let hi1 (patch -539 224)
  let hi2 (patch -270 210)
  let hi3 (patch -55 -192)
  let hi4 (patch -284 -185)
  let hi-patches patches with [pycor < (([pycor] of hi2 - [pycor] of hi1) / ([pxcor] of hi2 - [pxcor] of hi1)) * (pxcor - [pxcor] of hi1) + [pycor] of hi1]
  set hi-patches hi-patches with [pycor > (([pycor] of hi4 - [pycor] of hi3) / ([pxcor] of hi4 - [pxcor] of hi3)) * (pxcor - [pxcor] of hi3) + [pycor] of hi3]
  set hi-patches hi-patches with [pycor < (([pycor] of hi3 - [pycor] of hi2) / ([pxcor] of hi3 - [pxcor] of hi2)) * (pxcor - [pxcor] of hi2) + [pycor] of hi2]
  set hi-patches hi-patches with [pycor > (([pycor] of hi4 - [pycor] of hi1) / ([pxcor] of hi4 - [pxcor] of hi1)) * (pxcor - [pxcor] of hi1) + [pycor] of hi1]
  set hi-patches hi-patches with [walkable = 1]

  let med1 (patch -270 210)
  let med2 (patch 35 195)
  let med3 (patch 255 -207)
  let med4 (patch -55 -192)
  let med-patches patches with [pycor < (([pycor] of med2 - [pycor] of med1) / ([pxcor] of med2 - [pxcor] of med1)) * (pxcor - [pxcor] of med1) + [pycor] of med1]
  set med-patches med-patches with [pycor > (([pycor] of med4 - [pycor] of med3) / ([pxcor] of med4 - [pxcor] of med3)) * (pxcor - [pxcor] of med3) + [pycor] of med3]
  set med-patches med-patches with [pycor < (([pycor] of med3 - [pycor] of med2) / ([pxcor] of med3 - [pxcor] of med2)) * (pxcor - [pxcor] of med2) + [pycor] of med2]
  set med-patches med-patches with [pycor > (([pycor] of med4 - [pycor] of med1) / ([pxcor] of med4 - [pxcor] of med1)) * (pxcor - [pxcor] of med1) + [pycor] of med1]
  set med-patches med-patches with [walkable = 1]

  let low1 (patch 35 195)
  let low2 (patch 284 182)
  let low3 (patch 484 -213)
  let low4 (patch 255 -207)
  let low-patches patches with [pycor < (([pycor] of low2 - [pycor] of low1) / ([pxcor] of low2 - [pxcor] of low1)) * (pxcor - [pxcor] of low1) + [pycor] of low1]
  set low-patches low-patches with [pycor > (([pycor] of low4 - [pycor] of low3) / ([pxcor] of low4 - [pxcor] of low3)) * (pxcor - [pxcor] of low3) + [pycor] of low3]
  set low-patches low-patches with [pycor < (([pycor] of low3 - [pycor] of low2) / ([pxcor] of low3 - [pxcor] of low2)) * (pxcor - [pxcor] of low2) + [pycor] of low2]
  set low-patches low-patches with [pycor > (([pycor] of low4 - [pycor] of low1) / ([pxcor] of low4 - [pxcor] of low1)) * (pxcor - [pxcor] of low1) + [pycor] of low1]
  set low-patches low-patches with [walkable = 1]
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  let no-pedest 13300
  ask n-of (0.5 * no-pedest) hi-patches [sprout-people 1 [setup-pedestrians]]
  ask n-of (0.3 * no-pedest) med-patches [sprout-people 1 [setup-pedestrians]]
  ask n-of (0.2 * no-pedest) low-patches [sprout-people 1 [setup-pedestrians]]
  set gen-pop no-pedest
end

to create-cars-on-streets
  ask n-of 200 roads with [not member? self intersections] [sprout-idrivers 1 [setup-icars]]
end



to setup-people
  ;let out one-of exits with [count people-here < max-pop]
  ;ifelse out != nobody [move-to out] [move-to one-of patches with [count people-here < max-pop and walkable = 1]]
  move-to one-of exits ;with [count people-here < max-pop]
  set pre patch-here
  set color green
  set shape "circle"
  set target min-one-of target-set [distance myself]
  face target
  set size 5
  ifelse random 100 < car_ownership [set car? true] [set car? false]
  set follow-wall? false
  set wspeed max-speed-ped * 2
  while [wspeed > max-speed-ped or wspeed < (mu-speed - 2 * std-speed)] [set wspeed random-normal mu-speed std-speed]
  set wspeed (wspeed / max-speed-ped)
  ;set wspeed (random-weibull 1.41 10.14) / max-speed-ped
  set tolerance (tolerance_time * max-speed-ped / scale)
  set H nobody
  ifelse random 2 = 0 [set direction 1] [set direction -1]
  set r max-pxcor / 2
  if algorithm = "Bug1" [
    set L nobody
    set search-mode? false
  ]
  if algorithm = "Bug2" or algorithm = "New Algorithm2" [create-mline]
  if algorithm = "DistBug" [
    set step r   ;; set value to that of r (unless it will be problematic!)
    set d-min 0
    set m-line []
    set extension2? false
    set extension3? false
    set vir-obs []
  ]
  if algorithm = "TangentBug" [
    set ltg-list []
    set transition1? false
    set transition2? false
  ]
  if algorithm = "K-Bug" [
    set edge nobody
    set node nobody
    ifelse not target-visible? target [set H find-wall (towards target) r] [set H nobody]
    set last-node nobody
    ifelse H != nobody [
      find-node
      ifelse node = nobody [show "initial error: dead" die] [face node]
    ]
    [
      face target
    ]
  ]
  if algorithm = "New Algorithm" [
    set step r   ;; set value to that of r (unless it will be problematic!)
    set d-min 0
    set m-line []
  ]
  if algorithm = "COPE0" [
        set step r   ;; set value to that of R (unless it will be problematic!)
        set d-min 0
  ]
end

to setup-pedestrians
  set pre patch-here
  set color green
  set shape "circle"
  set target min-one-of target-set [distance myself]
  face target
  set size 5
  set car? false
  set follow-wall? false
  set wspeed max-speed-ped * 2
  while [wspeed > max-speed-ped or wspeed < (mu-speed - 2 * std-speed)] [set wspeed random-normal mu-speed std-speed]
  set wspeed (wspeed / max-speed-ped)
  ;set wspeed (random-weibull 1.41 10.14) / max-speed-ped
  set tolerance (tolerance_time * max-speed-ped / scale)
  set H nobody
  ifelse random 2 = 0 [set direction 1] [set direction -1]
  set r max-pxcor / 2
  if algorithm = "Bug1" [
    set L nobody
    set search-mode? false
  ]
  if algorithm = "Bug2" or algorithm = "New Algorithm2" [create-mline]
  if algorithm = "DistBug" [
    set step r   ;; set value to that of r (unless it will be problematic!)
    set d-min 0
    set m-line []
    set extension2? false
    set extension3? false
    set vir-obs []
  ]
  if algorithm = "TangentBug" [
    set ltg-list []
    set transition1? false
    set transition2? false
  ]
  if algorithm = "K-Bug" [
    set edge nobody
    set node nobody
    ifelse not target-visible? target [set H find-wall (towards target) r] [set H nobody]
    set last-node nobody
    ifelse H != nobody [find-node face node] [face target]
  ]
  if algorithm = "New Algorithm" [
    set step r   ;; set value to that of r (unless it will be problematic!)
    set d-min 0
    set m-line []
  ]
  if algorithm = "COPE0" [
        set step r   ;; set value to that of R (unless it will be problematic!)
        set d-min 0
  ]
end

to walk-bug1
  if patch-here = target [die]
  if follow-wall? [
    let c1 wall-p? patch-here (heading + 90 * direction) 1
    let c2 wall-p? patch-here (heading + 135 * direction) 1
    if c1 = nobody and c2 != nobody [rt direction * 90]
  ]
  if wall? 0 wspeed != nobody [
    ifelse follow-wall? [
;;;;;;;;;;;; changing direction when agent gets stuck at the end of a corridor
      ifelse wall? 0 1 = patch-here and wall? (90 * direction) 1 != nobody [
        set direction -1 * direction
        set heading heading + 180
      ]
;;;;;;;;;;;;
      [
        while [wall? 0 wspeed != nobody] [lt direction * 90]
      ]
    ]
    [
      set search-mode? true
      set follow-wall? true
      set H patch-here
      set Q distance target
      set L patch-here
      while [wall? 0 wspeed != nobody] [lt direction * 44.5]
      ifelse (round (heading / 90) = 0) or (round (heading / 90) = 4) [set heading 0] [
        ifelse round (heading / 90) = 1 [set heading 90] [
          ifelse round (heading / 90) = 2 [set heading 180] [set heading 270]
        ]
      ]
      while [wall? 0 wspeed != nobody] [lt direction * 90]
    ]
  ]

  ifelse distance target <= 1 [move-to target die] [
    let people_ahead (turtles-on patch-ahead wspeed) with [self != myself and breed = people]
    ifelse (human-interaction? = false) or (count people_ahead < max-pop) or abs(abs(heading - [heading] of (one-of people_ahead)) - 180) <= 90 [
      walk-watch-cars
      ifelse search-mode? [
        set R2 R2 + wspeed
        set R3 R3 + wspeed
        if distance target < Q [
          set Q distance target
          set L patch-here
          set R3 0
        ]
        if patch-here = H and patch-right-and-ahead 180 wspeed != H and L != nobody [
          set search-mode? false
          if (R2 / 2) > R3 [
            set heading heading + 180
            set direction -1 * direction
          ]
        ]
      ]
      [
        if patch-here = L [
          set follow-wall? false
          set search-mode? false
          face target
          set R2 0
          set R3 0
          set Q 0
          set H nobody
          set L nobody
          if wall? 0 wspeed != nobody [
            ask patch 10 0 [set plabel "Agent is trapped!" set plabel-color yellow]
            ;output-type "Agent is trapped on: " output-print patch-here
            set breed trappeds
            set shape "face sad"
            set size 2
          ]
        ]
      ]
    ]
    [
      if human-interaction? [
        set tol-count tol-count + 1
        if tol-count > tolerance [
          set tol-count 0
          let alter one-of neighbors with [walkable = 1 and count people-here < max-pop]
          if alter != nobody [
            move-to alter
            if follow-wall? [
              set follow-wall? false
              set search-mode? false
              set R2 0
              set R3 0
              set Q 0
              set H nobody
              set L nobody
            ]
            face target
          ]
        ]
      ]
    ]
  ]
end

to walk-bug2
  if patch-here = target [die]
  if follow-wall? [
    let c1 wall-p? patch-here (heading + 90 * direction) 1
    let c2 wall-p? patch-here (heading + 135 * direction) 1
    if c1 = nobody and c2 != nobody [rt direction * 90]
  ]
  if wall? 0 wspeed != nobody [
    ifelse follow-wall? [
;;;;;;;;;;;; changing direction when agent gets stuck at the end of a corridor
      ifelse wall? 0 1 = patch-here and wall? (90 * direction) 1 != nobody [
        set direction -1 * direction
        set heading heading + 180
      ]
;;;;;;;;;;;;
      [
        while [wall? 0 wspeed != nobody] [lt direction * 90]
      ]
    ]
    [
      set follow-wall? true
      set H patch-here
      while [wall? 0 wspeed != nobody] [lt direction * 44.5]
      ifelse (round (heading / 90) = 0) or (round (heading / 90) = 4) [set heading 0] [
        ifelse round (heading / 90) = 1 [set heading 90] [
          ifelse round (heading / 90) = 2 [set heading 180] [set heading 270]
        ]
      ]
      while [wall? 0 wspeed != nobody] [lt direction * 90]
    ]
  ]

  ifelse distance target <= 1 [move-to target die] [
    let people_ahead (turtles-on patch-ahead wspeed) with [self != myself and breed = people]
    ifelse (human-interaction? = false) or (count people_ahead < max-pop) or abs(abs(heading - [heading] of (one-of people_ahead)) - 180) <= 90 [
      walk-watch-cars
      if follow-wall? [
        if member? patch-here m-line and patch-here != H and distance target < distance2v H target [
          set follow-wall? false
          face target
          set H nobody
        ]
        if patch-here = H and patch-right-and-ahead 180 wspeed != H [
          ask patch 10 0 [set plabel "Agent is trapped!" set plabel-color yellow]
          ;output-type "Agent is trapped on: " output-print patch-here
          set breed trappeds
          set shape "face sad"
          set size 2
        ]
      ]
    ]
    [
      if human-interaction? [
        set tol-count tol-count + 1
        if tol-count > tolerance [
          set tol-count 0
          let alter one-of neighbors with [walkable = 1 and count people-here < max-pop]
          if alter != nobody [
            move-to alter
            if follow-wall? [
              set follow-wall? false
              set H nobody
            ]
            face target
          ]
        ]
      ]
    ]
  ]

end

to walk-distbug
  if patch-here = target [die]
  if follow-wall? [
;;;;;;;;;;;; extension 2
if Extension-2 = true [
    if not extension2? [
      let a 0
      set a atan ([pxcor] of target - xcor) ([pycor] of target - ycor) - heading
      ifelse abs(a) > 180 [set a 360 - abs(a)] [set a abs(a)]
      if a > 135 [
        set direction -1 * direction
        set heading heading + 180
        set extension2? true
      ]
    ]
]
;;;;;;;;;;;;
    let c1 wall-p? patch-here (heading + 90 * direction) 1
    let c2 wall-p? patch-here (heading + 135 * direction) 1
    if c1 = nobody and c2 != nobody [rt direction * 90]
  ]
  if wall? 0 wspeed != nobody [
    ifelse follow-wall? [
;;;;;;;;;;;; changing direction when agent gets stuck at the end of a corridor
      ifelse wall? 0 1 = patch-here and wall? (90 * direction) 1 != nobody [
        set direction -1 * direction
        set heading heading + 180
      ]
;;;;;;;;;;;;
      [
        while [wall? 0 wspeed != nobody] [lt direction * 90]
      ]
    ]
    [
      set follow-wall? true
      set H patch-here
      create-mline
      set d-min distance target
      if Extension-3 = true [build-virtual-obstacles distance target]
;;;;;;;;;;;; extension 1
ifelse Extension-1 = true [
      ifelse heading mod 90 = 0 [lt 90 * direction] [
        let horz false
        let vert false
        if wall-p? patch-here 0 1 != nobody or wall-p? patch-here 180 1 != nobody [set horz true]
        if wall-p? patch-here 90 1 != nobody or wall-p? patch-here 270 1 != nobody [set vert true]
        if horz and not vert [
          ifelse heading > 0 and heading < 180 [
            ifelse heading < 90 [set direction -1] [set direction 1]
            set heading 90
            ]
          [
            ifelse heading < 270 [set direction -1] [set direction 1]
            set heading 270
            ]
        ]
        if not horz and vert [
          ifelse heading > 90 and heading < 270 [
            ifelse heading < 180 [set direction -1] [set direction 1]
            set heading 180
            ]
          [
            ifelse heading < 90 [set direction 1] [set direction -1]
            set heading 0
            ]
        ]

        if (horz and vert) or (not horz and not vert) [
          while [wall? 0 wspeed != nobody] [lt direction * 44.5]
          ifelse (round (heading / 90) = 0) or (round (heading / 90) = 4) [set heading 0] [
            ifelse round (heading / 90) = 1 [set heading 90] [
              ifelse round (heading / 90) = 2 [set heading 180] [set heading 270]
            ]
          ]
          while [wall? 0 wspeed != nobody] [lt direction * 90]
        ]
      ]
      while [wall? 0 wspeed != nobody] [lt direction * 90]
]
;;;;;;;;;;;;
[
  while [wall? 0 wspeed != nobody] [lt direction * 44.5]
  ifelse (round (heading / 90) = 0) or (round (heading / 90) = 4) [set heading 0] [
    ifelse round (heading / 90) = 1 [set heading 90] [
      ifelse round (heading / 90) = 2 [set heading 180] [set heading 270]
    ]
  ]
  while [wall? 0 wspeed != nobody] [lt direction * 90]
]
    ]
  ]

  ifelse distance target <= 1 [move-to target die] [
    let people_ahead (turtles-on patch-ahead wspeed) with [self != myself and breed = people]
    ifelse (human-interaction? = false) or (count people_ahead < max-pop) or abs(abs(heading - [heading] of (one-of people_ahead)) - 180) <= 90 [
      walk-watch-cars
      if follow-wall? [
;;;;;;;;;;;; extension 3
if Extension-3 = true [
  if member? patch-here vir-obs [
    ifelse extension3? [
      build-virtual-obstacles (distance target + r / 2)
      ]
    [
      set direction -1 * direction
      set heading heading + 180
      set extension3? true
      let c1 wall-p? patch-here (heading + 90 * direction) 1
      let c2 wall-p? patch-here (heading + 135 * direction) 1
      if c1 = nobody and c2 != nobody [ifelse human-interaction? [interaction-walk] [fd wspeed]]   ;;; to prevent agents from being stuck between corners of two obstacles (an identified problem)
    ]
  ]
]
;;;;;;;;;;;;
        if distance target < d-min [set d-min distance target]
        if target-visible? target or distance target - r < d-min - step or (member? patch-here m-line and patch-here != H and distance target < distance2v H target) [
          set follow-wall? false
          face target
          set d-min 0
          set H nobody
          set m-line []
          set extension2? false
          set cn 0
          set extension3? false
          set vir-obs []
        ]
        if patch-here = H and patch-right-and-ahead 180 wspeed != H [
          ifelse cn = 1 or Extension-2 = false [
            ask patch 10 0 [set plabel "Agent is trapped!" set plabel-color yellow]
            ;output-type "Agent is trapped on: " output-print patch-here
            set breed trappeds
            set shape "face sad"
            set size 2
          ]
          [set cn 1]
        ]
      ]
    ]
    [
      if human-interaction? [
        set tol-count tol-count + 1
        if tol-count > tolerance [
          set tol-count 0
          let alter one-of neighbors with [walkable = 1 and count people-here < max-pop]
          if alter != nobody [
            move-to alter
            if follow-wall? [
              set follow-wall? false
              set d-min 0
              set H nobody
              set m-line []
              set extension2? false
              set cn 0
              set extension3? false
              set vir-obs []
            ]
            face target
          ]
        ]
      ]
    ]
  ]
end

to walk-kbug
  set step_forward? false
  if patch-here = target [die]
  if find-wall (towards target) r != nobody and node = nobody [set node patch-here]
  ifelse patch-here != node [set step_forward? true] [
    set node nobody
    ifelse target-visible? target or find-wall (towards target) r = nobody [face target set step_forward? true] [
      find-node
      set last-node patch-here
      ifelse node = nobody [show "extra error: dead" die] [face node]
      set step_forward? true
    ]
  ]

  if step_forward? [
    ifelse human-interaction? [
      let people_ahead (turtles-on patch-ahead wspeed) with [self != myself and breed = people]
      ifelse count people_ahead < max-pop or abs(abs(heading - [heading] of (one-of people_ahead)) - 180) <= 90 [
        walk-watch-cars
      ]
      [
        set tol-count tol-count + 1
        if tol-count > tolerance [
          set tol-count 0
          let alter one-of neighbors with [walkable = 1 and count people-here < max-pop]
          if alter != nobody [
            move-to alter
            set node nobody
            face target
          ]
        ]
      ]
    ]
    [fd wspeed]
  ]
end


to walk-new
  if patch-here = target [die]
  if member? patch-here [neighbors] of target [die]
  if follow-wall? [
    let c1 wall-p? patch-here (heading + 90 * direction) 1
    let c2 wall-p? patch-here (heading + 135 * direction) 1
    if c1 = nobody and c2 != nobody [rt direction * 90]
  ]

  if wall? 0 wspeed != nobody [
    ifelse follow-wall? [
;;;;;;;;;;;; changing direction when agent gets stuck at the end of a corridor
      ifelse wall? 0 1 = patch-here and wall? (90 * direction) 1 != nobody [
        set direction -1 * direction
        set heading heading + 180
      ]
;;;;;;;;;;;;
      [
        while [wall? 0 wspeed != nobody] [lt direction * 90]
      ]
    ]
    [
      set follow-wall? true
      set H patch-here
      create-mline
      set d-min distance target
;;;;;;;;;;;; extension 1
      ifelse heading mod 90 = 0 [lt 90 * direction] [
        let horz false
        let vert false
        if wall-p? patch-here 0 1 != nobody or wall-p? patch-here 180 1 != nobody [set horz true]
        if wall-p? patch-here 90 1 != nobody or wall-p? patch-here 270 1 != nobody [set vert true]
        if horz and not vert [
          ifelse heading > 0 and heading < 180 [
            ifelse heading < 90 [set direction -1] [set direction 1]
            set heading 90
          ]
          [
            ifelse heading < 270 [set direction -1] [set direction 1]
            set heading 270
          ]
        ]
        if not horz and vert [
          ifelse heading > 90 and heading < 270 [
            ifelse heading < 180 [set direction -1] [set direction 1]
            set heading 180
          ]
          [
            ifelse heading < 90 [set direction 1] [set direction -1]
            set heading 0
          ]
        ]

        if (horz and vert) or (not horz and not vert) [
          while [wall? 0 wspeed != nobody] [lt direction * 44.9]
          ifelse (round (heading / 90) = 0) or (round (heading / 90) = 4) [set heading 0] [
            ifelse round (heading / 90) = 1 [set heading 90] [
              ifelse round (heading / 90) = 2 [set heading 180] [set heading 270]
            ]
          ]
        ]
      ]
      while [wall? 0 wspeed != nobody] [lt direction * 90]
;;;;;;;;;;;;
    ]
  ]

  ifelse distance target <= 1 [move-to target die] [
    let people_ahead (turtles-on patch-ahead wspeed) with [self != myself and breed = people]
    ifelse (human-interaction? = false) or (count people_ahead < max-pop) or abs(abs(heading - [heading] of (one-of people_ahead)) - 180) < 90 [
      walk-watch-cars
      if follow-wall? [
        if distance target < d-min [set d-min distance target]
        if target-visible? target or distance target - r < d-min - step or (member? patch-here m-line and patch-here != H and distance target < distance2v H target) [
          set follow-wall? false
          face target
          set d-min 0
          set H nobody
          set m-line []
        ]
      ]
    ]
    [
      set tol-count tol-count + 1
      if tol-count > tolerance [
        ;set tol-count 0
        let alter one-of neighbors with [walkable = 1 and count people-here < max-pop]
        if alter != nobody [
          set tol-count 0
          move-to alter
          if follow-wall? [
            set follow-wall? false
            set d-min 0
            set H nobody
            set m-line []
          ]
          face target
        ]
      ]
    ]
  ]
end


to walk-new2 ;;; another new algorithm based on old-alg
  if patch-here = target [die]
  if member? patch-here [neighbors] of target [die]
  if follow-wall? [
    let c1 wall-p? patch-here (heading + 90 * direction) 1
    let c2 wall-p? patch-here (heading + 135 * direction) 1
    if c1 = nobody and c2 != nobody [
      ifelse distance2v-max c2 target > br-dist [rt 90 * direction] [
        face target
        set follow-wall? false
      ]
    ]
  ]

  if wall? 0 wspeed != nobody [
    ifelse follow-wall? [
;;;;;;;;;;;; changing direction when agent gets stuck at the end of a corridor
      ifelse wall? 0 1 = patch-here and wall? (90 * direction) 1 != nobody [
        set direction -1 * direction
        set heading heading + 180
      ]
;;;;;;;;;;;;
      [
        while [wall? 0 wspeed != nobody] [lt direction * 90]
      ]
    ]
    [
      set follow-wall? true
      set H patch-here
      set int-target wall? 0 wspeed
      set n 1 + wspeed
      while [[walkable] of int-target = 0] [
        set int-target patch-at-heading-and-distance heading n
        set n n + 1
      ]
      set br-dist distance2v-max int-target target
      while [wall? 0 wspeed != nobody] [lt direction * 44.5]
      ifelse (round (heading / 90) = 0) or (round (heading / 90) = 4) [set heading 0] [
        ifelse round (heading / 90) = 1 [set heading 90] [
          ifelse round (heading / 90) = 2 [set heading 180] [set heading 270]
        ]
      ]
      while [wall? 0 wspeed != nobody] [lt direction * 90]
    ]
  ]

  ifelse distance target <= 1 [move-to target die] [
    let people_ahead (turtles-on patch-ahead wspeed) with [self != myself and breed = people]
    ifelse (human-interaction? = false) or (count people_ahead < max-pop) or abs(abs(heading - [heading] of (one-of people_ahead)) - 180) < 90 [
      walk-watch-cars
      if follow-wall? [
        if patch-here = int-target [
          set follow-wall? false
          face target
          set H nobody
        ]
      ]
    ]
    [
      set tol-count tol-count + 1
      if tol-count > tolerance [
        ;set tol-count 0
        let alter one-of neighbors with [walkable = 1 and count people-here < max-pop]
        if alter != nobody [
          set tol-count 0
          move-to alter
          if follow-wall? [
            set follow-wall? false
            set H nobody
          ]
          face target
        ]
      ]
    ]
  ]
end

to walk-cope0
  if patch-here = target [
    let nearest-target min-one-of target-set [distance [target] of myself]
    if nearest-target = (patch 280 125) or nearest-target = (patch 316 123) [set e1 e1 + 1]
    if nearest-target = (patch 345 -162) or nearest-target = (patch 369 -163) or nearest-target = (patch 394 -165) or nearest-target = (patch 420 -168) [set e2 e2 + 1]
    ;set loc lput pre loc
    ;set time lput ticks time
    die

  ]
  if member? patch-here [neighbors] of target [
    let nearest-target min-one-of target-set [distance [target] of myself]
    if nearest-target = (patch 280 125) or nearest-target = (patch 316 123) [set e1 e1 + 1]
    if nearest-target = (patch 345 -162) or nearest-target = (patch 369 -163) or nearest-target = (patch 394 -165) or nearest-target = (patch 420 -168) [set e2 e2 + 1]
    ;set loc lput pre loc
    ;set time lput ticks time
    die
  ]
  if follow-wall? [
    let c1 wall-p? patch-here (heading + 90 * direction) 1
    let c2 wall-p? patch-here (heading + 135 * direction) 1
    if c1 = nobody and c2 != nobody [
      ifelse distance2v c2 target > br-dist [rt 90 * direction] [
        face target
        set follow-wall? false
      ]
    ]
  ]

  if wall? 0 wspeed != nobody [
    ifelse follow-wall? [
;;;;;;;;;;;; changing direction when agent gets stuck at the end of a corridor
      ifelse wall? 0 1 = patch-here and wall? (90 * direction) 1 != nobody [
        set direction -1 * direction
        set heading heading + 180
      ]
;;;;;;;;;;;;
      [
        while [wall? 0 wspeed != nobody] [lt direction * 90]
      ]
    ]
    [
      set follow-wall? true
      set H patch-here
      set d-min distance target
      set int-target wall? 0 wspeed
      set n 1 + wspeed
      while [[walkable] of int-target = 0] [
        set int-target patch-at-heading-and-distance heading n
        set n n + 1
      ]
      set br-dist distance2v int-target target
;;;;;;;;;;;; extension 1
      ifelse heading mod 90 = 0 [lt 90 * direction] [
        let horz false
        let vert false
        if wall-p? patch-here 0 1 != nobody or wall-p? patch-here 180 1 != nobody [set horz true]
        if wall-p? patch-here 90 1 != nobody or wall-p? patch-here 270 1 != nobody [set vert true]
        if horz and not vert [
          ifelse heading > 0 and heading < 180 [
            ifelse heading < 90 [set direction -1] [set direction 1]
            set heading 90
          ]
          [
            ifelse heading < 270 [set direction -1] [set direction 1]
            set heading 270
          ]
        ]
        if not horz and vert [
          ifelse heading > 90 and heading < 270 [
            ifelse heading < 180 [set direction -1] [set direction 1]
            set heading 180
          ]
          [
            ifelse heading < 90 [set direction 1] [set direction -1]
            set heading 0
          ]
        ]

        if (horz and vert) or (not horz and not vert) [
          while [wall? 0 wspeed != nobody] [lt direction * 44.9]
          ifelse (round (heading / 90) = 0) or (round (heading / 90) = 4) [set heading 0] [
            ifelse round (heading / 90) = 1 [set heading 90] [
              ifelse round (heading / 90) = 2 [set heading 180] [set heading 270]
            ]
          ]
        ]
      ]
      while [wall? 0 wspeed != nobody] [lt direction * 90]
;;;;;;;;;;;;
    ]
  ]

  ifelse distance target <= 1 [move-to target die] [
    let people_ahead (turtles-on patch-ahead wspeed) with [self != myself and breed = people]
    ifelse (human-interaction? = false) or (count people_ahead < max-pop) or abs(abs(heading - [heading] of (one-of people_ahead)) - 180) < 90 [
      walk-watch-cars
      if follow-wall? [
        if distance target < d-min [set d-min distance target]
        if target-visible? target or distance target - r < d-min - step or patch-here = int-target [
          set follow-wall? false
          face target
          set d-min 0
          set H nobody
        ]
;        if patch-here = H and patch-right-and-ahead 180 wspeed != H [
;          ifelse cn = 1 or Extension-2 = false [
;            ask patch 10 0 [set plabel "Agent is trapped!" set plabel-color yellow]
;            ;output-type "Agent is trapped on: " output-print patch-here
;            set breed trappeds
;            set shape "face sad"
;            set size 2
;          ]
;          [set cn 1]
;        ]
      ]
    ]
    [
      set tol-count tol-count + 1
      if tol-count > tolerance [
        ;set tol-count 0
        let alter one-of neighbors with [walkable = 1 and count people-here < max-pop and count cars-here < 1]
        if alter != nobody [
          set tol-count 0
          move-to alter
          if follow-wall? [
            set follow-wall? false
            set d-min 0
            set H nobody
          ]
          face target
        ]
      ]
    ]
  ]
end

to create-mline
  set m-line []
  let k 0
  while [patch-right-and-ahead 0 k != target] [
    set m-line lput (patch-right-and-ahead 0 k) m-line
    set k k + 1
  ]
end

to build-virtual-obstacles [d]
  set vir-obs []
  let b 0
  while [b < 360] [
    if [patch-at-heading-and-distance b (d + 1)] of target != nobody and [[walkable] of patch-at-heading-and-distance b (d + 1)] of target = 1 [set vir-obs (lput ([patch-at-heading-and-distance b (d + 1)] of target) vir-obs)]
    set b b + 180 / pi / d
  ]
end

to interaction-walk
  let people_ahead (turtles-on patch-ahead wspeed) with [self != myself and breed = people]
  ifelse count people_ahead < max-pop or abs(abs(heading - [heading] of (one-of people_ahead)) - 180) <= 90 [
    walk-watch-cars
  ]
  [
    set tol-count tol-count + 1
    if tol-count > tolerance [
      set tol-count 0
      let alter one-of neighbors with [walkable = 1 and count people-here < max-pop]
      if alter != nobody [
        move-to alter
        if follow-wall? [
          set follow-wall? false
          set h nobody
        ]
        face target
      ]
    ]
  ]
end

to walk-watch-cars3
  ifelse human-interaction? [
    let nxt patch-ahead wspeed
    ifelse not member? nxt roads or [suggested] of nxt = 1 [fd wspeed] [
      lt direction * 90
      set nxt patch-ahead wspeed
      ifelse [walkable] of nxt = 1 and count people-on nxt < max-pop and count cars-on nxt < 1 [
        fd wspeed
        face target
        if follow-wall? [
          set follow-wall? false
          set d-min 0
          set H nobody
        ]
      ]
      [
        rt direction * 90
      ]
    ]
  ]
  [
    fd wspeed
  ]
end

to walk-watch-cars2
  ifelse human-interaction? [
    if member? patch-here roads with [suggested = 0] [
      let alt one-of neighbors with [walkable = 1 and count people-here < max-pop and count cars-here < 1]
      if alt != nobody [
        move-to alt
        face target
        if follow-wall? [
          set follow-wall? false
          set d-min 0
          set H nobody
        ]
      ]
    ]
  ]
  [
    fd wspeed
  ]
end

to walk-watch-cars
  ifelse human-interaction? [
    ifelse ([walkable] of (patch-ahead wspeed) = 1) [
      fd wspeed
    ]
    [
      ifelse [traffic_light] of patch-ahead wspeed = 1 [
        ;if abs(heading - green_direction) < 10 or abs(heading - green_direction - 180) < 10 [fd wspeed]
        let intsec one-of patches in-radius 3 with [member? self intersections]
        ifelse green_direction = 0 [
          if [pycor] of (patch-ahead wspeed) = [pycor] of intsec [fd wspeed]
        ]
        [
          if [pycor] of (patch-ahead wspeed) != [pycor] of intsec [fd wspeed]
        ]
      ]
      [
        ;let max-speed-car (max [speed_limit] of roads)
        ;if count cars-on (roads with [id = [id] of [patch-ahead wspeed] of myself]) in-radius max-speed-car = 0 [fd wspeed]
        fd wspeed
      ]
    ]
  ]
  [
    fd wspeed
  ]
end

to motion-toward-target
  if patch-here = target [die]
  ifelse target-visible? target or find-wall (towards target) r = nobody [
    face target
    ifelse human-interaction? [interaction-walk] [fd wspeed]
    ;ifelse wall? 0 wspeed != nobody [move-to one-of neighbors with [walkable = 1]] [fd wspeed]
  ]
  [
    LTG2
    set node (min-one-of (patches with [member? self [ltg-list] of myself]) [distance myself + distance [target] of myself])
    face node
    ifelse distance2v node target >= distance target [
      set H node
      set d-min distance2v (find-wall (towards target) r) target
      ifelse ([pxcor] of node < [pxcor] of (find-wall (towards target) r)) or ([pycor] of node > [pycor] of (find-wall (towards target) r)) [set direction 1] [set direction -1]
      set transition1? true
      ]
    [
      ifelse wall? 0 wspeed != nobody [move-to min-one-of neighbors with [walkable = 1] [distance [target] of myself]] [ifelse human-interaction? [interaction-walk] [fd wspeed]]
    ]
  ]
end

to follow-boundary
  if patch-here = target [die]
  LTG2

  let c1 wall-p? patch-here (heading + 90 * direction) 1
  let c2 wall-p? patch-here (heading + 135 * direction) 1
  if c1 = nobody and c2 != nobody [rt direction * 90]

  ifelse wall? 0 wspeed != nobody [
;;;;;;;;;;;; changing direction when agent gets stuck at the end of a corridor
      ifelse wall? 0 1 = patch-here and wall? (90 * direction) 1 != nobody [
        set direction -1 * direction
        set heading heading + 180
      ]
;;;;;;;;;;;;
      [
        while [wall? 0 wspeed != nobody] [lt direction * 90]
      ]
  ]
  [
    ifelse distance target <= 1 [move-to target die] [
      let people_ahead (turtles-on patch-ahead wspeed) with [self != myself and breed = people]
      ifelse (human-interaction? = false) or (count people_ahead < max-pop) or abs(abs(heading - [heading] of (one-of people_ahead)) - 180) <= 90 [
        walk-watch-cars
        if distance target < d-min [set d-min distance target]
        if patch-here = H and patch-right-and-ahead 180 wspeed != H [
          ask patch 10 0 [set plabel "Agent is trapped!" set plabel-color yellow]
          output-type "Agent is trapped on: " output-print patch-here
          set breed trappeds
          set shape "face sad"
          set size 2
        ]

        set node (min-one-of (patches with [member? self [ltg-list] of myself]) [distance [target] of myself])
        if distance2v node target < d-min [
          set follow-wall? false
          set transition2? true
          face node
        ]
      ]
      [
        if human-interaction? [
          set tol-count tol-count + 1
          if tol-count > tolerance [
            set tol-count 0
            let alter one-of neighbors with [walkable = 1 and count people-here < max-pop]
            if alter != nobody [
              move-to alter
              if follow-wall? [
                set follow-wall? false
              ]
              face target
            ]
          ]
        ]
      ]
    ]
  ]

  if target-visible? target or find-wall (towards target) r = nobody [
    set follow-wall? false
    set H nobody
    face target
  ]

end

to find-node
  set H find-wall (towards target) r
  let HH patch-at-heading-and-distance (towards target) (distance H - 1)
  let lst (patch-set HH ([neighbors] of HH) with [walkable = 1 and self != [patch-here] of myself])
  set lst lst with [self != [patch-here] of myself and [target-visible? myself] of myself and any? neighbors with [walkable = 0]]
  let lyr lst

  while [lyr != nobody] [
    let ph patch-here
    let a nobody
    ask lyr [
      set a (patch-set a neighbors with [self != ph and walkable = 1 and not member? self lst and any? neighbors with [walkable = 0]])
    ]
    if a != nobody [set a a with [[target-visible? myself] of myself]]
    set lst (patch-set lyr lst)
    set lyr a
  ]
  set edge lst


;;;; another approach to find the available edges, considers the entire obstacle,
;;;; could lead to errors for U shape obstacles with the target inside,
;;;; also fails to identify rather narrow passages
;  set H find-wall (towards target) r
;  set lst (patch-set H ([neighbors] of H) with [walkable = 0])
;  set lyr lst
;  while [lyr != nobody] [
;    let a nobody
;    ask lyr [
;      set a (patch-set a neighbors with [walkable = 0 and not member? self lst])
;    ]
;    set lst (patch-set lyr lst)
;    set lyr a
;  ]
;
;  set edge (patch-set ([neighbors] of lst)) with [self != [patch-here] of myself and walkable = 1 and [target-visible? myself] of myself]
;;;;;

  let head (towards target)
  let rs (edge with [(atan (pxcor - [pxcor] of myself) (pycor - [pycor] of myself) - head) mod 360 < 180])
  set rs sort rs
  set rs sort-by [distance ?1 < distance ?2] rs

  let a 0
  let node1 nobody
  foreach rs [
    let b (atan ([pxcor] of ? - pxcor) ([pycor] of ? - pycor) - head) mod 360
    ;let b ((towards ?) - head) mod 360
    ifelse b >= a [set node1 ? set a b] [
      if abs(b - a) < 10 and distance ? > distance node1 [set node1 ? set a b]      ;;;;;;;;;; for long inclined obstacles this line would improve the algorithm (**)
    ]
  ]

  let ls (edge with [(atan (pxcor - [pxcor] of myself) (pycor - [pycor] of myself) - head) mod 360 > 180])
  set ls sort ls
  set ls sort-by [distance ?1 < distance ?2] ls

  set a 360
  let node2 nobody
  foreach ls [
    let b (atan ([pxcor] of ? - pxcor) ([pycor] of ? - pycor) - head) mod 360
    ifelse b <= a [set node2 ? set a b] [
      if abs(b - a) < 10 and distance ? > distance node2 [set node2 ? set a b]      ;;;;;;;;;; (**)
    ]
  ]

  if node1 = nobody [set node node2 stop]
  if node2 = nobody [set node node1 stop]

  ifelse distance2v patch-here node1 < distance2v patch-here node2 [set node node1] [
    ifelse distance2v patch-here node1 = distance2v patch-here node2 [
      set node min-one-of (patch-set node1 node2) [distance2v self [target] of myself]
    ]
    [
      set node node2
    ]
  ]

  if node = last-node [
    ifelse node = node1 [set node node2] [set node node1]
  ]

end


;;;; boolean, report if the given point is visible to the agent
to-report target-visible? [pch]
  let dir towards pch
  let m 0
  let output false
  while [m <= r and [walkable] of (patch-at-heading-and-distance dir m) = 1] [
    ifelse patch-at-heading-and-distance dir m = pch [
      set output true
      set m r + 1     ;; to exit the loop (stop is not allowed in to-report)
    ]
    [set m m + wspeed]
  ]
  report output
end

;;; create Local Tangent Graph
to LTG2
  set ltg-list []
  let dir towards target
  if find-wall dir r = nobody [set ltg-list lput (patch-at-heading-and-distance dir r) ltg-list]
  let vis (patches in-radius r) with [self != [patch-here] of myself]
  set vis sort vis
  foreach vis [
    if target-visible? ? = false [set vis remove ? vis]
  ]
  let edge1 []
  foreach vis [
    if any? ([neighbors] of ?) with [walkable = 0] [set edge1 lput ? edge1]
  ]
  foreach edge1 [
    if count ([neighbors4] of ?) with [member? self edge1] = 1 [set ltg-list lput ? ltg-list]
  ]
  foreach ltg-list [
    if distance2v ? patch-here <= 1 [set ltg-list remove ? ltg-list]
  ]
end

;;;; report the farthest patch visible in a specific direction on the boundry of an obstacle
;;;; if no obctacle, report nobody
to-report find-wall [dir dist]
  let m 1
  ;let output patch-here
  while [m <= dist] [
    ifelse (patch-at-heading-and-distance dir m = nobody) or [walkable] of (patch-at-heading-and-distance dir m) = 1 [
      ;set output (patch-at-heading-and-distance dir m)
      set m m + wspeed
    ]
    [
      report (patch-at-heading-and-distance dir m)
      set m dist + 10
    ]
    if abs(m - dist) < 2 [report nobody]
  ]
end

to-report wall? [angle dt]
  ;; note that angle may be positive or negative. If angle is positive, the turtle looks right. If angle is negative, the turtle looks left.

  let wall nobody
  ifelse patch-right-and-ahead angle dt != nobody [
    if [walkable] of patch-right-and-ahead angle dt = 0 [set wall patch-right-and-ahead angle dt]
  ]
  [
    set wall patch-here
  ]
  report wall
end

to-report wall-p? [ph angle dt]
;; note that angle may be positive or negative. If angle is positive, the turtle looks right. If angle is negative, the turtle looks left.

  let wall nobody
  ask ph [
    ifelse patch-at-heading-and-distance angle dt != nobody [
      if [walkable] of patch-at-heading-and-distance angle dt = 0 [set wall patch-at-heading-and-distance angle dt]
    ]
    [
      set wall nobody
    ]
  ]
  report wall
end

to-report random-weibull [scale-p shape-p]
  let F random-float 1
  let y (1 / shape-p)
  let x scale-p * (-1 * ln (1 - F)) ^ y
  report x
end


to-report distance2v [agent1 agent2]
  let x1 0 let x2 0 let y1 0 let y2 0
  ifelse is-turtle? agent1 [set x1 [xcor] of agent1 set y1 [ycor] of agent1][set x1 [pxcor] of agent1 set y1 [pycor] of agent1]
  ifelse is-turtle? agent2 [set x2 [xcor] of agent2 set y2 [ycor] of agent2][set x2 [pxcor] of agent2 set y2 [pycor] of agent2]
  report sqrt ((x1 - x2) ^ 2 + (y1 - y2) ^ 2)
end

to-report distance2v-max [agent1 agent2]
  let x1 0 let x2 0 let y1 0 let y2 0 let xm 0 let ym 0
  ifelse is-turtle? agent1 [set x1 [xcor] of agent1 set y1 [ycor] of agent1][set x1 [pxcor] of agent1 set y1 [pycor] of agent1]
  ifelse is-turtle? agent2 [set x2 [xcor] of agent2 set y2 [ycor] of agent2][set x2 [pxcor] of agent2 set y2 [pycor] of agent2]
  set xm abs (x1 - x2) set ym abs (y1 - y2)
  ifelse xm < ym [report ym] [report xm]
end
@#$#@#$#@
GRAPHICS-WINDOW
975
13
2066
503
540
229
0.36985233604538176
1
10
1
1
1
0
0
0
1
-540
540
-229
229
1
1
1
ticks
10.0

BUTTON
853
286
920
319
GO!
go
T
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

MONITOR
804
594
920
639
count people left
count people
17
1
11

BUTTON
852
248
920
281
walk
go
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

TEXTBOX
326
340
665
538
Instructions:\n1) Set mapping scale & load map\n2) Modify the geometry (if needed) \n3) Setup Network\n4) Setup Exit Points\n5) Set car ownership, Traffic light period, walking speed mean and std\n6) choose walking algorithm\n7) Setup Population\n8) Set human-interaction and tolerance time\n9) Set model scale\n10) GO!
12
0.0
1

MONITOR
682
593
800
638
count trapped people
count trappeds
17
1
11

BUTTON
15
86
137
121
simplify obstacles
let a 0\nwhile [a < 40] [\n  ask patches with [pcolor = brown] [\n    if pxcor = max-pxcor or pycor = max-pycor or pxcor = min-pxcor or pycor = min-pycor [set pcolor black]\n    if count neighbors4 with [pcolor = brown] = 0 [set pcolor black]\n  ]\n\n  ask patches with [pcolor = red] [if count neighbors4 with [pcolor = red] = 0 [set pcolor black]]\n\n  ask patches with [pcolor = black] [\n    if count neighbors4 with [pcolor = brown] = 4 [set pcolor brown]\n    if count neighbors4 with [pcolor = red] = 4 [set pcolor red]\n    if count neighbors4 with [pcolor = brown or pcolor = red] = 4 [set pcolor brown]\n    \n    if count neighbors4 with [pcolor = brown ] = 3 and count neighbors with [pcolor = brown] >= 5 [set pcolor brown]\n    if count neighbors4 with [pcolor = red] = 3 and count neighbors with [pcolor = red] >= 5 [set pcolor red]\n    if count neighbors4 with [pcolor = brown or pcolor = red] = 3 and count neighbors with [pcolor = brown or pcolor = red] >= 5 [set pcolor brown]\n  ]\n  set a a + 1\n]
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

INPUTBOX
13
10
91
70
m_scale
50000
1
0
Number

BUTTON
95
37
180
70
Load Map
load-gis-map
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

INPUTBOX
705
340
755
400
scale
2.14
1
0
Number

TEXTBOX
100
11
321
29
mapping scale (the larger, the more details)
11
0.0
1

TEXTBOX
762
344
912
362
1 patch = scale [unit distance]
11
0.0
1

BUTTON
23
422
160
455
Setup Population
setup-people-from-gis\ncreate-cars-on-streets\ncreate-people-on-streets\nif movie? [\n  movie-start user-new-file\n  movie-set-frame-rate 10\n]
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

PLOT
360
11
816
320
Evacuated people
step
pop
0.0
100.0
0.0
100.0
true
false
"clear-plot" ""
PENS
"default" 1.0 0 -16777216 true "" "plot (gen-pop - count people - count drivers)"

PLOT
241
591
441
741
Generated population
step
pop
0.0
10.0
0.0
10.0
true
false
"clear-plot" ""
PENS
"default" 1.0 0 -16777216 true "" "plot (gen-pop)"

MONITOR
788
372
906
417
Evacuation time [min]
round (ticks * scale / max-speed-ped / 60)
17
1
11

INPUTBOX
85
291
150
351
std-speed
0.2
1
0
Number

INPUTBOX
16
215
100
275
car_ownership
0
1
0
Number

BUTTON
830
14
954
47
show intersections IDs
foreach nodes_roads [\nask item 1 ? [set plabel (item 0 ?) set pcolor yellow]\n]
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
831
57
920
90
drive
move-cars\nif count cars = 0 [stop]\ntick
T
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

MONITOR
924
594
995
639
count cars
count cars
17
1
11

BUTTON
831
99
926
132
test drivers
ask people [die]\nask cars[die]\nask n-of 10 exit-doors with [pop != 0] [sprout-people 2 [setup-people]]\nask people [set car? true setup-cars]
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
833
140
930
173
roll back car
ask cars [\n  ;move-to item 0 pre\n  \n  let elm item (read-from-string (remove \"r\" ([id] of patch-here))) roads_connect\n  ifelse [direct] of patch-here != \"bi\" [\n    set heading [direct] of patch-here\n    set target item (position (item 2 elm) nodes_list + 1) nodes_list\n  ]\n  [\n    let n1 item (position (item 1 elm) nodes_list + 1) nodes_list\n    let n2 item (position (item 2 elm) nodes_list + 1) nodes_list\n    set target min-one-of (patch-set n1 n2) [distance myself]\n    face target\n  ]\n]\n
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
14
128
138
161
Setup Network
setup-road-directions\nsetup-nodes\nfind-shortest-paths\n
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

TEXTBOX
107
248
132
266
[%]
11
0.0
1

TEXTBOX
160
319
258
337
walking speed [m/s]
11
0.0
1

INPUTBOX
18
291
80
351
mu-speed
1.5
1
0
Number

INPUTBOX
155
216
222
276
TL_period
30
1
0
Number

TEXTBOX
229
254
350
272
Traffic light period (sec)
11
0.0
1

CHOOSER
22
368
160
413
algorithm
algorithm
"Bug1" "Bug2" "VisBug" "DistBug" "TangentBug" "K-Bug" "New Algorithm" "New Algorithm2" "COPE0"
8

SWITCH
172
385
293
418
Extension-1
Extension-1
1
1
-1000

SWITCH
172
420
293
453
Extension-2
Extension-2
1
1
-1000

SWITCH
173
457
294
490
Extension-3
Extension-3
1
1
-1000

TEXTBOX
177
365
281
383
DistBug's extensions
11
0.0
1

SWITCH
23
464
168
497
human-interaction?
human-interaction?
0
1
-1000

INPUTBOX
24
508
114
568
tolerance_time
0
1
0
Number

TEXTBOX
125
541
152
559
[sec]
11
0.0
1

PLOT
27
590
227
740
cars
NIL
NIL
0.0
10.0
0.0
10.0
true
false
"clear-plot" ""
PENS
"default" 1.0 0 -16777216 true "" "plot (count cars)"

BUTTON
851
202
945
235
walk on streets
ask patches [set traffic_light 0]\nask roads [set walkable 1]\n;ask patches with [pcolor = brown] [set walkable 0]\nask patches with [pxcor = max-pxcor or pycor = max-pycor or pxcor = min-pxcor or pycor = min-pycor] [set walkable 0]\nreset-timer
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

PLOT
1153
547
1353
697
e1
NIL
NIL
0.0
10.0
0.0
10.0
true
false
"clear-plot" ""
PENS
"default" 1.0 0 -16777216 true "" "plot e1"

PLOT
1394
548
1594
698
e2
NIL
NIL
0.0
10.0
0.0
10.0
true
false
"clear-plot" ""
PENS
"default" 1.0 0 -16777216 true "" "plot e2"

INPUTBOX
639
424
970
587
notes
1 patch = 1.47m for mapping scale 75000;\n1 patch = 2.14m for mapping scale 50000; \nincreasing m_scale will decrease the scale linearly; \nspace allocated to each pedestrian = 0.5 * 0.5 m;
1
1
String

SWITCH
171
524
305
557
walk-on-streets?
walk-on-streets?
0
1
-1000

SWITCH
158
128
261
161
movie?
movie?
1
1
-1000

@#$#@#$#@
## WHAT IS IT?

The turtles in this example follow walls made out of colored patches.  Some turtles try to keep the wall on their right; others keep the wall on their left.

## HOW IT WORKS

Consider a turtle that wants to keep a wall on its right.  If there is no wall immediately to its right, but there is a wall behind it on the right, it must turn right in order not to lose the wall.  If there's a wall directly in front of the turtle, it keeps turning left until there's free space in front of it.  Then it can move forward.

## THINGS TO NOTICE

The turtles always stay on patch centers and never move diagonally.

If there isn't a wall next to a turtle when it is born, it just moves forward until it finds one.

## EXTENDING THE MODEL

The `walk` procedure will get stuck in an infinite loop if a turtle finds itself surrounded by walls on all four sides.  The `setup` procedure detects such turtles and kills them off, so the model won't get stuck.  How would you change `walk` not to have this problem?  (You might need to do so in a model where the walls could move and grow.)

## NETLOGO FEATURES

Turtles use the `patch-right-and-ahead` primitive to look at patches around themselves, relative to the direction the turtle is facing.
@#$#@#$#@
default
true
0
Polygon -7500403 true true 150 5 40 250 150 205 260 250

airplane
true
0
Polygon -7500403 true true 150 0 135 15 120 60 120 105 15 165 15 195 120 180 135 240 105 270 120 285 150 270 180 285 210 270 165 240 180 180 285 195 285 165 180 105 180 60 165 15

arrow
true
0
Polygon -7500403 true true 150 0 0 150 105 150 105 293 195 293 195 150 300 150

box
false
0
Polygon -7500403 true true 150 285 285 225 285 75 150 135
Polygon -7500403 true true 150 135 15 75 150 15 285 75
Polygon -7500403 true true 15 75 15 225 150 285 150 135
Line -16777216 false 150 285 150 135
Line -16777216 false 150 135 15 75
Line -16777216 false 150 135 285 75

bug
true
0
Circle -7500403 true true 96 182 108
Circle -7500403 true true 110 127 80
Circle -7500403 true true 110 75 80
Line -7500403 true 150 100 80 30
Line -7500403 true 150 100 220 30

butterfly
true
0
Polygon -7500403 true true 150 165 209 199 225 225 225 255 195 270 165 255 150 240
Polygon -7500403 true true 150 165 89 198 75 225 75 255 105 270 135 255 150 240
Polygon -7500403 true true 139 148 100 105 55 90 25 90 10 105 10 135 25 180 40 195 85 194 139 163
Polygon -7500403 true true 162 150 200 105 245 90 275 90 290 105 290 135 275 180 260 195 215 195 162 165
Polygon -16777216 true false 150 255 135 225 120 150 135 120 150 105 165 120 180 150 165 225
Circle -16777216 true false 135 90 30
Line -16777216 false 150 105 195 60
Line -16777216 false 150 105 105 60

car
false
0
Polygon -7500403 true true 300 180 279 164 261 144 240 135 226 132 213 106 203 84 185 63 159 50 135 50 75 60 0 150 0 165 0 225 300 225 300 180
Circle -16777216 true false 180 180 90
Circle -16777216 true false 30 180 90
Polygon -16777216 true false 162 80 132 78 134 135 209 135 194 105 189 96 180 89
Circle -7500403 true true 47 195 58
Circle -7500403 true true 195 195 58

circle
false
0
Circle -7500403 true true 0 0 300

circle 2
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240

cow
false
0
Polygon -7500403 true true 200 193 197 249 179 249 177 196 166 187 140 189 93 191 78 179 72 211 49 209 48 181 37 149 25 120 25 89 45 72 103 84 179 75 198 76 252 64 272 81 293 103 285 121 255 121 242 118 224 167
Polygon -7500403 true true 73 210 86 251 62 249 48 208
Polygon -7500403 true true 25 114 16 195 9 204 23 213 25 200 39 123

cylinder
false
0
Circle -7500403 true true 0 0 300

dot
false
0
Circle -7500403 true true 90 90 120

face happy
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 255 90 239 62 213 47 191 67 179 90 203 109 218 150 225 192 218 210 203 227 181 251 194 236 217 212 240

face neutral
false
0
Circle -7500403 true true 8 7 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Rectangle -16777216 true false 60 195 240 225

face sad
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 168 90 184 62 210 47 232 67 244 90 220 109 205 150 198 192 205 210 220 227 242 251 229 236 206 212 183

fish
false
0
Polygon -1 true false 44 131 21 87 15 86 0 120 15 150 0 180 13 214 20 212 45 166
Polygon -1 true false 135 195 119 235 95 218 76 210 46 204 60 165
Polygon -1 true false 75 45 83 77 71 103 86 114 166 78 135 60
Polygon -7500403 true true 30 136 151 77 226 81 280 119 292 146 292 160 287 170 270 195 195 210 151 212 30 166
Circle -16777216 true false 215 106 30

flag
false
0
Rectangle -7500403 true true 60 15 75 300
Polygon -7500403 true true 90 150 270 90 90 30
Line -7500403 true 75 135 90 135
Line -7500403 true 75 45 90 45

flower
false
0
Polygon -10899396 true false 135 120 165 165 180 210 180 240 150 300 165 300 195 240 195 195 165 135
Circle -7500403 true true 85 132 38
Circle -7500403 true true 130 147 38
Circle -7500403 true true 192 85 38
Circle -7500403 true true 85 40 38
Circle -7500403 true true 177 40 38
Circle -7500403 true true 177 132 38
Circle -7500403 true true 70 85 38
Circle -7500403 true true 130 25 38
Circle -7500403 true true 96 51 108
Circle -16777216 true false 113 68 74
Polygon -10899396 true false 189 233 219 188 249 173 279 188 234 218
Polygon -10899396 true false 180 255 150 210 105 210 75 240 135 240

house
false
0
Rectangle -7500403 true true 45 120 255 285
Rectangle -16777216 true false 120 210 180 285
Polygon -7500403 true true 15 120 150 15 285 120
Line -16777216 false 30 120 270 120

leaf
false
0
Polygon -7500403 true true 150 210 135 195 120 210 60 210 30 195 60 180 60 165 15 135 30 120 15 105 40 104 45 90 60 90 90 105 105 120 120 120 105 60 120 60 135 30 150 15 165 30 180 60 195 60 180 120 195 120 210 105 240 90 255 90 263 104 285 105 270 120 285 135 240 165 240 180 270 195 240 210 180 210 165 195
Polygon -7500403 true true 135 195 135 240 120 255 105 255 105 285 135 285 165 240 165 195

line
true
0
Line -7500403 true 150 0 150 300

line half
true
0
Line -7500403 true 150 0 150 150

pentagon
false
0
Polygon -7500403 true true 150 15 15 120 60 285 240 285 285 120

person
false
0
Circle -7500403 true true 110 5 80
Polygon -7500403 true true 105 90 120 195 90 285 105 300 135 300 150 225 165 300 195 300 210 285 180 195 195 90
Rectangle -7500403 true true 127 79 172 94
Polygon -7500403 true true 195 90 240 150 225 180 165 105
Polygon -7500403 true true 105 90 60 150 75 180 135 105

plant
false
0
Rectangle -7500403 true true 135 90 165 300
Polygon -7500403 true true 135 255 90 210 45 195 75 255 135 285
Polygon -7500403 true true 165 255 210 210 255 195 225 255 165 285
Polygon -7500403 true true 135 180 90 135 45 120 75 180 135 210
Polygon -7500403 true true 165 180 165 210 225 180 255 120 210 135
Polygon -7500403 true true 135 105 90 60 45 45 75 105 135 135
Polygon -7500403 true true 165 105 165 135 225 105 255 45 210 60
Polygon -7500403 true true 135 90 120 45 150 15 180 45 165 90

square
false
0
Rectangle -7500403 true true 30 30 270 270

square 2
false
0
Rectangle -7500403 true true 30 30 270 270
Rectangle -16777216 true false 60 60 240 240

star
false
0
Polygon -7500403 true true 151 1 185 108 298 108 207 175 242 282 151 216 59 282 94 175 3 108 116 108

target
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240
Circle -7500403 true true 60 60 180
Circle -16777216 true false 90 90 120
Circle -7500403 true true 120 120 60

tree
false
0
Circle -7500403 true true 118 3 94
Rectangle -6459832 true false 120 195 180 300
Circle -7500403 true true 65 21 108
Circle -7500403 true true 116 41 127
Circle -7500403 true true 45 90 120
Circle -7500403 true true 104 74 152

triangle
false
0
Polygon -7500403 true true 150 30 15 255 285 255

triangle 2
false
0
Polygon -7500403 true true 150 30 15 255 285 255
Polygon -16777216 true false 151 99 225 223 75 224

truck
false
0
Rectangle -7500403 true true 4 45 195 187
Polygon -7500403 true true 296 193 296 150 259 134 244 104 208 104 207 194
Rectangle -1 true false 195 60 195 105
Polygon -16777216 true false 238 112 252 141 219 141 218 112
Circle -16777216 true false 234 174 42
Rectangle -7500403 true true 181 185 214 194
Circle -16777216 true false 144 174 42
Circle -16777216 true false 24 174 42
Circle -7500403 false true 24 174 42
Circle -7500403 false true 144 174 42
Circle -7500403 false true 234 174 42

turtle
true
0
Polygon -10899396 true false 215 204 240 233 246 254 228 266 215 252 193 210
Polygon -10899396 true false 195 90 225 75 245 75 260 89 269 108 261 124 240 105 225 105 210 105
Polygon -10899396 true false 105 90 75 75 55 75 40 89 31 108 39 124 60 105 75 105 90 105
Polygon -10899396 true false 132 85 134 64 107 51 108 17 150 2 192 18 192 52 169 65 172 87
Polygon -10899396 true false 85 204 60 233 54 254 72 266 85 252 107 210
Polygon -7500403 true true 119 75 179 75 209 101 224 135 220 225 175 261 128 261 81 224 74 135 88 99

wheel
false
0
Circle -7500403 true true 3 3 294
Circle -16777216 true false 30 30 240
Line -7500403 true 150 285 150 15
Line -7500403 true 15 150 285 150
Circle -7500403 true true 120 120 60
Line -7500403 true 216 40 79 269
Line -7500403 true 40 84 269 221
Line -7500403 true 40 216 269 79
Line -7500403 true 84 40 221 269

x
false
0
Polygon -7500403 true true 270 75 225 30 30 225 75 270
Polygon -7500403 true true 30 75 75 30 270 225 225 270

@#$#@#$#@
NetLogo 5.3.1
@#$#@#$#@
random-seed 2
setup
repeat 50 [ go ]
ask turtles [ pen-down ]
repeat 150 [ go ]
@#$#@#$#@
@#$#@#$#@
<experiments>
  <experiment name="experiment" repetitions="12" runMetricsEveryStep="true">
    <setup>setup-people-from-gis
;create-cars-on-streets
create-people-on-streets</setup>
    <go>go</go>
    <timeLimit steps="5000"/>
    <metric>gen-pop - count people - count drivers - count trappeds</metric>
  </experiment>
</experiments>
@#$#@#$#@
@#$#@#$#@
default
0.0
-0.2 0 0.0 1.0
0.0 1 1.0 0.0
0.2 0 0.0 1.0
link direction
true
0
Line -7500403 true 150 150 90 180
Line -7500403 true 150 150 210 180

@#$#@#$#@
0
@#$#@#$#@
