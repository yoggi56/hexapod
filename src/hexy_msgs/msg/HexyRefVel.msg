#legs
float64[6] 	leg_x
float64[6] 	leg_y
float64[6] 	leg_z

#body
float64[3] 	body_transl
float64[3] 	body_rotate

#walking
float64[2] 	walk_dir
float64 	walk_rotate
uint8 		walk_type

#system
uint8 		command # 1 - go_idle, 2 - go_sleep, 3 - moving
