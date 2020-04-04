#ifndef MOVE_H_
#define MOVE_H_
#define PI 3.141592

#define WHEEL_DISTANCE  	5.6f	//cm
#define PERIMETER_EPUCK 	PI * WHEEL_DISTANCE
#define ANGLE(angle) (PERIMETER_EPUCK*angle)/360

void navigation_start(void);



#endif /* CAMREG_EDGEGARD_H_ */
