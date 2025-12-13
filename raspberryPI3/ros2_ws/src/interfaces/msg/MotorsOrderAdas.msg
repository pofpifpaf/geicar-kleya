

int8 offset_right_rear_pwm 0    #PWM [0;100]
int8 offset_left_rear_pwm 0     #PWM [0;100]

int8 offset_steering_angle 0    #Angle [-128;127]


int8 command_right_rear_pwm 50
int8 command_left_rear_pwm 50

bool command_pwm False


int8 command_steering_angle 0

bool command_steering False


int8 max_pwm 0 					#PWM relatif [0;50]

bool emergency_stop False


string state

bool active False        		#if any information has been changed from previous motors_orders
