import numpy as np

HOME_X = 99.7
HOME_Y = 85.6
NUMBER_OF_ROCKS = 6
MAX_STEER_ANGLE = 15
DEVIATION_BIAS = 0.2
OBSTACLE_BIAS = 0.2
STUCK_COUNTER = 0
BACKUP_COUNTER = 0
LAST_COLLECTED = 0
CHANGE_DIR = False
PERTURBATION_SIZE = 4

def has_all_rocks(Rover):
    return Rover.samples_collected >= NUMBER_OF_ROCKS

def get_random_angle(isNegative):

    steerAngle = np.random.random_sample()

    if isNegative:
        return -MAX_STEER_ANGLE + PERTURBATION_SIZE * steerAngle
    else:
        return MAX_STEER_ANGLE - PERTURBATION_SIZE * steerAngle

def is_stuck(Rover):

    global STUCK_COUNTER

    if STUCK_COUNTER > 0:
        return True

    if Rover.throttle > 0 and Rover.brake == 0 and Rover.mode == 'forward' and Rover.vel == 0:
        STUCK_COUNTER = 5
        return True

    return False

def determine_forward_direction(Rover, isWander = False):
    mean_navigable = np.mean(Rover.nav_angles * 180 / np.pi)

    if isWander and len(Rover.nav_angles) > 2000:
        steerAngle = np.random.random_sample()
        std_navigable = np.std(Rover.nav_angles * 180 / np.pi)

        if DEVIATION_BIAS < 0:
            mean_navigable = np.clip(mean_navigable + DEVIATION_BIAS * std_navigable, -MAX_STEER_ANGLE, MAX_STEER_ANGLE) + steerAngle * PERTURBATION_SIZE
        else:
            mean_navigable = np.clip(mean_navigable + DEVIATION_BIAS * std_navigable, -MAX_STEER_ANGLE, MAX_STEER_ANGLE) - steerAngle * PERTURBATION_SIZE

    Rover.steer = np.clip(mean_navigable, -MAX_STEER_ANGLE, MAX_STEER_ANGLE)

    return Rover

def set_rover_throttle(Rover, max_velocity):
    if Rover.vel < max_velocity:
        # Set throttle value to throttle setting
        Rover.throttle = Rover.throttle_set
    else:  # Else coast
        Rover.throttle = 0
    Rover.brake = 0

    return Rover


#got nothing else to do, might as well look around
def wander(Rover):
    print("WANDER")

    global CHANGE_DIR
    global DEVIATION_BIAS

    mean_obstacle_angle = np.mean(Rover.ob_angles)

    if not CHANGE_DIR and not Rover.mode == 'stop':
        if mean_obstacle_angle > 0 and DEVIATION_BIAS < 0:
            DEVIATION_BIAS *= -1

        if mean_obstacle_angle < 0 and DEVIATION_BIAS > 0:
            DEVIATION_BIAS *= -1

    if Rover.nav_angles is not None:
        # Check for Rover.mode status

        if Rover.mode == 'forward':

            if len(Rover.nav_angles) >= Rover.change_dir and not CHANGE_DIR:
                Rover = set_rover_throttle(Rover, Rover.max_vel)
                DEVIATION_BIAS *= -1
                CHANGE_DIR = True
                Rover = determine_forward_direction(Rover, True)

            # Check the extent of navigable terrain
            elif len(Rover.nav_angles) >= Rover.stop_forward:
                CHANGE_DIR = False
                Rover = set_rover_throttle(Rover, Rover.max_vel)
                Rover = determine_forward_direction(Rover, True)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                Rover = stop_rover(Rover)

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover = stop_rover(Rover)
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    Rover.brake = 0

                    min_angle = np.min(np.abs(Rover.ob_angles))

                    if min_angle > 0:
                        Rover.steer = get_random_angle(False)
                    else:
                        Rover.steer = get_random_angle(True)

                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    Rover = set_rover_throttle(Rover, Rover.max_vel)
                    Rover = determine_forward_direction(Rover)
                    Rover.mode = 'forward'

    return Rover

def stop_rover(Rover):
    print("STOP ROVER")

    if Rover.vel == 0:
        Rover.brake = 0
        return Rover

    Rover.throttle = 0
    Rover.brake = Rover.brake_set
    Rover.steer = 0
    Rover.mode = 'stop'

    return Rover

def collect(Rover):
    print("COLLECT")

    if Rover.vel > 1.0:
        return stop_rover(Rover)

    Rover.mode = 'forward'
    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -MAX_STEER_ANGLE, MAX_STEER_ANGLE)
    Rover = set_rover_throttle(Rover, Rover.max_collect_vel)

    return Rover


def grab(Rover):
    print("GRAB")

    Rover = stop_rover(Rover)

    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    return Rover

def go_home(Rover):
    print("GO HOME")

    x_pixel = HOME_X - Rover.pos[0]
    y_pixel = HOME_Y - Rover.pos[1]

    dist = np.sqrt(x_pixel ** 2 + y_pixel ** 2)

    if dist < 5:
        print ("HOME SWEET HOME!")
        Rover = stop_rover(Rover)
        return Rover

    # Calculate angle away from vertical for each pixel
    angle = np.arctan2(y_pixel, x_pixel) * 180 / np.pi
    mean_navigable = np.mean(Rover.nav_angles * 180 / np.pi)
    std_navigable = np.std(Rover.nav_angles * 180 / np.pi)

    if angle < 0:
        mean_navigable = mean_navigable - 0.2 * std_navigable
    else:
        mean_navigable = mean_navigable + 0.2 * std_navigable

    if dist < 10:
        Rover.throttle = 0.2
    else:
        # mean nav with bias towards home
        return wander(Rover)

    Rover.mode = 'forward'
    Rover.brake = 0

    if Rover.vel > Rover.max_vel:
        Rover.throttle = 0

    Rover.steer = np.clip(mean_navigable, -MAX_STEER_ANGLE, MAX_STEER_ANGLE)
    return Rover

def avoid(Rover):

    global STUCK_COUNTER

    print("AVOID")
    STUCK_COUNTER -= 1

    min_angle = np.min(Rover.ob_angles)

    if min_angle > 0:
        Rover.steer = get_random_angle(False)
    else:
        Rover.steer = get_random_angle(True)
    Rover.throttle = 0.0
    return Rover

def start_backup(Rover):
    global LAST_COLLECTED
    global BACKUP_COUNTER

    if Rover.samples_collected > LAST_COLLECTED:
        BACKUP_COUNTER = 10

    LAST_COLLECTED = Rover.samples_collected

    return BACKUP_COUNTER > 0

def backup_rover(Rover):
    global BACKUP_COUNTER

    BACKUP_COUNTER -= 1
    Rover.steer = 0
    Rover.throttle = -0.1
    return Rover



# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    print ("Num Angles: ", len(Rover.nav_angles))
    print ("OB Angles: ", len(Rover.ob_angles))
    print ("OB Mean: ", np.mean(Rover.ob_angles))

    #have to wait for the rover to finish the pickup task before doing anything else
    if Rover.picking_up:
        return Rover

    if Rover.near_sample:
        return grab(Rover)

    if start_backup(Rover):
        return backup_rover(Rover)

    if Rover.seeSample:
        return collect(Rover)

    if is_stuck(Rover):
        return avoid(Rover)

    if has_all_rocks(Rover):
        return go_home(Rover)

    return wander(Rover)
