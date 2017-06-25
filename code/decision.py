import numpy as np

# detect is clear
# return true when is clear
def is_clear(Rover):
    clear = (np.sum(Rover.vision_image[140:150,150:170,2]) >130) & (np.sum(Rover.vision_image[110:120,150:170,2]) >100)
    if not clear:
        print("is_clear check 1: ",np.sum(Rover.vision_image[140:150,150:170,2]))
        print("is_clear check 2: ",np.sum(Rover.vision_image[110:120,150:170,2]))
    return clear

# detect is struck or not,
# min: the min length of the Rover.xpos_stored
# period : the min period between two data
# return true when is stuck
def is_stuck(Rover,min, period):
    # print("is_stuck check::::",Rover.xpos_stored[period],Rover.pos[0],Rover.ypos_stored[period],Rover.pos[1])
    return (len(Rover.xpos_stored) >= min) \
           & (((Rover.pos[0] - 0.1) < Rover.xpos_stored[:period]).all()) & ((Rover.xpos_stored[:period] < (Rover.pos[0] + 0.1)).all()) \
           & (((Rover.pos[1] - 0.1) < Rover.ypos_stored[:period]).all()) & ((Rover.ypos_stored[:period] < (Rover.pos[1] + 0.1)).all())

# check nearby has rock or not
# min, default as 2
# return true when nearby has rock
def nearby_has_rock(Rover,min):
    samples_pos= [list(i) for i in zip(*Rover.samples_pos)]
    for temp in samples_pos:
        if ( (Rover.pos[0] - min) <= temp[0] <= (Rover.pos[0] + min) ) &( (Rover.pos[1] - min) <= temp[1] <= (Rover.pos[1] + min)) :
            return True
    return False

# change the pos to -3 after the rock has been found
def changeSamples_posToZero(Rover):
    rows = 6
    print("test 02****")
    for x in range(rows):
        print("test 01****")
        if ( Rover.pos[0]-2 <= Rover.samples_pos[0][x] <= Rover.pos[0]+2) & \
                ( Rover.pos[1]-2 <= Rover.samples_pos[1][x] <= Rover.pos[1]+2):
            Rover.samples_pos[0][x] = -3
            Rover.samples_pos[1][x] = -3
            print("Picked Item ", x)
            break

# Calculate the distance between rover and rock
def distance_to_rock(Rover):
    if len(Rover.rock_angles) >= 1:
        print("# Of Rock",len(Rover.rock_angles),"mean",np.mean(Rover.rock_dists))
        return np.mean(Rover.rock_dists)
    else:
        return 5000


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # store the postion of Rover
    Rover.xpos_stored = np.insert(Rover.xpos_stored,0,Rover.pos[0])
    Rover.ypos_stored = np.insert(Rover.ypos_stored,0,Rover.pos[1])

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            print("---------Forward ----------")
            print("is clear",is_clear(Rover))
            print("# of nav.angles",len(Rover.nav_angles),Rover.samples_found)
            if (len(Rover.rock_angles) >= 2) & (not Rover.picking_up) & (not Rover.send_pickup) :
                Rover.mode = 'stop'
            # nearby has rock and rock didn't see
            if (nearby_has_rock(Rover,3) & (len(Rover.rock_angles) <2)):
                Rover.vel = 0
                Rover.brake = Rover.brake_set
                Rover.yaw = 360
                print("yawing now")
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:
                print("case 1: good now")
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                # test the
                if ((Rover.total_time > 15) & (Rover.vel <= 0.2) &(len(Rover.rock_angles)<1)
                        & is_stuck(Rover,180,180)):
                    print("Stunk now")
                    Rover.throttle = -1
                    Rover.mode = 'stuck'
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif (len(Rover.nav_angles) < Rover.stop_forward) | (not is_clear(Rover)):
                print("case 2: change to stop mode")
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.stuck_counter = 0
                Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            print("---------stop ----------")
            # if rock show up

            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                print("Rover.rock_angles",len(Rover.rock_angles))
                if len(Rover.rock_angles) >= 1 :
                    print("going for rock")
                    Rover.mode = 'picking'
                else:
                    # Now we're stopped and we have vision data to see if there's a path forward
                    if len(Rover.nav_angles) < Rover.go_forward:
                        Rover.stuck_counter += 1
                        Rover.throttle = 0
                        # Release the brake to allow turning
                        Rover.brake = 0
                        # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                        if np.mean(Rover.nav_angles) > 0:
                            Rover.steer = -15 # Could be more clever here about which way to turn
                            print("Turn Right")
                            if Rover.stuck_counter > 300:
                                Rover.pitch = 360
                                print("Yawing")
                        else:
                            Rover.steer = 15
                            Rover.stuck_counter = 0
                            print("Turn Left")
                    elif len(Rover.nav_angles) < 300: #
                        print ("hit the mountain")
                        Rover.throttle = 0
                        Rover.pitch = 360
                    # If we're stopped but see sufficient navigable terrain in front then go!
                    if len(Rover.nav_angles) >= Rover.go_forward:
                        # Set throttle back to stored value
                        Rover.throttle = Rover.throttle_set
                        # Release the brake
                        Rover.brake = 0
                        # Set steer to mean angle
                        Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                        Rover.mode = 'forward'
        elif Rover.mode == 'stuck':
            print("---------STUCK-----------")
            print("stuck.counter",Rover.stuck_counter)
            Rover.stuck_counter += 1
            if len(Rover.nav_angles) >= Rover.go_forward \
                    & Rover.stuck_counter > 60 :
                Rover.throttle = Rover.throttle_set
                Rover.brake = 0
                Rover.stuck_counter = 0
                Rover.mode = 'forward'
            elif (Rover.stuck_counter > 30) & (np.mean(Rover.nav_angles* 180/np.pi) >0):
                Rover.vel = 0
                Rover.yaw = 360
            elif (Rover.stuck_counter > 30) & (np.mean(Rover.nav_angles* 180/np.pi) <=0):
                Rover.vel = 0
                Rover.yaw = -360
            elif np.mean(Rover.nav_angles) >= 0:
                print("STUCK, steer =-15")
                Rover.throttle = -1
                Rover.steer = -15
            elif np.mean(Rover.nav_angles) < 0:
                print("STUCK, steer =15")
                Rover.throttle = -1
                Rover.steer = 15
            else:
                print("STUCK, else now")
        elif Rover.mode == 'picking':
            print("-------Approaching Rock--------")
            print("near sample:",Rover.near_sample)
            print("len",len(Rover.rock_angles))
            print("Dis:",distance_to_rock(Rover))
            # case 1: slowly approach the rock
            print("test nearby_has_rock",nearby_has_rock(Rover,3))
            if distance_to_rock(Rover) <= 25: # give very close to the sample
                print("case 1: slowly approach the rock")
                Rover.brake = 0
                Rover.throttle = 0
                Rover.vel = 0.1
                print("Rover.steer:",0)
                Rover.steer = 0
                if Rover.near_sample:
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.vel = 0
                    print("sending out the picking signal")
            # case 2: sample show up
            elif len(Rover.rock_angles) >= 1: # as long as the sample show up,
                print("case 2: sample not close, but still show up")
                Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)
                print("Rover.steer:",np.mean(Rover.rock_angles * 180/np.pi))
                Rover.throttle = 0.25
                print("Rover.vel:",Rover.vel)
                print("Rover.throttle:",Rover.throttle)
            # case 3: Rock not show up and Rover not near sample and Rover.vel = 0.0
            elif (len(Rover.rock_angles) < 1) & (not Rover.near_sample) \
                    & (Rover.vel == 0.0)  :
                print("# case 3:  change to forward mode")
                Rover.mode = 'stop'
            # case 4: nearby has rock, but doesn't show up
            elif nearby_has_rock(Rover,3) & (len(Rover.rock_angles) <1):
                print("# case 4: nearby has rock, ")
                print(Rover.samples_pos)
                Rover.vel = 0
                Rover.yaw = 360
            else: # case 4: sample don't show up.don't know what to do"
                Rover.throttle = 0.15
                print("case 5: don't know what to do")
                print("close to rock")
                print("Rover.vel:",Rover.vel)
                print("Rover.throttle:",Rover.throttle)
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        print("---------Pick up-----------")
        Rover.send_pickup = True
        Rover.stuck_counter = -100
        print("Great Great !!!!!!!!!!!!!!!!", Rover.picking_up)
        if not Rover.picking_up:
            Rover.mode = 'forward'
            # Rover.samples_to_find -= 1
            Rover.worldmap[int(Rover.pos[1]), int(Rover.pos[0]), 1] += 1
            print("Samples_to_find",Rover.pos)
            changeSamples_posToZero(Rover)
            print("Rover.samples_pos",Rover.samples_pos)
    return Rover