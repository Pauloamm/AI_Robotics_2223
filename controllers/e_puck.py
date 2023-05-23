from controller import Robot, DistanceSensor, Motor



def run_robot(robot):
    # time in [ms] of a simulation step
    TIME_STEP = 64
    
    MAX_SPEED = 6.28
    
    # create the Robot instance.
    robot = Robot()
    
  
    
    leftMotor = robot.getDevice('left wheel motor')
    rightMotor = robot.getDevice('right wheel motor')
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))
    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)
    
    # feedback loop: step simulation until receiving an exit event
    while robot.step(TIME_STEP) != -1:
        # read sensors outputs
        psValues = []
        for i in range(8):
            psValues.append(ps[i].getValue())
    
        # detect obstacles
        right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0
        left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0
    
        # initialize motor speeds at 50% of MAX_SPEED.
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
        # modify speeds according to obstacles
        if left_obstacle:
            # turn right
            leftSpeed  = 0.5 * MAX_SPEED
            rightSpeed = -0.5 * MAX_SPEED
        elif right_obstacle:
            # turn left
            leftSpeed  = -0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED
        # write actuators inputs
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
        
if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)

