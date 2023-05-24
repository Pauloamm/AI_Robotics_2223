import cv2
import numpy as np
from controller import Camera
from controller import Supervisor

from CellsGraph import CellsGraph
from PathFindingAStar import PathFindingAStar



def run_robot(robot):

    # Camera Image Variables
    image_width = 640
    image_height = 240


    timestep = int(robot.getBasicTimeStep())
    base_speed = 0.25
    max_speed = 6.28 # documentation vel

    # Initialize motors
    left_motor = robot.getDevice("left wheel motor")
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)

    right_motor = robot.getDevice("right wheel motor")
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)


    # Initialize camera
    camera = robot.getDevice('camera')
    camera.enable(timestep)

    #Initialize GPS
    gps = robot.getDevice('gps')
    gps.enable(timestep)

    while robot.step(timestep) != -1:

        #Read Sensors
        gpsValues = gps.getValues()
        gpsValuesText = " {0:0.5f} , {1:0.5f}, {2:0.5f}".format(gpsValues[0],gpsValues[1],gpsValues[2])
        print(gpsValuesText)


        #Process data

        #Actions

        frame = np.frombuffer(camera.getImage(), dtype=np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
        # save rgb
        # cv2.imwrite('processed_frame.jpg', frame)

        # Preprocessing
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        # cv2.imwrite('processed_gray.jpg', gray)

        _, binary = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
        # cv2.imwrite('processed_binary.jpg', binary)

        # Canny edge detection
        edges = cv2.Canny(binary, 50, 150)
        # Hough line transform
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

        if lines is not None:
            center_x_total = 0
            center_y_total = 0
            count = 0

            # Iterate over all detected lines
            for line in lines:
                x1, y1, x2, y2 = line[0]

                # Calculate the center of the line segment
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2

                center_x_total += center_x
                center_y_total += center_y
                count += 1

            if count > 0:
                # Calculate the average center point
                average_center_x = center_x_total // count
                average_center_y = center_y_total // count

                # Draw a circle at the average center point (for visualization)
            cv2.circle(frame, (average_center_x, average_center_y), 5, (0, 255, 0), -1)

        cv2.imwrite('CENTER.jpg', frame)

        # CONTROL MOTORS
        # Find Error
        center_x = average_center_x  # Assuming you have obtained the average center x-coordinate
        desired_position = image_width / 2  # Assuming the desired position is the center of the image width
        print(f"Center X: {center_x}")
        error = center_x - desired_position
        # Set the proportional control gain
        Ke = 0.5
        Kp = 2

        # Calculate the individual wheel speeds
        left_wheel_speed = Kp + ((Ke * error) / desired_position)
        right_wheel_speed = Kp - ((Ke * error) / desired_position)
        left_motor.setVelocity(left_wheel_speed)
        right_motor.setVelocity(right_wheel_speed)
        print(f"left wheel: {left_wheel_speed}")
        print(f"right wheel: {right_wheel_speed}")

   
if __name__=='__main__':
    # Initialize Robot
    robot = Supervisor()

    cellsGraph = CellsGraph(supervisor=robot)
    pathFinding = PathFindingAStar(cellsGraph)


    run_robot(robot)



