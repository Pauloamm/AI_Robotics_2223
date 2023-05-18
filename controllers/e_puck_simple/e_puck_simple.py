import cv2
import numpy as np
from controller import Camera
from controller import Robot

#Initialize Robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

#Initialize motors
left_motor = robot.getDevice("left wheel motor")
left_motor.setPosition(float('inf'))
right_motor = robot.getDevice("right wheel motor")
right_motor.setPosition(float('inf'))

# Initialize camera
camera = robot.getDevice('camera')
camera.enable(timestep)

while robot.step(timestep) != -1:
    left_motor.setVelocity(2.0)
    right_motor.setVelocity(2.0)
    #R = 207 , G = 207 , B = 207 LINE RGB VALUES
    #H = 0 , S = 0 , V = 80 LINE HSV VALUES GRAY
   
    img = np.frombuffer(camera.getImage(), dtype=np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    cv2.imwrite('processed_frame.jpg', img)
    # Display the image
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    cv2.imwrite('processed_frame_gray.jpg', gray)
    _, thresholded = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    cv2.imwrite('processed_frame_threshold.jpg', thresholded)
    
    # Perform edge detection
    edges = cv2.Canny(thresholded, 50, 150)
    # Find contours in the edge image
    _, contours, _= cv2.findContours(edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) #find the contours
    cv2.drawContours(edges,contours,-1,(0,255,0),3)
    cv2.imwrite('processed_frame_detection.jpg', edges)
    
   

        
  
    
    
   
  

    
    
    