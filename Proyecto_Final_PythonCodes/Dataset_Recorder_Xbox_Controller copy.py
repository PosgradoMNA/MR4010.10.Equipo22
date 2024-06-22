"""camera_pid controller."""

from controller import Display, Keyboard, Robot, Camera
from vehicle import Car, Driver
import numpy as np
import cv2
from datetime import datetime
import os


#This inputs are to have a better steering angle with a joystick
from inputs import get_gamepad
import math
import threading

#Getting image from camera
def get_image(camera):
    raw_image = camera.getImage()  
    image = np.frombuffer(raw_image, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)
    )
    return image

#Image processing
def greyscale_cv2(image):
    gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return gray_img

#Display image 
def display_image(display, image):
    # Image to display
    image_rgb = np.dstack((image, image,image,))
    # Display image
    image_ref = display.imageNew(
        image_rgb.tobytes(),
        Display.RGB,
        width=image_rgb.shape[1],
        height=image_rgb.shape[0],
    )
    display.imagePaste(image_ref, 0, 0, False)


iter_treshold = 0  # Inicializar el contador de iteraciones sin líneas detectadas
turning_cycle_count = 0  # Inicializar el contador de ciclos de giro
threshold_turn_right = 150  # Establecer el umbral para los ciclos de giro hacia la derecha

# Función para calcular el ángulo de dirección del auto
def calculate_steering_angle(lines, image_width):
    
    lines_array = np.array([line[0] for line in lines])  # Convertir las líneas a un array numpy
    global iter_treshold  # Declarar que se utilizará la variable global iter_treshold
    global turning_cycle_count  # Declarar que se utilizará la variable global turning_cycle_count
    
    # Verificar si se detectaron líneas y si el array de líneas no está vacío
    if lines is not None and len(lines_array) > 0:
        iter_treshold = 0  # Reiniciar el contador de iteraciones sin líneas detectadas
        x_mid = np.mean(lines_array[:, 0])  # Calcular el punto medio de todas las líneas detectadas
        deviation = x_mid - (image_width / 2)  # Calcular la desviación del auto respecto al centro
        steering_angle = deviation / (image_width / 2) * 0.5  # Convertir la desviación en un ángulo de dirección
        # Verificar si se deben aplicar ciclos de giro hacia la derecha
        if turning_cycle_count > threshold_turn_right or turning_cycle_count == 0:
            turning_cycle_count = 0  # Reiniciar el contador de ciclos de giro
            return steering_angle  # Devolver el ángulo de dirección calculado
        else:
            turning_cycle_count += 1  # Incrementar el contador de ciclos de giro
            return 0.18  # Si se están acumulando ciclos de giro, mantener el ángulo actual
    else:
        iter_treshold += 1  # Incrementar el contador de iteraciones sin líneas detectadas
        # Verificar si se superó el umbral de iteraciones sin líneas
        if iter_treshold >= 3:
            turning_cycle_count += 1  # Incrementar el contador de ciclos de giro
            return 0.18  # Si no se detectan líneas, mantener el ángulo actual
        else:
            return 0  # Si no se supera el umbral de iteraciones sin líneas, no realizar ningún giro


#initial angle and speed 
#initial angle and speed 
manual_steering = 0
steering_angle = 0
angle = 0.0
speed=50
max_speed = 50
mid =speed*0.8
zero=0

# set target speed
def set_speed(kmh):
    global speed            #robot.step(50)
    speed = kmh
#update steering angle
def set_steering_angle(wheel_angle):
    global angle, steering_angle
    # Check limits of steering
    if (wheel_angle - steering_angle) > 0.1:
        wheel_angle = steering_angle + 0.1
    if (wheel_angle - steering_angle) < -0.1:
        wheel_angle = steering_angle - 0.1
    steering_angle = wheel_angle
  
    # limit range of the steering angle
    if wheel_angle > 0.5:
        wheel_angle = 0.5
    elif wheel_angle < -0.5:
        wheel_angle = -0.5
    # update steering angle
    angle = wheel_angle

#validate increment of steering angle
def change_steer_angle(inc):
    global manual_steering
    # Apply increment
    new_manual_steering = manual_steering + inc
    # Validate interval 
    if new_manual_steering <= 25.0 and new_manual_steering >= -25.0: 
        manual_steering = new_manual_steering
        set_steering_angle(manual_steering * 0.02)
    # Debugging
    if manual_steering == 0:
        print("going straight")
    else:
        turn = "left" if steering_angle < 0 else "right"
        print("turning {} rad {}".format(str(steering_angle),turn))



# main

def main():
    counter = 0
    # Create the Robot instance.
    robot = Car()
    driver = Driver()

    # Get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())

    # Create camera instance
    camera = robot.getDevice("camera")
    camera.enable(timestep)  # timestep

    # processing display
    display_img = Display("display")

    #create keyboard instance
    keyboard=Keyboard()
    keyboard.enable(timestep)

    #Lidar 
    lidar= robot.getDevice('lidar')
    lidar.enable(timestep)
    lidar.enablePointCloud()

    num_points = lidar.getNumberOfPoints()


    while robot.step() != -1:
        # Get image from camera
        image = get_image(camera)
        greyscale_image = greyscale_cv2(image)
        image_path =r"Training\\"+ f'{counter}.png'
        
        #cv2.imwrite(image_path, greyscale_image)

        display_image(display_img, greyscale_cv2(image))

        #update angle and speed
        steering_angle = joy.read()/4 #to be more precise
        if abs(steering_angle) < 0.01:
            steering_angle = 0
        print(steering_angle)

        """with open(r"steering_angles.csv", 'a') as f:
            f.write(f'{image_path},{steering_angle}\n')"""

        range_image = lidar.getRangeImage()
        #print('{}'.format(range_image))
       
        for i in range(num_points):
            distance = range_image[i]
            if distance < 10.0:
               angle = lidar.getHorizontalResolution() * i
               print(f"Objeto detectado a {distance:.2f} metros en el ángulo {angle:.2f} radianes")
               speed =mid
               print('Actual speed: ',mid)
            elif distance < 5.0:
                speed =zero
                print('Vehiculo detenido evitar colision')
            else:
                speed=max_speed
            
        driver.setSteeringAngle(steering_angle)
        driver.setCruisingSpeed(speed)
        counter += 1

class XboxController(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):

        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.UpDPad = 0
        self.DownDPad = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()


    def read(self): # return the buttons/triggers that you care about in this methode
        
        x = self.LeftJoystickX
        return x


    def _monitor_controller(self):
        self.LeftJoystickX = 0
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_X':
                    self.LeftJoystickX = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1

joy = XboxController()

if __name__ == "__main__":
    main()