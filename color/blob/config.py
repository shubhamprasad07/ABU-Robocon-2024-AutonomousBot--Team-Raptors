#camera
CAMERA_PORT = '/dev/ttyUSB0'
WIDTH = 320
HEIGHT = 240

# Serial communication settings
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

# Color range for blob detection
BLUE_MIN = (47, 21, 66)
BLUE_MAX = (116, 240, 255)

# Blur value for blob detection
BLUR_VALUE = 0

#Bounding Box Dimentions
HEIGHT_CENTER = (HEIGHT/2)
WIDTH_CENTER  = (WIDTH/2)
BB_MIN_HEIGHT =  HEIGHT_CENTER * 0.7
BB_MAX_HEIGHT =  HEIGHT_CENTER * 0.3 + HEIGHT_CENTER
BB_MIN_WIDTH  =  WIDTH_CENTER  * 0.7
BB_MAX_WIDTH  =  WIDTH_CENTER  * 0.3 + WIDTH_CENTER
BB_COLOR = (0,0,0)


# Detection window for blob detection
DETECTION_WINDOW = [0.0, 0.0, 1.0, 1.0]

min_size = 10
max_size = 255
