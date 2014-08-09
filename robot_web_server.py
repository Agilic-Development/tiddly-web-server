__author__ = 'alexgray'

import logging

LOG_FILENAME = "/tmp/robot_web_server_log.txt"
logging.basicConfig(filename=LOG_FILENAME, level=logging.DEBUG)

# Also log to stdout
consoleHandler = logging.StreamHandler()
consoleHandler.setLevel(logging.DEBUG)
logging.getLogger("").addHandler(consoleHandler)

import os
import os.path
import math
import time
import signal
import tornado.httpserver
import tornado.ioloop
import tornado.web
import tornado.escape
import sockjs.tornado
import threading
import Queue
import camera_streamer
import robot_controller
import robot_config
import json
import subprocess

robot = TiddlyAPI.TiddlyBot  # Put your robot API class here
robotConfig = robot_config.RobotConfig()

cameraStreamer = None
scriptPath = os.path.dirname(__file__)
webPath = os.path.abspath(scriptPath + "/www")
robotConnectionResultQueue = Queue.Queue()
isClosing = False


#---------------------------------------------------------------------------------------------------
def createRobot(robotConfig, resultQueue):

    r = robot_controller.RobotController(robotConfig)
    resultQueue.put(r)


#---------------------------------------------------------------------------------------------------
class ConnectionHandler(sockjs.tornado.SockJSConnection):

    #-----------------------------------------------------------------------------------------------
    def on_open(self, info):
        pass

    #-----------------------------------------------------------------------------------------------
    def on_message(self, message):

        try:
            message = str(message)
        except Exception:
            logging.warning("Got a message that couldn't be converted to a string")
            return

        if isinstance(message, str):

            line_data = message.split(" ")
            if len(line_data) > 0:

                if line_data[0] == "":
                    if robot is not None:
                        robot.centreNeck()

                elif line_data[0] == "StartStreaming":  # "StartStreaming"
                    cameraStreamer.startStreaming()

                elif line_data[0] == "Shutdown":  # "Shutdown"
                    subprocess.call(["poweroff"])

                elif line_data[0] == "LineFollower":  # "LineFollower"
                    if robot is not None:
                        robot.line_follower(True)

                elif line_data[0] == "SetMovementServos" and len(line_data) >= 3:  # "SetMovementServos 0.25 0.5"

                    left_motor_speed, right_motor_speed = self.extract_joystick_data(line_data[1], line_data[2])

                    if robot is not None:
                        robot.setMotorSpeeds(left_motor_speed, right_motor_speed)

                elif line_data[0] == "CameraAngle" and len(line_data) >= 3:  # "CameraAngle -0.21"

                    neck_joystick_x, neck_joystick_y = self.extract_joystick_data(line_data[1], line_data[2])

                    if robot is not None:
                        robot.set_camera_angle(neck_joystick_x, neck_joystick_y)

                elif line_data[0] == "SetLed" and len(line_data) >= 4:  # "SetLed 0 0 1"
                    led_one, led_two, led_three = self.extract_led_data(line_data[1], line_data[2], line_data[3])

                    if robot is not None:
                        robot.set_led(led_one, led_two, led_three)

                elif line_data[0] == "Code":
                    pass
                    ## Code from blockly to Blockly Decoder class

    #-----------------------------------------------------------------------------------------------
    def on_close(self):
        logging.info("SockJS connection closed")

    #-----------------------------------------------------------------------------------------------
    def getLogsDict(self):

        logs_dict = {}

        # Read in main logs file
        try:
            with open(LOG_FILENAME, "r") as logFile:
                logs_dict["MainLog"] = logFile.read()
        except Exception:
            pass

        # Read in Ino build output if it exists
        try:
            with open(ino_uploader.BUILD_OUTPUT_FILENAME, "r") as logFile:
                logs_dict["InoBuildLog"] = logFile.read()
        except Exception:
            pass

        return logs_dict

    #-----------------------------------------------------------------------------------------------
    def extract_joystick_data(self, data_x, data_y ):

        joystick_x = 0.0
        joystick_y = 0.0

        try:
            joystick_x = float(data_x)
        except Exception:
            pass

        try:
            joystick_y = float(data_y)
        except Exception:
            pass

        return joystick_x, joystick_y

    def extract_led_data(self, one, two, three):
        led_one, led_two, led_three = 0
        try:
            led_one = float(data_x)
        except Exception:
            pass

        try:
            led_two = float(data_y)
        except Exception:
            pass

        try:
            led_three = float(data_y)
        except Exception:
            pass

        return led_one, led_two, led_three


#---------------------------------------------------------------------------------------------------
class MainHandler(tornado.web.RequestHandler):

    #------------------------------------------------------------------------------------------------
    def get(self):
        self.render(webPath + "/index.html")

#---------------------------------------------------------------------------------------------------
def robotUpdate():

    global robot
    global isClosing

    if isClosing:
        tornado.ioloop.IOLoop.instance().stop()
        return

    if robot is None:

        if not robotConnectionResultQueue.empty():

            robot = robotConnectionResultQueue.get()

    else:

        robot.update()

#---------------------------------------------------------------------------------------------------
def signalHandler(signum, frame):

    if signum in [signal.SIGINT, signal.SIGTERM]:
        global isClosing
        isClosing = True


#---------------------------------------------------------------------------------------------------
if __name__ == "__main__":

    signal.signal(signal.SIGINT, signalHandler)
    signal.signal(signal.SIGTERM, signalHandler)

    # Create the configuration for the web server
    router = sockjs.tornado.SockJSRouter(
        ConnectionHandler, '/robot_control')
    application = tornado.web.Application(router.urls + [
        (r"/", MainHandler),
        (r"/(.*)", tornado.web.StaticFileHandler, {"path": webPath}),
        (r"/css/(.*)", tornado.web.StaticFileHandler, {"path": webPath + "/css"}),
        (r"/css/images/(.*)", tornado.web.StaticFileHandler, {"path": webPath + "/css/images"}),
        (r"/images/(.*)", tornado.web.StaticFileHandler, {"path": webPath + "/images"}),
        (r"/js/(.*)", tornado.web.StaticFileHandler, {"path": webPath + "/js"})])

    #( r"/(.*)", tornado.web.StaticFileHandler, {"path": scriptPath + "/www" } ) ] \

    # Create a camera streamer
    cameraStreamer = camera_streamer.CameraStreamer()

    # Make sure SPI module is loaded (shouldn't need to do this...)
    subprocess.call(["modprobe", "spi_bcm2708"])
    time.sleep(0.5)

    # Start connecting to the robot asyncronously
    robotConnectionThread = threading.Thread(target=createRobot,
        args=[robotConfig, robotConnectionResultQueue])
    robotConnectionThread.start()

    # Now start the web server
    logging.info("Starting web server...")
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(80)

    robotPeriodicCallback = tornado.ioloop.PeriodicCallback(
        robotUpdate, 100, io_loop=tornado.ioloop.IOLoop.instance())
    robotPeriodicCallback.start()

    cameraStreamerPeriodicCallback = tornado.ioloop.PeriodicCallback(
        cameraStreamer.update, 1000, io_loop=tornado.ioloop.IOLoop.instance())
    cameraStreamerPeriodicCallback.start()

    tornado.ioloop.IOLoop.instance().start()

    # Shut down code
    robotConnectionThread.join()

    if robot is not None:
        robot.disconnect()
    else:
        if not robotConnectionResultQueue.empty():
            robot = robotConnectionResultQueue.get()
            robot.disconnect()

    cameraStreamer.stopStreaming()
