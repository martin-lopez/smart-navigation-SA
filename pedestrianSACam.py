#pedestrianSACam - software for recording the head behavior of pedestrians
#this code relies on LSMD90.py and GPSController.py helper files

from GPSController import *
import RPi.GPIO as GPIO
import picamera
import time
import datetime
import threading
import os
import argparse

#relevant BerryIMU imports
import smbus
import math
from LSM9DS0 import *

# constants
VIDEOFPS = 18
VIDEOHEIGHT = 720
VIDEOWIDTH = 1280


if __name__ == "__main__":
    
    #Command line options
    parser = argparse.ArgumentParser(description="Pelmetcam")
    parser.add_argument("path", help="The location of the data directory")
    parser.add_argument("-d", "--dataoverlay", action="store_true", help="Output data overlay images at runtime")
    args = parser.parse_args()
    
    try:
        
        print "Starting pedestrian situational awareness cam"
        print "Data path - " + args.path
        print "Data overlay - " + str(args.dataoverlay)
        
        #set gpio mode
        GPIO.setmode(GPIO.BCM)
        
        #start gps controller
        gpscontrol = GpsController()
        gpscontrol.start()
        print "GPS - started controller"

        #setup BerryIMU data stream
        bus = smbus.SMBus(1)

        RAD_TO_DEG = 57.29578
        M_PI = math.pi
        G_GAIN = 0.00875  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
        LP = 0.041    	# Loop period = 41ms.   This needs to match the time it takes each loop to run
        AA =  0.80      # Complementary filter constant


        def writeACC(register,value):
                bus.write_byte_data(ACC_ADDRESS , register, value)
                return -1

        def writeMAG(register,value):
                bus.write_byte_data(MAG_ADDRESS, register, value)
                return -1

        def writeGRY(register,value):
                bus.write_byte_data(GYR_ADDRESS, register, value)
                return -1


        def readACCx():
                acc_l = bus.read_byte_data(ACC_ADDRESS, OUT_X_L_A)
                acc_h = bus.read_byte_data(ACC_ADDRESS, OUT_X_H_A)
                acc_combined = (acc_l | acc_h <<8)

                return acc_combined  if acc_combined < 32768 else acc_combined - 65536


        def readACCy():
                acc_l = bus.read_byte_data(ACC_ADDRESS, OUT_Y_L_A)
                acc_h = bus.read_byte_data(ACC_ADDRESS, OUT_Y_H_A)
                acc_combined = (acc_l | acc_h <<8)

                return acc_combined  if acc_combined < 32768 else acc_combined - 65536


        def readACCz():
                acc_l = bus.read_byte_data(ACC_ADDRESS, OUT_Z_L_A)
                acc_h = bus.read_byte_data(ACC_ADDRESS, OUT_Z_H_A)
                acc_combined = (acc_l | acc_h <<8)

                return acc_combined  if acc_combined < 32768 else acc_combined - 65536


        def readMAGx():
                mag_l = bus.read_byte_data(MAG_ADDRESS, OUT_X_L_M)
                mag_h = bus.read_byte_data(MAG_ADDRESS, OUT_X_H_M)
                mag_combined = (mag_l | mag_h <<8)

                return mag_combined  if mag_combined < 32768 else mag_combined - 65536


        def readMAGy():
                mag_l = bus.read_byte_data(MAG_ADDRESS, OUT_Y_L_M)
                mag_h = bus.read_byte_data(MAG_ADDRESS, OUT_Y_H_M)
                mag_combined = (mag_l | mag_h <<8)

                return mag_combined  if mag_combined < 32768 else mag_combined - 65536


        def readMAGz():
                mag_l = bus.read_byte_data(MAG_ADDRESS, OUT_Z_L_M)
                mag_h = bus.read_byte_data(MAG_ADDRESS, OUT_Z_H_M)
                mag_combined = (mag_l | mag_h <<8)

                return mag_combined  if mag_combined < 32768 else mag_combined - 65536



        def readGYRx():
                gyr_l = bus.read_byte_data(GYR_ADDRESS, OUT_X_L_G)
                gyr_h = bus.read_byte_data(GYR_ADDRESS, OUT_X_H_G)
                gyr_combined = (gyr_l | gyr_h <<8)

                return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536
          

        def readGYRy():
                gyr_l = bus.read_byte_data(GYR_ADDRESS, OUT_Y_L_G)
                gyr_h = bus.read_byte_data(GYR_ADDRESS, OUT_Y_H_G)
                gyr_combined = (gyr_l | gyr_h <<8)

                return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536

        def readGYRz():
                gyr_l = bus.read_byte_data(GYR_ADDRESS, OUT_Z_L_G)
                gyr_h = bus.read_byte_data(GYR_ADDRESS, OUT_Z_H_G)
                gyr_combined = (gyr_l | gyr_h <<8)

                return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536



                
        #initialise the accelerometer
        writeACC(CTRL_REG1_XM, 0b01100111) #z,y,x axis enabled, continuos update,  100Hz data rate
        writeACC(CTRL_REG2_XM, 0b00001000) #+/- 4G full scale

        #initialise the magnetometer
        writeMAG(CTRL_REG5_XM, 0b10010100) #Temp enable, M data rate = 30Hz
        writeMAG(CTRL_REG6_XM, 0b11100000) #+/-8gauss
        writeMAG(CTRL_REG7_XM, 0b00000000) #Continuous-conversion mode

        #initialise the gyroscope
        writeGRY(CTRL_REG1_G, 0b00001111) #Normal power mode, all axes enabled
        writeGRY(CTRL_REG4_G, 0b00000000) #Continuos update, 245 dps full scale

        gyroXangle = 0.0
        gyroYangle = 0.0
        gyroZangle = 0.0
        CFangleX = 0.0
        CFangleY = 0.0
        print "BerryIMU Ready"        
        
        print "pedestrianSACam Ready"

        #create data folder
        # can I get a time from GPS?
        currenttime = gpscontrol.fixdatetime
        # if not, use the system time
        if(currenttime == None):
            currenttime = datetime.datetime.now()
        foldername = args.path + "/" + "{0:02d}".format(currenttime.year) + "{0:02d}".format(currenttime.month) + "{0:02d}".format(currenttime.day) + "{0:02d}".format(currenttime.hour) + "{0:02d}".format(currenttime.minute) + "{0:02d}".format(currenttime.second)
        if not os.path.exists(foldername): os.makedirs(foldername)
        print "Data - folder created - " + foldername
        
        #create data file
        datafile = open(foldername+"/data.csv", "w")
        
        
        #start recording
        #create picamera
        with picamera.PiCamera() as camera:
            #setup camera
            camera.resolution = (VIDEOWIDTH, VIDEOHEIGHT)
            camera.framerate = VIDEOFPS
            camera.vflip = True
            camera.hflip = True
            camera.video_stabilization = True
            camera.rotation = 270
            
            #start recording
            camera.start_recording(foldername+"/vid.h264", inline_headers=False)
            print "Recording - started pi camera"
            #datafile.write("Camera recording started at " + "{0:02d}".format(currenttime.year) + " " + "{0:02d}".format(currenttime.month) + " " + "{0:02d}".format(currenttime.day) + " " + "{0:02d}".format(currenttime.hour) + ":" + "{0:02d}".format(currenttime.minute) + ":" +  "{0:02d}".format(currenttime.second) + "\n")
            startTime = datetime.datetime.now()
            n = True
            while(n):
                #get frame number
                framenumber = camera.frame.index
                #wait for a bit, the GPS data is a little behind + give the processor a rest
                time.sleep(0.1)
                #record data
                dataString = str(framenumber) + ","
                dataString += str(gpscontrol.fix.mode) + ","
                #dataString += str(gpscontrol.fixdatetime) + ","
                deltaTime = datetime.datetime.now() - startTime
                dataString += str(deltaTime) + "," 
                #dataString += str(gpscontrol.fix.time) + ","
                dataString += str(gpscontrol.fix.latitude) + ","
                dataString += str(gpscontrol.fix.longitude) + ","
                #dataString += str(gpscontrol.fix.altitude) + ","
                dataString += str(gpscontrol.fix.speed) + ","
                dataString += str(gpscontrol.fix.track) + ","
                #dataString += str(gpscontrol.fix.climb) + "\n"
                

                #calculate IMU data
                a = datetime.datetime.now()

                #Read our magnetometer, accelerometer and gyro values
                                #Read our magnetometer, accelerometer and gyro values
                MAGx = readMAGx()
    ##                MAGx = -MAGx
                MAGy = readMAGy()
    ##                MAGy = -MAGy
                MAGz = readMAGz()
    ##                MAGz = -MAGz
                ACCx = readACCx()
                ACCx = -ACCx
                ACCy = readACCy()
    ##                ACCy = -ACCy
                ACCz = readACCz()
                ACCz = -ACCz
                GYRx = readGYRx()
                GYRx = -GYRx
                GYRy = readGYRy()
    ##                GYRy = -GYRy
                GYRz = readGYRz()
                GYRz = -GYRz

                #Convert Accelerometer values to degrees
                AccXangle =  (math.atan2(ACCy,ACCz)+M_PI)*RAD_TO_DEG
                AccYangle =  (math.atan2(ACCz,ACCx)+M_PI)*RAD_TO_DEG



                #Convert Gyro raw to degrees per second
                rate_gyr_x =  GYRx * G_GAIN
                rate_gyr_y =  GYRy * G_GAIN
                rate_gyr_z =  GYRz * G_GAIN


                #Calculate the angles from the gyro. LP = loop period 
                gyroXangle+=rate_gyr_x*LP
                gyroYangle+=rate_gyr_y*LP
                gyroZangle+=rate_gyr_z*LP


                #Change the rotation value of the accelerometer to -/+ 180 and move the Y axis '0' point to up.
                #Two different pieces of code are used depending on how your IMU is mounted.
                #If IMU is upside down
                #
                if AccXangle >180:
                        AccXangle -= 360.0
                AccYangle-=90
                if (AccYangle >180):
                        AccYangle -= 360.0
                

                #If IMU is up the correct way, use these lines
        ##                AccXangle -= 180.0
        ##                if AccYangle > 90:
        ##                        AccYangle -= 270.0
        ##                else:
        ##                        AccYangle += 90.0
                
                #Complementary filter used to combine the accelerometer and gyro values.
                CFangleX=AA*(CFangleX+rate_gyr_x*LP) +(1 - AA) * AccXangle
                CFangleY=AA*(CFangleY+rate_gyr_y*LP) +(1 - AA) * AccYangle



                #Calculate heading
                heading = 180 * math.atan2(MAGy,MAGx)/M_PI

                if heading < 0:
                        heading += 360

                #Normalize accelerometer raw values.
                accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
                accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)

                #Calculate pitch and roll
                pitch = math.asin(accXnorm)
                roll = math.asin(accYnorm/math.cos(pitch))

                #Calculate the new tilt compensated values
                magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
                magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)

                #Calculate tilt compensated heading
                tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/M_PI

                if tiltCompensatedHeading < 0:
                        tiltCompensatedHeading += 360


        ##                print "Heading: " , heading,"\n"
        ##                print "Tilt Compensated heading" , tiltCompensatedHeading
        ##
        ##
        ##                time.sleep(0.03)
        ##                b = datetime.datetime.now()
        ##                c = b - a
        ##
        ##                print "Loop Time |",  c.microseconds/1000,"|","\n"

                print ("\033[1;34;40mACCX Angle %5.2f ACCY Angle %5.2f\033[1;31;40m\tGRYX Angle %5.2f  GYRY Angle %5.2f  GYRZ Angle %5.2f \033[1;35;40m    \tCFangleX Angle %5.2f \033[1;36;40m  CFangleY Angle %5.2f \33[1;32;40m  HEADING  %5.2f \33[1;37;40m tiltCompensatedHeading %5.2f\033[0m  " % (AccXangle, AccYangle,gyroXangle,gyroYangle,gyroZangle,CFangleX,CFangleY,heading,tiltCompensatedHeading))


                time.sleep(0.03)
                b = datetime.datetime.now()
                c = b - a

                print "Loop Time |",  c.microseconds/1000,"|",

                #the following may be useful for IMU data
                dataString += str(CFangleX) + ","
                dataString += str(CFangleY) + ","
                dataString += str(heading) + ","
                dataString += str(tiltCompensatedHeading) + ","
                dataString += str(c.microseconds/1000) + ","
                print(dataString)

                dataString += str(MAGx) + ","
                dataString += str(MAGy) + ","
                dataString += str(MAGz) + ","
                dataString += str(ACCx) + ","
                dataString += str(ACCy) + ","
                dataString += str(ACCz) + ","
                dataString += str(GYRx) + ","
                dataString += str(GYRy) + ","
                dataString += str(GYRz) + "\n"
                #dataString += str(tiltCompensatedHeading) + "\n"

                datafile.write(dataString)
                #debug, print data to screen
##                print(dataString)


##                n = n+1
            #stop the camera
            camera.stop_recording()
            camera.close()

            
        #recording has finished
        print "Recording - stopped"
            
        #close data file
        datafile.close()

        
        #wait for a bit
        time.sleep(0.1)
        #debug, dots to see code is running
        #print "."

    except KeyboardInterrupt:
        n = False
        print "User Cancelled (Ctrl C)"
    
    except:
        print "Unexpected error - ", sys.exc_info()[0], sys.exc_info()[1]
        raise

    finally:
        print "Stopping Pelmetcam"
        #shutdown gps controller
        print "Stopping GPS controller"
        gpscontrol.stopController()
        gpscontrol.join()
        #GPIO.cleanup()
        print "Stopped"
