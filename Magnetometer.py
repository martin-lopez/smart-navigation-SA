import time
import datetime
import threading


#relevant BerryIMU imports
import smbus
import math
from LSM9DS0 import *




        #setup BerryIMU data stream
        bus = smbus.SMBus(1)

        RAD_TO_DEG = 57.29578
        M_PI = 3.14159265358979323846 # is this necessary in python
        G_GAIN = 0.00875  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
        LP = 0.041    	# Loop period = 41ms.   This needs to match the time it takes each loop to run
        AA =  0.80      # Complementary filter constant


        def writeMAG(register,value):
                bus.write_byte_data(MAG_ADDRESS, register, value)
                return -1


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


        #initialise the magnetometer
        writeMAG(CTRL_REG5_XM, 0b11110000) #Temp enable, M data rate = 50Hz
        writeMAG(CTRL_REG6_XM, 0b01100000) #+/-8gauss
        writeMAG(CTRL_REG7_XM, 0b00000000) #Continuous-conversion mode


        gyroXangle = 0.0
        gyroYangle = 0.0
        gyroZangle = 0.0
        CFangleX = 0.0
        CFangleY = 0.0
            
        while(True)
                        

                        #calculate IMU data
                        a = datetime.datetime.now()
	
                        #Read our magnetometer  values
                        MAGx = readMAGx()
                        MAGy = readMAGy()
                        MAGz = readMAGz()


                        #Calculate heading
                        heading = 180 * math.atan2(MAGy,MAGx)/M_PI

                        if heading < 0:
                                heading += 360

                        print "Heading: " , heading


                        time.sleep(0.03)
                        b = datetime.datetime.now()
                        c = b - a

                        print "Loop Time |",  c.microseconds/1000,"|","\n"


    except KeyboardInterrupt:
        print "User Cancelled (Ctrl C)"
    
    except:
        print "Unexpected error - ", sys.exc_info()[0], sys.exc_info()[1]
        raise

    finally:
        print "Stopped"
