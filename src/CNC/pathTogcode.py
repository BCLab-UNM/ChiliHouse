# Written by Andrei Popa-Simil

#Made for Leapfrog Creatr HS
# https://support.lpfrg.com/support/solutions/1000108213
# https://support.lpfrg.com/support/solutions/articles/11000083231-user-manual
# https://support.lpfrg.com/support/solutions/articles/11000083136-spare-parts-list-leapfrog-creatr-hs-xl-
# Max bed size 270mm x 270
# "In the gcode, all movements of your printer and characteristics of the different layers are encrypted"

# (O,O,O) : (X,Y,Z) = Left, Front, Top.
# (270,280,180) : (X-max, Y-max, Z-max)

# https://fabricesalvaire.github.io/pythonic-gcode-machine/gcode-reference/rs-274/index.html
# https://www.3erp.com/blog/g-code/
# Some of the best G-code editing software are NC Viewer, Notepad++, Cura, gCode Editor, and G-code QnDirty.

# Libraries
# https://docs.python.org/3/library/argparse.html
# https://pyserial.readthedocs.io/en/latest/shortintro.html
# https://docs.python.org/3/library/io.html#module-io

########################################## IMPORTS ################################################

import sys
import argparse
import serial
import time
import csv




################################### COMMANDLINE ARGS ##############################################
"""
for arg in sys.argv:
    print(arg)
"""

parser=argparse.ArgumentParser(
                    prog='pathTogcode.py',
                    description='The program requires the following libraries: [pyserial]. '
                    +'The program, allows for complete control of a 3 axis CNC device.'
                    +' Designed to allow a user to set it on a path and inturrupt to assign new paths.'
                    +' This program is intended for use by the UNM NASA MINDS 2024 team in the APH for sensor control.'
                    +' A set of basic movements commands are provided and options to read other path/gcode files will be provided',
                    epilog='\n This code was written by Andrei Popa-Simil\n'
)


# Special Arguments
parser.add_argument('-v','--verbose',help="Verbose mode\n",action='store_true')
parser.add_argument('-V','--version',help="Display current version info",action='store_true')
parser.add_argument('-i','--igloo',help="sets igloo name",action='store')
parser.add_argument('-p','--print',help="prints g-code",action='store_true')

# PATH Arguments
parser.add_argument('-if','--input_file',help="add input file path",action='store')
parser.add_argument('-of','--output_file',help="add output file path",action='store')
parser.add_argument('-dev','--device',help="choose serial device path to send/recieve g-code to and from",action='store')
parser.add_argument('-baud','--baudrate',help="sets baudrate of device",action='store')
parser.add_argument('-wait','--wait_time',help="sets wait time between commands",action='store')
parser.add_argument('-tout','--timeout',help="sets connection timeout",action='store')


# Header/Footer Options
parser.add_argument('-ah','--header',help="adds header",action='store_true')
parser.add_argument('-af','--footer',help="adds footer",action='store_true')

# G-code Options
parser.add_argument('-c','--calibrate',help="calibrate",action='store_true')
parser.add_argument('-gcmd','--gcommand',help="input a g-code command",action='store')
parser.add_argument('-gr','--graster',help="g-code for raster scan",action='store_true')
parser.add_argument('-grp','--graster_pause',help="does a raster scan with pauses",action='store_true')
parser.add_argument('-gp','--gpoint',help="input <x> <y> <z> coords to move head to",action='store')
parser.add_argument('-gps','--gpoints',help="input cvs with -if and reads xyz values",action='store_true')


# Printer Options
parser.add_argument('-pb','--printer_build',help="choose printer from: leapfrog, prusamini",action='store')

parser.add_argument('-ph','--home',help="input <x> <y> <z> coords to set home x,y,z",action='store')

parser.add_argument('-Mx','--max_x',help="set the max x axis value",action='store')
parser.add_argument('-mx','--min_x',help="set the max x axis value",action='store')
parser.add_argument('-My','--max_y',help="set the max y axis value",action='store')
parser.add_argument('-my','--min_y',help="set the max y axis value",action='store')
parser.add_argument('-Mz','--max_z',help="set the max z axis value",action='store')
parser.add_argument('-mz','--min_z',help="set the max z axis value",action='store')

parser.add_argument('-ix','--invert_x',help="inverts the direction of movement on the x axis",action='store_true')
parser.add_argument('-iy','--invert_y',help="inverts the direction of movement on the y axis",action='store_true')
parser.add_argument('-iz','--invert_z',help="inverts the direction of movement on the z axis",action='store_true')


args=parser.parse_args()


if(args.verbose):
    """
    print("verbose {}\nhelp".format(
        args.verbose,
        args.igloo
        ))
    """
    print(args.verbose)
    print(args.igloo)

####################################### SERIAL METHODS ############################################
# Vars
SERIAL_CACHE_SIZE=255 #bytes
BAUDRATE=115200 #bits/sec
TIMEOUT=1 #second(s)
if(args.printer_build=='prusamini'):
    print("Setting Serial Connection for Prusa Mini.")
    SERIAL_CACHE_SIZE=255 #bytes
    BAUDRATE=115200 #bits/sec
else:
    print("Setting Serial Connection for Leapfrog.")


def send_command(command):
    ser.write((command+'\n').encode())
    response=ser.readline().decode().strip()
    print("Response",response)

def read_buffer():
    """response=ser.readlines().decode().strip()
    print("Responses:", response)"""
    lines = []
    while True:
        line = ser.readline().decode().strip()  # Read a line from the serial port
        if line:  # If line is not empty
            lines.append(line)  # Append the line to the list
        else:
            break  # Exit the loop if there's no more data available

    # Print all lines grabbed
    for line in lines:
        print(line)

def write_dev(command):
    devpath=open(args.device,'w')
    write((command+'\n'))
    #response=ser.readline().strip()
    #print("Response",response)
    devpath.close()

###################################### PRINTER SELECTION ##########################################
#fpath = open("rasterscan.csv",r)

#Default Maxes and Mins in milimeters
#Space Maxes and Mins
XMAX=250
XMIN=0
YMAX=250
YMIN=0
ZMAX=170#180
ZMIN=0

#PrintHead maxs and mins
PXMAX=40
PXMIN=-40
PYMAX=50
PYMIN=-50
ZGH=2 #Gantry Height

# Set up offsets
XSIZE=XMAX-XMIN
YSIZE=YMAX-YMIN
ZSIZE=ZMAX-ZMIN
PXOFFSET=-40
PYOFFSET=-25
PZOFFSET=-0

#Choose Printer Build
if(args.printer_build=='prusamini'):
    print("Setting XYZ Axis for Prusa Mini.")
    #Space Maxes and Mins
    XMAX=180
    XMIN=0
    YMAX=180
    YMIN=0
    ZMAX=180
    ZMIN=0

    #PrintHead maxs and mins
    PXMAX=0
    PXMIN=-0
    PYMAX=0
    PYMIN=-0
    ZGH=1 #Gantry Height

    # Set up offsets
    XSIZE=XMAX-XMIN
    YSIZE=YMAX-YMIN
    ZSIZE=ZMAX-ZMIN
    PXOFFSET=-0
    PYOFFSET=-0
    PZOFFSET=-0
else:
    print("Setting XYZ Axis for Leapfrog.")


###################################### G-CODE COMMANDS ############################################

def calibrate():
    gstr="\n"
    gstr=gstr+"G29"
    return gstr


def LawnmowerScan(dx,margin,Z):
    gstr="\n"
    Xmin=margin+PXOFFSET
    Xmax=XMAX-margin+PXOFFSET
    Ymin=margin+PYOFFSET
    Ymax=YMAX-margin+PYOFFSET

    #gstr=gstr+Point_move(margin+PXOFFSET,margin+PYOFFSET,Z)
    gstr=gstr+XYmove_to_G1(margin+PXOFFSET,margin+PYOFFSET)
    for i in range(margin+PXOFFSET+dx,Xmax+dx,2*dx):
        gstr=gstr+XYmove_to_G1(i,Ymin)
        gstr=gstr+XYmove_to_G1(i,Ymax)
        gstr=gstr+XYmove_to_G1(i+dx,Ymax)
        gstr=gstr+XYmove_to_G1(i+dx,Ymin)

    return gstr


def RasterScan(dx,margin,startX,startY,Z):

    gstr="\n"
    Xmin=margin+PXOFFSET
    Xmax=XMAX-margin+PXOFFSET
    Ymin=margin+PYOFFSET
    Ymax=YMAX-margin+PYOFFSET

    gstr=gstr+XYmove_to_G1(margin+PXOFFSET,margin+PYOFFSET)
    for i in range(margin+PXOFFSET+dx,Xmax+dx,2*dx):
        gstr=gstr+XYmove_to_G1(i,Ymin)
        gstr=gstr+XYmove_to_G1(i,Ymax)
        gstr=gstr+XYmove_to_G1(i+dx,Ymax)
        #gstr=gstr+XYmove_to_G1(i+dx,Ymin)

    return gstr

def PausingLawnmowerScan(dx,margin,Z,pausetime):
    gstr="\n"
    Xmin=margin+PXOFFSET
    Xmax=XMAX-margin+PXOFFSET
    Ymin=margin+PYOFFSET
    Ymax=YMAX-margin+PYOFFSET

    #gstr=gstr+Point_move(margin+PXOFFSET,margin+PYOFFSET,Z)
    gstr=gstr+XYmove_to_G1(margin+PXOFFSET,margin+PYOFFSET)
    for i in range(margin+PXOFFSET+dx,Xmax+dx,2*dx):
        #Pause
        gstr=gstr+G4Pause(pausetime)
        #Move
        gstr=gstr+XYmove_to_G1(i,Ymin)
        gstr=gstr+XYmove_to_G1(i,Ymax)

        #Pause
        gstr=gstr+G4Pause(pausetime)
        #Move
        gstr=gstr+XYmove_to_G1(i+dx,Ymax)
        gstr=gstr+XYmove_to_G1(i+dx,Ymin)


    return gstr


def PointScanXY(pointarr):
    gstr="\n"
    #sort arr by y value
    # or just attack every point in order
    for point in pointarr:
        gstr=gstr+XYmove_to_G1(point.x,point.y)

def PointScanXYZ(pointarr):
    gstr="\n"
    sorted_points = sorted(pointarr, key=lambda point: (point[0]))
    for point in sorted_points:
        gstr=gstr+Point_move(point[0],point[1],point[2])
    return gstr



def G4Pause(miliseconds):
    return "\nG4 P"+str(miliseconds)

def XYmove_to_G1(x,y):
    return "\nG1 X"+str(x)+" Y"+str(y)


def Xmove_to_G1(x):
    return "\nG1 X"+str(x)


def Ymove_to_G1(y):
    return "\nG1 Y"+str(y)

def Zmove_to_G1(z):
    return "\nG1 Z"+str(z)

def Point_move(x,y,z):
    return "\nG1 Z"+str(ZMAX)+"\nG1 X"+str(x)+" Y"+str(y)+"\nG1 Z"+str(z)


###################################### G-CODE DEFAULTS ############################################

gheader=""" 
; ; Compiled G-Codefile
; ; Made by Andrei Popa-Simil
; ; UNM NASA-MINDS Spring 2024

G28 X0 Y0
G28 Z0
G92 X0 Y0 Z0
"""



squareMill_body="""    
    G1 Z-2          ; Lower the tool to start
    G1 X20          ; do the first side of the square
    G1 Y20          ; do the second side of the square
    G1 X10          ; do the third side of the square 
    G1 Y10          ; do the fourth side of the square\n
    """

gfooter="""
; Start Footer
G28 X0
G1 Y200
G1 Z160
M84
"""

######################################## FILE COMMANDS ############################################
def readInputCVS():
    print("running")
    if(args.input_file):
        print("Hi")
        csvpath = open(args.input_file,'r')
        points=[]
        reader=csv.reader(csvpath)
        for row in reader:
            print(row)
            if len(row)==3:
                x,y,z=map(float,row)
                points.append((x,y,z))
                
            else:
                print("Skipping invalid row:",row)
    return points

######################################## FILE ACTIONS #############################################








# Open Path to output to
gpath = open("path.tmp.gcode","w")

# Write Header
if(args.header):
    gpath.write(gheader)
    gpath.write("\n")

if(args.calibrate):
    gpath.write(calibrate())
    gpath.write("\n")

# Identify which methods to use

if(args.graster):
    gpath.write(LawnmowerScan(10,20,ZMAX-10))
    gpath.write("\n")
    #gpath.write(makeRasterScan(5,10,20,20,40))
    #gpath.write(squareMill_body)

if(args.graster_pause):
    gpath.write(PausingLawnmowerScan(25,20,ZMAX-10,1000))
    gpath.write("\n")


if(args.gpoints):
    pointarr=readInputCVS()
    #print(pointarr)
    gpath.write(PointScanXYZ(pointarr))




if(args.gcommand):
    gpath.write(args.gcommand)
    gpath.write("\n")

# Write Footer
if(args.footer):
    gpath.write(gfooter)
    gpath.write("\n")

#Close File paths
gpath.close

# Print G-code File
if(args.print):
    gpath=open("path.tmp.gcode","r")
    print(gpath.read())
    gpath.close()


###################################### SERIALIZATION ##############################################
# This section runs the 3d printer.
# Takes the command given and sends it to printer
# If raster scan is requested than it runs it from the generated file.

# If a device is given, open device and send gcode to device.
if(args.device):
    #Defaults
    ser = serial.Serial()
    ser.baudrate = BAUDRATE
    ser.timeout = TIMEOUT
    ser_waittime = 1
    ser.port=args.device

    # User input
    if(args.baudrate):
        ser.baudrate = int(args.baudrate)
    
    if(args.wait_time):
        ser_waittime = int(args.wait_time)
    
    if(args.timeout):
        ser_waittime = int(args.timeout)

    #display connection
    print(ser)

    #Open Connection
    ser.open()
    print("Serial Connection is open: "+str(ser.is_open))
    read_buffer()

    # This code runs command, but command is already tossed into file.
    """
    if(args.gcmd):
        send_command(args.gcmd)
    """

    """# If raster is requested. Read raster file.
    if(args.graster):"""
    gpath=open("path.tmp.gcode","r")
    while gpath:
        command=gpath.readline()
        print("Sending command: ",command)
        send_command(command)
        time.sleep(ser_waittime)
    gpath.close()
            

    #ser.write("G0 X0 Y0 Z0")
    ser.close()
    print("Serial Connection is open: "+str(ser.is_open))




