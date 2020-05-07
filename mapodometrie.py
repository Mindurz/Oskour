import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches 
from matplotlib.widgets import Slider, Button, RadioButtons
import serial
import struct
import sys
import signal
import time
from threading import Thread

#Can be converted into a portable package by using the PyInstaller module
# pip install pyinstaller (need to be used with Python3)
# cf. https://pyinstaller.readthedocs.io/en/v3.3.1/usage.html


goodbye = """
          |\      _,,,---,,_
          /,`.-'`'    -.  ;-;;,_
         |,4-  ) )-,_..;\ (  `'-'
 _______'---''(_/--'__`-'\_)______   ______            _______  _
(  ____ \(  ___  )(  ___  )(  __  \ (  ___ \ |\     /|(  ____ \| |
| (    \/| (   ) || (   ) || (  \  )| (   ) )( \   / )| (    \/| |
| |      | |   | || |   | || |   ) || (__/ /  \ (_) / | (__    | |
| | ____ | |   | || |   | || |   | ||  __ (    \   /  |  __)   | |
| | \_  )| |   | || |   | || |   ) || (  \ \    ) (   | (      |_|
| (___) || (___) || (___) || (__/  )| )___) )   | |   | (____/\ _ 
(_______)(_______)(_______)(______/ |______/    \_/   (_______/(_)                                         
"""

goodbye2 = """
                   /\_/\\
                 =( °w° )=
                   )   (  //
                  (__ __)//
 _____                 _ _                _ 
|  __ \               | | |              | |
| |  \/ ___   ___   __| | |__  _   _  ___| |
| | __ / _ \ / _ \ / _` | '_ \| | | |/ _ \ |
| |_\ \ (_) | (_) | (_| | |_) | |_| |  __/_|
 \____/\___/ \___/ \__,_|_.__/ \__, |\___(_)
                                __/ |       
                               |___/        
"""


#number of samples for one line from the camera
n = 5
#maximum value for an uint8
max_value = 255

#handler when closing the window
def handle_close(evt):
    #we stop the serial thread
    reader_thd.stop()
    print(goodbye)
    
#update the plots
def update_plot():
    if(reader_thd.need_to_update_plot()):
        fig.canvas.draw_idle()
        reader_thd.plot_updated()

#function used to update the plot of the cam data
def update_cam_plot(port,oldx,oldy):

    data = readUint8Serial(port)

    if(len(data)>0):

        direction = 0

        x_new = data[0]

        y_new = data[1]
        
        nb_line = data[2]

        if(oldx > x_new):
            x_high = oldx + 8
            x_low = x_new + 2
            direction = 2
        elif(x_new > oldx):
            x_high = x_new + 8
            x_low = oldx + 2
            direction = 0
        else:
            x_high =x_new + 8
            x_low = oldx + 2
        if(oldy > y_new):
            y_high = oldy + 8
            y_low = y_new + 2
            direction = 3
        elif(y_new > oldy):
            y_high = y_new + 8
            y_low = oldy + 2
            direction = 1
        else:
            y_high =y_new + 8
            y_low = oldy + 2


        if(nb_line == 64):
            reader_thd.stop_reading(0)
        elif(direction == 0):    
                if(nb_line != 0):
                    if(nb_line == 1):
                        image[x_new + 8,y_low:y_high] = rouge
                    else:
                        image[x_new + 8,y_low:y_high] = bleu

        elif(direction == 2):
                if(nb_line != 0):
                    if(nb_line == 1):
                        image[x_new,y_low:y_high] = rouge
                    else:
                        image[x_new,y_low:y_high] = bleu
        
        elif(direction == 1):
                if(nb_line != 0):
                    if(nb_line == 1):
                        image[x_low:x_high,y_new+8] = rouge
                    else:
                        image[x_low:x_high,y_new+8] = bleu
        
        elif(direction == 3):
                if(nb_line != 0):
                    if(nb_line == 1):
                        image[x_low:x_high,y_new] = rouge
                    else:
                        image[x_low:x_high,y_new] = bleu
        else:  
            pass

        image[x_low:x_high,y_low:y_high] = blanc
        if(reader_thd.draw_start or nb_line == 64):
            image[x_low:x_low+6,y_low:y_low+6] = vert
            reader_thd.draw_start = 0
        
        
        imgplot = plt.imshow(image)

        plt.pause(1)

        reader_thd.x_old = x_new
        reader_thd.y_old = y_new

        reader_thd.tell_to_update_plot()


#reads the data in uint8 from the serial
def readUint8Serial(port):

    state = 0

    while(state != 5):

        #reads 1 byte
        c1 = port.read(1)
        #timeout condition
        if(c1 == b''):
            print('Timout...')
            return []

        if(state == 0):
            if(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 1):
            if(c1 == b'T'):
                state = 2
            elif(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 2):
            if(c1 == b'A'):
                state = 3
            elif(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 3):
            if(c1 == b'R'):
                state = 4
            elif (c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 4):
            if(c1 == b'T'):
                state = 5
            elif (c1 == b'S'):
                state = 1
            else:
                state = 0

    #reads the size
    #converts as short int in little endian the two bytes read
    size = struct.unpack('<h',port.read(2)) 
    
    #removes the second element which is void
    size = size[0]  
    data = []
    #reads the data
    rcv_buffer = port.read(size) 
    
    if(len(rcv_buffer) == size):
        i = 0
        while(i < size):
            data.append(rcv_buffer[i])
            i = i+1
        print('received !')
        return data
    else:
        print('Timout...')
        return []

def bytes_to_int(bytes):
    result = 0
    for b in bytes:
        result = result * 256 + int(b)
    return result

#thread used to control the communication part
class serial_thread(Thread):

    #init function called when the thread begins
    def __init__(self, port):
        Thread.__init__(self)
        self.contReceive = True
        self.alive = True
        self.need_to_update = False
        self.contSendAndReceive = False
        self.x_old = 0
        self.y_old = 0
        self.draw_start = 1


        print('Connecting to port {}'.format(port))

        try:
            self.port = serial.Serial(port, timeout=2)
        except:
            print('Cannot connect to the e-puck2')
            sys.exit(0)
    #function called after the init
    def run(self):
        update_cam_plot(self.port,self.x_old,self.y_old)
        self.port.write(b'1')

        while(self.alive):
            #self.port.write(b'1')
            if(self.contReceive):
                update_cam_plot(self.port,self.x_old,self.y_old)
            else:
                #flush the serial
                self.port.read(self.port.inWaiting())
                time.sleep(0.1)

    #enables the continuous reading
    def setContReceive(self, val):  
        self.contReceive = True

    #disables the continuous reading
    def stop_reading(self, val):
        self.contReceive = False

    def update_old(newx,newy):
        self.x_old = newx
        self.y_old = newy

    #tell the plot need to be updated
    def tell_to_update_plot(self):
        self.need_to_update = True

    #tell the plot has been updated
    def plot_updated(self):
        self.need_to_update = False

    #tell if the plot need to be updated
    def need_to_update_plot(self):
        return self.need_to_update

    #clean exit of the thread if we need to stop it
    def stop(self):
        self.alive = False
        self.join()
        if(self.port.isOpen()):
            while(self.port.inWaiting() > 0):
                self.port.read(self.port.inWaiting())
                time.sleep(0.01)
            self.port.close()

        
#test if the serial port as been given as argument in the terminal
if len(sys.argv) == 1:
    print('Please give the serial port to use as argument')
    sys.exit(0)

#serial reader thread config
#begins the serial thread
reader_thd = serial_thread(sys.argv[1])
reader_thd.start()

#figure config

#cam graph config with initial plot
rouge = [0.807, 0.066, 0.149]
blanc = [1, 1, 1]
bleu =  [0, 0.129, 0.878]
vert = [0, 0.8, 0]
hauteur = 36
largeur = 36
image = np.zeros((hauteur, largeur,3))
patches = []
patches.append(mpatches.Patch(color = blanc, label ="Path"))
patches.append(mpatches.Patch(color = rouge, label = "1 line"))
patches.append(mpatches.Patch(color = bleu, label = "2 lines"))
patches.append(mpatches.Patch(color = vert , label = "Start"))
plt.legend(handles=patches,bbox_to_anchor=(1.05,1), loc = 2, borderaxespad = 0.)
imgplot = plt.imshow(image)

#starts the matplotlib main
plt.show()