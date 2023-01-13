#Name: Mit Patel
#Student Number: 400308098
#MacID: patem97
#Bus Speed: 40MHz
#LED: D1


#Modify the following line with your own serial port details
#   Currently set COM3 as serial port at 115.2kbps 8N1
#   Refer to PySerial API for other options.  One option to consider is
#   the "timeout" - allowing the program to proceed if after a defined
#   timeout period.  The default = 0, which means wait forever.

import serial
import numpy as np
import open3d as o3d
import math

# Implement UART using pyserial
s=serial.Serial('COM3', baudrate=115200,timeout=60)
print("Opening: " + s.name)

# reset the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()

# wait for user's signal to start the program
input("Press Enter to start communication...")

y=[]
z=[]
index=0
pi = math.pi
s.readline()


# Iterates three times as there are 3 scans to be done
for x in range(3):

    # Prints the current scan number
    print("Scan "+ str(x+1))

    # Iterates 8 times to extract 8 different measurements
    for m in range(8):
        while 1:

            # Read the sent data
            p=s.readline()
            if(p!=''):
                break;
        a=p
        distance=int(float(a))

        # m represents the angle m=1 -> pi/4, 45 degrees
        z.insert(index,(distance*math.cos(pi*m/4)))
        y.insert(index,(distance*math.sin(pi*m/4)))
        print("x , y, z:"+str(float(x+1)*100)+", "+str(y[index])+", "+str(z[index]))
        index+=1

        
# the encode() and decode() function are needed to convert string to bytes
# because pyserial library functions work with type "bytes"


#close the port
print("Closing: " + s.name)
s.close()


if __name__ == "__main__":
    #Remember the goals of modularization
    #   -- smaller problems, reuse, validation, debugging
    #To simulate the data from the sensor lets create a new file with test data 
    f = open("demo2dx.xyz", "w")    #create a new file for writing 
    
    #Test data: Lets make a rectangular prism as a point cloud in XYZ format
    #   A simple prism would only require 8 vertices, however we
    #   will sample the prism along its x-axis a total of 10x
    #   4 vertices repeated 10x = 40 vertices
    #   This for-loop generates our test data in xyz format
    index=0
    for x in range(3):
        for j in range(8):
            f.write('{0:f} {1:f} {2:f}\n'.format(float(x*100),y[index],z[index]))
            index = index+1

    f.close()   #there should now be a file containing 40 vertex coordinates                               
    
    #Read the test data in from the file we created        
    print("Read in the prism point cloud data (pcd)")
    pcd = o3d.io.read_point_cloud("demo2dx.xyz", format="xyz")

    #Lets see what our point cloud data looks like numerically       
    print("The PCD array:")
    print(np.asarray(pcd.points))

    #Lets see what our point cloud data looks like graphically       
    print("Lets visualize the PCD: (spawns seperate interactive window)")
    o3d.visualization.draw_geometries([pcd])

    #OK, good, but not great, lets add some lines to connect the vertices
    #   For creating a lineset we will need to tell the packahe which vertices need connected
    #   Remember each vertex actually contains one x,y,z coordinate

    #Give each vertex a unique number
    yz_slice_vertex = []
    for x in range(0,24):
        yz_slice_vertex.append([x])

    #Define coordinates to connect lines in each yz slice        
    lines = []  
    for x in range(0,24,8):
        lines.append([yz_slice_vertex[x], yz_slice_vertex[x+1]])
        lines.append([yz_slice_vertex[x+1], yz_slice_vertex[x+2]])
        lines.append([yz_slice_vertex[x+2], yz_slice_vertex[x+3]])
        lines.append([yz_slice_vertex[x+3], yz_slice_vertex[x+4]])
        lines.append([yz_slice_vertex[x+4], yz_slice_vertex[x+5]])
        lines.append([yz_slice_vertex[x+5], yz_slice_vertex[x+6]])
        lines.append([yz_slice_vertex[x+6], yz_slice_vertex[x+7]])
        lines.append([yz_slice_vertex[x+7], yz_slice_vertex[x]])

    #Define coordinates to connect lines between current and next yz slice        
    for x in range(0,15,8):
        lines.append([yz_slice_vertex[x], yz_slice_vertex[x+8]])
        lines.append([yz_slice_vertex[x+1], yz_slice_vertex[x+9]])
        lines.append([yz_slice_vertex[x+2], yz_slice_vertex[x+10]])
        lines.append([yz_slice_vertex[x+3], yz_slice_vertex[x+11]])
        lines.append([yz_slice_vertex[x+4], yz_slice_vertex[x+12]])
        lines.append([yz_slice_vertex[x+5], yz_slice_vertex[x+13]])
        lines.append([yz_slice_vertex[x+6], yz_slice_vertex[x+14]])
        lines.append([yz_slice_vertex[x+7], yz_slice_vertex[x+15]])

    #This line maps the lines to the 3d coordinate vertices
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

    #Lets see what our point cloud data with lines looks like graphically       
    o3d.visualization.draw_geometries([line_set])
