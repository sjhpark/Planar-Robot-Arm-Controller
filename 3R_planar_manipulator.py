import numpy as np
import tkinter as tk
import time
import optimized_IK

# 3R Planar Manipulator (Robot Arm) #
## The end effector will move in a stright line from one location to another location in the plane. ##

class RoboticArm:
    def __init__(self, master=None, canvas=None):

        # robot arm lengths
        self.seg1 = 150 # segment 1 length
        self.seg2 = 100 # segment 2 length
        self.seg3 = 50 # segment 3 length

        self.theta1 = np.pi/2 # starting angle of 1st segment [rads]
        self.theta2 = -np.pi/2 # starting angle of 2nd segment [rads]
        self.theta3 = -np.pi/2 # starting angle of 3rd segment [rads]

        # canvas
        self.master = master
        self.canvas = canvas


    def FK(self, theta1, theta2, theta3):
        '''
        Forward Kinematics
        Inputs:
            theta1: joint 1 angle [rads]
            theta2: joint 2 angle [rads]
            theta3: joint 3 angle [rads]
        Outputs:
            x1: x position of 1st segment end
            y1: y position of 1st segment end
            x2: x position of 2nd segment end
            y2: y position of 2nd segment end
            x3: x position of end effector
            y3: y position of end effector
        '''
        # robot arm segments lengths
        seg1 = self.seg1; seg2 = self.seg2; seg3 = self.seg3

        # end effector orientation [rads]
        psi = theta1 + theta2 + theta3

        # 1st segment end position
        x1 = seg1 * np.cos(theta1)
        y1 = seg1 * np.sin(theta1)

        # 2nd segment end position
        x2 = seg1 * np.cos(theta1) + seg2 * np.cos(theta1 + theta2)
        y2 = seg1 * np.sin(theta1) + seg2 * np.sin(theta1 + theta2)

        # end effector position
        x3 = seg3 * np.cos(psi) + x2
        y3 = seg3 * np.sin(psi) + y2

        return x1, y1, x2, y2, x3, y3


    def grid2canvas(self, x, y):
        '''
        Grid coordinates -> Canvas coordinates
        e.g. (0,0) in grid coordinates is (w=-300,h=300) in canvas coordinates
        Inputs: x, y in grid coordinates
        Outputs: x, y in canvas coordinates
        '''
        return x + 300, 300 - y


    def canvas2grid(self, x, y):
        '''
        Canvas coordinates -> Grid coordinates
        e.g. (w=-300,h=300) in canvas coordinates is (0,0) in grid coordinates
        Inputs: x, y in canvas coordinates
        Outputs: x, y in grid coordinates
        '''
        return x-300, 300-y
        

    def create_grid(self):
        '''
        Draw a grid on the canvas.
        '''
        # create a canvas
        canvas = self.canvas # canvas
        width_grid = canvas.winfo_reqwidth() # width of canvas
        height_grid = canvas.winfo_reqheight() # height of canvas
        cell_width = 20 # width of each cell
        cell_height = 20 # height of each cell
        
        # draw vertical lines
        for line in range(0, width_grid, cell_width):
            canvas.create_line([(line, 0), (line, height_grid)], 
                                fill='grey', tags='grid_line_w')

        # draw horizontal lines
        for line in range(0, height_grid, cell_height):
            canvas.create_line([(0, line), (width_grid, line)], 
                                fill='grey', tags='grid_line_h')

        # draw an arc
        canvas.create_arc((0,600,600,0), start=0, extent=180, outline='blue', width=3)

        # draw the arm at the starting position
        global i
        try:
            i
        except NameError:
            edit = self.grid2canvas # converter: grid coordinates -> canvas coordinates
            x0=0; y0=0
            x1=0; y1=150; x2=100; y2=150; x3=100; y3=150-50
            canvas.create_line(edit(x0,y0), edit(x1,y1), fill="forest green", width=4)
            canvas.create_line(edit(x1,y1), edit(x2,y2), fill="red", width=4)
            canvas.create_line(edit(x2,y2), edit(x3,y3), fill="orange", width=4)
            i = 'After Starting Position'
        else:
            pass


    def draw_arm(self, canvas, x1, y1, x2, y2, x3, y3):
        '''
        Draw robotic arm segments
        '''
        self.create_grid() # create a grid
        
        edit = self.grid2canvas # converter: grid coordinates -> canvas coordinates

        x0 = 0; y0 = 0 # base link is always (0,0)
        canvas.create_line(edit(x0,y0), edit(x1,y1), fill="forest green", width=4)
        canvas.create_line(edit(x1,y1), edit(x2,y2), fill="red", width=4)
        canvas.create_line(edit(x2,y2), edit(x3,y3), fill="orange", width=4)


    def move(self, canvas, x, y):
        '''
        Move the robotic arm to a desired position in a stright line motion.
        Inputs: 
            x, y: desired end effector positions (in grid coordinates)
        Global Inputs:
            psi [rads]: desired end effector orientation
            steps: number of steps to move the end effector to the desired position
        '''
        
        try:
            global xe_start, ye_start
            xe_start, ye_start
        except NameError:
            xe_start, ye_start = 100, 100
        else:
            global x3, y3
            xe_start, ye_start = x3, y3

        x_step = (x-xe_start)/steps # step size in x direction
        y_step = (y-ye_start)/steps # step size in y direction
        xe = xe_start; ye = ye_start # starting postion of the end effector
        theta1 = self.theta1; theta2 = self.theta2; theta3 = self.theta3 # starting angles

        # while the end effector is not at the desired position
        while xe != x or ye != y:
            xe += x_step # desired x end effector position
            ye += y_step # desired y end effector position

            theta1, theta2, theta3 = optimized_IK.optimized_angles(xe, ye, psi, theta1, theta2, theta3)
            x1, y1, x2, y2, x3, y3 = self.FK(theta1, theta2, theta3) # positions from forward kinematics
            
            ########################### Canvas & Window Updates ###########################
            canvas.delete('all') # clear canvas

            edit = self.grid2canvas # converter: grid coordinates -> canvas coordinates
            canvas.create_line(edit(x3,y3), edit(x,y), 
                                dash=(1,1), fill="white", 
                                width=2) # draw a dotted line from end effector to desired position

            self.draw_arm(canvas, x1, y1, x2, y2, x3, y3) # draw grid with robotic arm segments
            canvas.update() # update canvas

            tk.Label(self.master, 
            text = '\u03F41: ' + str(np.round(360*theta1/(2*np.pi),2)) + 'deg\n'
                    + '\u03F42: '+str(np.round(360*theta2/(2*np.pi),2)) + 'deg\n'
                    + '\u03F43: '+str(np.round(360*theta3/(2*np.pi),2)) + 'deg',
            font = ("Calibri", 15, 'bold'),
            fg = 'green').grid(row=7, column=0, sticky='ew')            
            ################################################################################
            
            time.sleep(0.15) # pause

            # Break the loop if the desired position is within the buffer zone
            buffer = 1e-5
            if np.abs(xe - x) <= buffer or np.abs(ye - y) <= buffer:
                break
    

    def click(self, event):
        '''
        The end effector position will be located at the point where the user clicks.
        '''
        x, y = self.canvas2grid(event.x, event.y) # clicked (x,y) position in grid coordinates

        tk.Label(self.master, 
                text = 'x: ' + str(x) + ', y: ' + str(y),
                font = ("Calibri", 15, 'bold'),
                fg = 'lime green').grid(row=5, column=0, sticky='ew')

        canvas = self.canvas
        self.move(canvas, x, y)


if __name__ == '__main__':
    
    # MASTER WINDOW
    master = tk.Tk() # master (root) window
    master.geometry("600x600") # size of window
    master.title("Robotic Arm Controller") # title of window

    # CANVAS
    width_grid = 600 # grid width
    height_grid = 300 # grid height
    canvas = tk.Canvas(master, 
                        width = width_grid, 
                        height = height_grid, 
                        bg = 'black')
    canvas.grid(row=0, column=0) # package all the canvas features

    # LABELS
    label1 = tk.Label(master, 
                    text = "3R Planar Manipulator Controller", 
                    bd = 5,
                    font = ("Calibri", 20, 'bold'),
                    bg = 'gold',
                    fg = 'blue').grid(row=1, column=0)
    label2 = tk.Label(master,
                    text = "Click anywhere on the canvas to move the end effector.",
                    bd = 5,
                    font = ("Calibri", 15, 'bold')).grid(row=2, column=0)
    label3 = tk.Label(master, 
                    text = 'Clicked Desired End Effector Location:',
                    font = ("Calibri", 15, 'bold'),
                    fg = 'blue').grid(row=4, column=0, sticky="ew")
 
    label4 = tk.Label(master, 
                    text = 'Optimized Joint Angles:',
                    font = ("Calibri", 15, 'bold'),
                    fg = 'blue').grid(row=6, column=0, sticky="ew") 
    
    # HYPERPARAMETERS
    global psi, steps
    psi = np.linspace(0, 2*np.pi, 256) # desired end effector orientation
    steps = 21 # number of steps to move the end effector

    # ROBOTIC ARM
    RA = RoboticArm(master, canvas) # robotic arm object
    RA.create_grid() # create a grid
    canvas.bind('<Button-1>', RA.click) # left click to move the end effector

    # MAINLOOP (Runs Infinitely)
    tk.mainloop()