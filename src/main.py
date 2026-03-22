"""
    Author: Uttara Naidu
    Date: 02/03/2026
    Version: v1.0
    About: Forward Kinematics calculations based on DH parameters

    Description:
        This script is split into two parts:

        STANDARD MODE: Mid Module Assignment (Part 1)
            -   Forward kinematic calculations are performed to determine the position and orientation of the end effector.
            -   The same is plotted using matplotlib.
            -   This mode is the default line of execution and fulfills the requirements of Mid Module Assignment (Part 1)

        ANIMATION MODE [AM]: Experimental mode
            -   A Linear Interpolation is performed in this mode where the 3R manipulator moves through its workspace from (0,0,0) to
                a user-specified target configuration (angle1,angle2,angle3).
            -   The user has to right-click to activate this mode.
    
        **CAUTION**:This animation is timing-sensitive. It only triggers if the user right-clicks the window immediately after the initial FK plot pops up. 
                    Optimization is in progress.

"""


import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def input_check(prompt):

    """
        Functional call for accepting and verifying user input. For ease of use and better readability, this project accepts angles in degrees.
        But all Robotics calculations are performed in radians for highest precision.
    """

    while True:
        str = input(prompt)
        try:
            x = float(str)
            if x > 180.0 or x < -180.0:                                     # pre-defined range for angles from -180 to +180 degrees
                raise ValueError
        except ValueError:
            print("Please enter a valid numerical value for the angle between - 180 to 180 degrees.")
            continue
        return x



def forward_kinematics_calc_radians(theta1, theta2, theta3):

    """ 
        Function call to calculate the end effector position given a set of user-defined angles.
        Although the angles are accepted in degrees, all calculations are performed in radians to achieve highest precision.
        The equations defined here for x1,x2,x3 and y1,y2,y3 are derived using DH table and a homogenous Transformation matrix. 
        Detailed derivation in the MMA report.
    """
    try:
        x0, y0 = 0,0
        x1 = A1*np.cos(np.radians(theta1))
        y1 = A1*np.sin(np.radians(theta1))
        
        x2 = x1+A2*np.cos(np.radians(theta1+theta2))
        y2 = y1+A2*np.sin(np.radians(theta1+theta2))
        
        x3 = x2+A3*np.cos(np.radians(theta1+theta2+theta3))
        y3 = y2+A3*np.sin(np.radians(theta1+theta2+theta3))

        x = [np.round(x0,4),np.round(x1,4),np.round(x2,4),np.round(x3,4)]
        y = [y0,y1,y2,y3]
        orientation = np.radians(theta1+theta2+theta3)
        orientation = np.round(orientation,4)
    except:
        raise TypeError
    
    return x,y,orientation


def forward_kinematics_calc_degrees(theta1, theta2, theta3):

    """ 
        DEPRECATED: Only used for comparison purposes
    """
    try:
        x0, y0 = 0,0
        x1 = A1*np.cos(theta1)
        y1 = A1*np.sin(theta1)
        
        x2 = x1+A2*np.cos(theta1+theta2)
        y2 = y1+A2*np.sin(theta1+theta2)
        
        x3 = x2+A3*np.cos(theta1+theta2+theta3)
        y3 = y2+A3*np.sin(theta1+theta2+theta3)

        x = [x0,x1,x2,x3]
        y = [y0,y1,y2,y3]
        orientation = theta1+theta2+theta3
    except:
        raise TypeError
    
    return x,y,orientation

def init():

    """
        Function call to set initial plot variables.
    """
    axes_limit = (A1+A2+A3)
    ax.set_xlim(-axes_limit,axes_limit)
    ax.set_ylim(-axes_limit,axes_limit)
    ax.set_xlabel('')
    ax.set_ylabel('')
    return line, end_effector_pos,end_effector_pos_label,trace_label



def animate(frame):

    """
        Function call to demonstrate the end effector position in a 2D space
    """
    if CURRENT_MODE == 0:                                                                                                  
        """
            This is the DEFAULT_MODE
        """
        x_coords, y_coords, orientation = forward_kinematics_calc_radians(angle1, angle2, angle3)                           #performing FK calculations
        line.set_data(x_coords,y_coords)
        end_effector_pos.set_data([x_coords[-1]], [y_coords[-1]])                                                           #setting end effector coordinates
        info_label.set_text(f'Joint angles [degree]: {angle1}, {angle2}, {angle3} \nOrientation (Φ): {orientation:.2f}°')   #displaying orientation and user-defined angles in degrees
        for i, (x,y) in enumerate(zip(x_coords,y_coords)):                                                                  #displaying coordinates for all joints at an offset w.r.t actual coordinates                           
            end_effector_pos_label[i].set_position((x-7+i,y-7+i))
            end_effector_pos_label[i].set_text(f'({x:.2f},{y:.2f})')
    else:
        """
            EXPERIMENTAL SECTION: ANIMATION_MODE
                -   Animation to present the planar robot arm rotating in its world space
                -   This animation is timing-sensitive. It only triggers if the user right-clicks the window immediately after the initial FK plot appears. 
                    Optimization is in progress.
                -   Experimental user-defined angles: (50,-30,20)
        """
        
        theta1, theta2, theta3 = linear_interpolation[frame]                                        #extracting joint angles for current frame
        x_coords, y_coords, orientation = forward_kinematics_calc_radians(theta1, theta2, theta3)   #calculating FK for the initial set of angles
        
        info_label.set_text('')                                                                     #clearing plot labels
        for i in range(4):
            end_effector_pos_label[i].set_text('')

        history_x.append(x_coords[-1])                                                              #storing end effector coordinates to demonstrate the trace
        history_y.append(y_coords[-1])

        ax.set_title('Linear Interpolation Tracing')

        line.set_data(x_coords, y_coords)                                                           #setting coordinates
        trace.set_data(history_x, history_y)
        end_effector_pos.set_data([x_coords[-1]], [y_coords[-1]])

        trace_label.set_position((x_coords[-1]-10,y_coords[-1]-10))                                 #setting labels to display coordinates
        trace_label.set_text(f'({x_coords[-1]:.2f},{y_coords[-1]:.2f})')
        info_label.set_text(f'Joint angles [degree]: {angle1}, {angle2}, {angle3} \nOrientation (Φ): {orientation:.2f}°')
    
    return line, end_effector_pos,end_effector_pos_label,trace_label



def on_click(event):

    """
        Function call to set the mode of operation. 
    """
    global CURRENT_MODE
    if event.button == 3:                   #event is triggered on mouse right-click - to set Animation mode
        CURRENT_MODE = 1
    else:                                   #default event
        CURRENT_MODE == 0
    
    return line, end_effector_pos,end_effector_pos_label,trace_label




"""
     ----- Initializing global parameters ------

"""

A1, A2, A3 = 30,25,20       #given link lengths
CURRENT_MODE = 0            #[AM] flag to switch to animation mode
FRAMES = 60                 #[AM] number of frames used by animation to show transition (linear interpolation)


"""
    Initializing plotting variables 
"""
fig, ax = plt.subplots(figsize=(15,10))                                     #setting up the plot
line, = ax.plot([],[],'o-',linewidth=4,color='blue')                        #placeholder plot object for links
end_effector_pos, = ax.plot([],[],'rd',linewidth='4',markersize=8,zorder=4) #placeholder plot object for end-effector position
trace, = ax.plot([], [], '--', lw=1, color='gray', alpha=0.5, zorder=1)     #[AM] placeholder plot object for tracing the transition in animation mode

end_effector_pos_label = [ax.text(0,0,'',                                   #placeholder label for displaying coordinates of 3 joints and 1 end effector
                                  fontsize=8, 
                                  fontweight='bold',
                                  bbox=dict(facecolor='white', 
                                  alpha=0.8)) 
                          for _ in range(4)]                                
info_label = ax.text(0.02,                                                  #placeholder label for displaying orientation of end effector
                     0.95, 
                     '', 
                     transform=ax.transAxes, 
                     weight='bold')  
trace_label = ax.text(0,                                                    #[AM] placeholder label for displaying the changing coordinates of the end effector position in animation mode
                      0,
                      '',
                      fontsize=8, 
                      fontweight='bold',
                      bbox=dict(facecolor='white', 
                      alpha=0.8))


"""
    User-defined angles
"""
angle1 = input_check('Joint 1: Enter the Base angle in degrees: ')              #user input 1
angle2 = input_check('Joint 2: Enter the second joint angle in degrees: ')      #user input 2
angle3 = input_check('Joint 3: Enter the third joint angle in degrees: ')       #user input 3


print(f'Joint angles [degree]: {angle1}, {angle2}, {angle3} ')
print(f'Joint angles [radians]: {np.radians(angle1)}, {np.radians(angle2)}, {np.radians(angle3)} ')
target_angles = [angle1,angle2,angle3]                                          #[AM] customized user-defined angles marking the end of transition
fig.canvas.mpl_connect('button_press_event',on_click)                           #button_press event to switch to Animation mode of planar arm 
animation = FuncAnimation(fig, animate,                                         #a handle for animation - assigned to a variable to prevent garbage collection
                          init_func=init, 
                          blit=False, 
                          frames = FRAMES, 
                          repeat=False)    

"""
    Initializing additional parameters for Animation Mode
"""
start_angles = [0,0,0]                                                          #beginning of interpolation trace
history_x, history_y = [], []                                                   #lists for outlining the trace of linear interpolation transition
linear_interpolation = np.linspace(start_angles,target_angles,FRAMES)           #creating 

"""
    Plotting the manipulator
"""
plt.title("3-Link Planar Robot: Forward Kinematics & Orientation") 
plt.grid() 
plt.show() 