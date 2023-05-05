try:
    # for Python2
    import Tkinter as tk
    from Tkinter import ttk
except ImportError:
    # for Python3
    import tkinter as tk
    from tkinter import ttk
import time
from tkinter.constants import S

class QrotorDisplayWidget(object):
    def __init__(self, parent, drone):
        self.parent = parent
        self.drone = drone

        # - - - - - - - - - - -
        # paramters
        self.bg_color = '#808080'

        # - - - - - - - - - - -
        # variables
        self.voltage = 0
        self.position = [0., 0., 0.]
        self.attitude_mode = 0
        self.loop_rate = 0

        # - - - - - - - - - - -
        # tk variables
        self.voltageString = tk.DoubleVar()
        self.positionString = tk.StringVar()
        self.looprateString = tk.StringVar()

        # - - - - - - - - - - -
        # display values
        status_frame = tk.LabelFrame(self.parent, text="status",bg=self.bg_color, relief=tk.RIDGE)
        status_frame.grid(row=1, column=1,  columnspan = 3, padx=6, sticky=tk.E + tk.W + tk.N + tk.S)

        l1 = tk.Label(status_frame,font=("LM Mono 8", 14), text="Voltage: ")
        l1.grid(row=2, column=1, sticky=tk.E + tk.W + tk.N + tk.S)
        l2 = tk.Label(status_frame, textvariable=self.voltageString, fg='#C70039', bg='#DAF7A6',font=("Helvetica", 15), width="10")
        l2.grid(row=2, column=2, columnspan=2, sticky=tk.E + tk.W + tk.N + tk.S)

        l1 = tk.Label(status_frame,font=("LM Mono 8", 14), text="Loop Rate: ")
        l1.grid(row=3, column=1, sticky=tk.E + tk.W + tk.N + tk.S)
        l2 = tk.Label(status_frame, textvariable=self.looprateString, fg='#581845', bg='#DAF7A6',font=("Helvetica", 12), width="10")
        l2.grid(row=3, column=2, columnspan=2, sticky=tk.E + tk.W + tk.N + tk.S)

        l3 = tk.Label(status_frame,font=("LM Mono 8", 14), text="Position: ")
        l3.grid(row=4, column=1, sticky=tk.E + tk.W + tk.N + tk.S)
        l4 = tk.Label(status_frame, textvariable=self.positionString, fg='#581845', bg = '#DAF7A6', font=("Helvetica", 12), width="25")
        l4.grid(row=4, column=2, columnspan=2, sticky=tk.E + tk.W + tk.N + tk.S)
        self.update_values()

        input_frame = tk.LabelFrame(self.parent, text="inputs",bg=self.bg_color, relief=tk.RIDGE)
        input_frame.grid(row=2, column=1,  columnspan = 3, padx=6, sticky=tk.E + tk.W + tk.N + tk.S)

        buttons_frame = tk.LabelFrame(self.parent, text="actions",bg=self.bg_color, relief=tk.RIDGE)
        buttons_frame.grid(row=3, column=1,  columnspan = 3, padx=6, sticky=tk.E + tk.W + tk.N + tk.S)
        b1 = tk.Button(buttons_frame, text='KILL DRONE', font=("LM Mono 10", 15), width=30 ,bg='#ff8000', command=self.drone.request_kill_vehicle)
        b1.grid(row=1, column=1, columnspan = 3, sticky=tk.W+tk.E+tk.S)
        b2 = tk.Button(buttons_frame, text='ARM DRONE', font=("LM Mono 10", 15), width=30 ,bg='#3cb371', command=self.drone.request_arm)
        b2.grid(row=2, column=1, columnspan = 3, sticky=tk.W+tk.E+tk.S)
        b3 = tk.Button(buttons_frame, text='DISARM DRONE', font=("LM Mono 10", 15), width=30 ,bg='#ff6347', command=self.drone.request_disarm)
        b3.grid(row=3, column=1, columnspan = 3, sticky=tk.W+tk.E+tk.S)

    def update_values(self):
        self.voltageString.set("{:.2f}".format(float(self.drone.log.voltage)))
        self.looprateString.set("{:.2f}".format(float(self.drone.log.loop_rate)))
        self.positionString.set("x: {:.2f}, y: {:.2f}, z: {:.2f}".format(float(self.drone.position[0]),\
                                                                            float(self.drone.position[1]),\
                                                                            float(self.drone.position[2])))


class GroundStationGUI(tk.Frame):
    def __init__(self, names, vehicles, station, width=400, height=400, **options):
        # - - - - - - - - - - -
        # GUI params
        self.parent = tk.Tk()
        tk.Frame.__init__(self, self.parent, **options)
        self.parent.title("Ground Station")
        self.bg_color = '#808080'
        self.parent.configure(bg=self.bg_color)

        # - - - - - - - - - -
        # parameters
        self.names = names
        self.vehicles = vehicles
        self.N = len(vehicles)
        self.station = station

        # - - - - - - - - - - 
        # add menu
        self.add_menu()

        # - - - - - - - - - - 
        # add display widget
        self.widget_container = []
        self.qrotor_widgets = []
        for n in range(self.N):
            name = self.names[n]
            self.create_display_widget(n+1, name=name, drone=self.vehicles[n])
        
        # - - - - - - - - - - -
        # add safety features
        self.parent.bind("<Control_L><k>", self.kill_vehicles_key)
        self.parent.bind("<Control_R><k>", self.kill_vehicles_key)
        self.add_safety_switches()
        # __init__ end
        # - - - - - - - - - - - 

    def add_menu(self):
        self.menu = tk.Menu(self.parent)
        self.filemenu = tk.Menu(self.menu)
        self.menu.add_cascade(label="File", menu=self.filemenu)
        self.filemenu.add_command(label="About", command=self.about)
        self.filemenu.add_command(label="Reset", command=self.reset)
        self.filemenu.add_command(label="Quit", command=self.parent.destroy)
        self.parent.config(menu=self.menu)
        
    def about(self):
        """about this program"""
        print("opened about widget"  )
        self.about_widget = tk.Toplevel()  
        l1 = tk.Label(self.about_widget, text="Hi! I'm Prasanth Kotaru")
        l1.grid(row=1, column=1)
        l2 = tk.Label(self.about_widget, text="Welcome to qrotor ground station gui")
        l2.grid(row=2,column=1)
        self.about_widget.title("Ground Station GUI")
        self.filemenu.entryconfig(0,state=tk.DISABLED)
        b1 = tk.Button(self.about_widget,text="Ok!",fg='red',command=self.about_quit)
        b1.grid(row=3, column=1)

    def about_quit(self):
        """function to close the about widget"""
        self.filemenu.entryconfig(0,state=tk.NORMAL)
        self.about_widget.destroy()

    def reset(self):
        """function to reset the problem"""
        print ("reset pressed!"  )

    def add_safety_switches(self):
        danger_frame = tk.LabelFrame(self.parent, text="Danger",bg=self.bg_color, relief=tk.RIDGE)
        danger_frame.grid(row=3, column=1,  columnspan = self.N, padx=6, sticky=tk.E + tk.W + tk.N + tk.S)
        b1 = tk.Button(danger_frame, text='KILL ALL VEHICLES', font=("LM Mono 10", 15), width=100, height=2 ,bg='#FF5733', command=self.station.kill_all_vehicles)
        b1.grid(row=3, column=1, columnspan = self.N, sticky=tk.W+tk.E+tk.S)

    def kill_vehicles_key(self, event):
        self.station.kill_all_vehicles()

    def create_display_widget(self, col_ind=1, name="", drone=None):
        widget_frame = tk.LabelFrame(self.parent, text=name,bg=self.bg_color,relief=tk.RIDGE)
        widget_frame.grid(row=1, column=col_ind, sticky=tk.E + tk.W + tk.N + tk.S)
        qrotor_widget = QrotorDisplayWidget(widget_frame, drone)
        self.widget_container.append(widget_frame)
        self.qrotor_widgets.append(qrotor_widget)

    def run(self):
        for i in range(self.N):
            self.qrotor_widgets[i].update_values()
        try:
            self.parent.update_idletasks()
            self.parent.update()
            return True
        except tk.TclError:
            return False


if __name__=="__main__":
    gs = GroundStationGUI()
    while(True):
        gs.run()
        time.sleep(0.01)

