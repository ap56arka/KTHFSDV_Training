#######################     Written by Arka Pal     #######################
#######################     1st September, 2024     #######################
#######################     References              #######################
# https://medium.com/@lukasschaub/modern-graphical-user-interfaces-using-python-customtkinter-80f42b698eaf
# https://customtkinter.tomschimansky.com/tutorial/




import customtkinter
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure
import time
import csv

is_canvas_full = 0

#Class for the Visulaizer
class FuncViz():
    def __init__(self, func):
        self.func = func
        self.figure,self.ax = plt.subplots(figsize=(10, 6))
    
    #Data gathering method
    def gather_data(self,t_range=None, t_step = 0.001, single_point = None):

        if t_range is not None:
            self.t_range = t_range
            self.t_step = t_step
            self.t_arr = np.linspace(t_range[0], t_range[1], int((t_range[1] - t_range[0])/t_step))
            self.h_arr = self.func(self.t_arr)
            self.h_min = min(self.h_arr)
            self.h_max = max(self.h_arr)
        elif single_point is not None:
            self.t_arr = np.array([single_point])
            self.h_arr = self.func(self.t_arr)
        
    #Plot Method
    def plot_fun(self,t_range=None, t_step = 0.001, single_point = None, live = None):
        if t_range is not None:
            self.gather_data(t_range)
            self.zoom_xaxis(self.t_range[0], self.t_range[1])
            self.zoom_yaxis(self.h_min, self.h_max)
            self.line, = self.ax.plot(self.t_arr, self.h_arr,color='blue')
            self.ax.set_xlabel('t')
            self.ax.set_ylabel('h(t)')
            self.ax.set_title('Static Visualization of h(t)')
    
        elif live is not None:
            self.gather_data(single_point=single_point)
            self.line = self.ax.plot(self.t_arr, self.h_arr,color='blue',lw = 2)
            self.ax.set_xlabel('t')
            self.ax.set_ylabel('h(t)')
            self.ax.set_title('Live visualization of h(t)')
            self.zoom_xaxis(int(single_point) - self.time_window, int(single_point))

    def show(self):
        plt.show()
    
    #Handle Grid on and off
    def grid_on(self):
        self.ax.grid('True')
    def grid_off(self,cur_range_x, cur_range_y):
        self.figure,self.ax = plt.subplots(figsize=(10, 6))
        self.plot_fun(self.t_range)
        self.zoom_xaxis(cur_range_x[0], cur_range_x[1])
        self.zoom_yaxis(cur_range_y[0], cur_range_y[1])

    #Handle zoom in X axis and Y axis
    def zoom_xaxis(self,xstart, xend):
        self.ax.set_xlim(xstart, xend)
        
    def zoom_yaxis(self,ystart, yend):
        self.ax.set_ylim(ystart, yend)
        
# Inherited from the FuncViz class, for live visualization
class FuncVizLive(FuncViz):
    def __init__(self,func, widgframe):
        super().__init__(func)
        self.widgframe = widgframe
        self.x_data, self.y_data = [], []
        self.line, = self.ax.plot([], [], lw=2)
        self.ax.set_xlabel('t')
        self.ax.set_ylabel('h(t)')
        self.ax.set_title('Live Visualization of h(t)')
        self.time_window = 10  
        self.ax.set_ylim(-1.5, 1.5)
        self.start_time = None
        self.running = False

    #Handle the start button  
    def start(self):
        global is_canvas_full
        self.x_data.clear()
        self.y_data.clear()
        self.line.set_data([], [])
        self.ax.set_xlim(0, self.time_window)
        self.widgframe.update_canvas_live()
        if is_canvas_full==0:
            self.widgframe.countPress_live+=1
        
            self.widgframe.canvas.get_tk_widget().place(relx=0.2, rely=0.025)
        else:
            self.widgframe.canvas = FigureCanvasTkAgg(self.widgframe.f_viz_obj_liv.figure,master=self.widgframe.master)
            self.widgframe.canvas.get_tk_widget().place(relx=0.2, rely=0.025)
            is_canvas_full = 0
        self.start_time = time.time()
        self.running = True
        self.update_plot()
    #Handle tne stop button    
    def stop(self):
        self.running = False

    def update_plot(self):
        if not self.running:
            return self.line,

        
        t = time.time() - self.start_time
        self.x_data.append(t)
        self.y_data.append(self.func(t))
        self.ax.set_xlim(t - self.time_window, t)
        self.ax.set_ylim(0,1400)
        self.line.set_data(self.x_data, self.y_data)
        self.widgframe.update_canvas_live()
        self.widgframe.canvas.get_tk_widget().after(10, self.update_plot)





#Frame created for the side bar
class WidgetsFrame(customtkinter.CTkFrame):
    def __init__(self, master):
        super().__init__(master)
        self.countPress = 0
        self.countPress_live = 0
        self.usr_start_time = 0
        self.usr_stop_time = 10
        self.x_zoom_start = self.usr_start_time
        self.x_zoom_stop = self.usr_stop_time
        self.y_zoom_start = 0
        self.y_zoom_stop = 1400
        self.grid_gui_display = "Grid on"
        #Entry for start and stop time from user
        self.start_entry = customtkinter.CTkEntry(self, placeholder_text="Start time",width=75)
        self.start_entry.grid(row=0, column=0, padx=10, pady=(5,20), sticky="w")
        self.stop_entry = customtkinter.CTkEntry(self, placeholder_text="End time",width=75)
        self.stop_entry.grid(row=0, column=1, padx=1, pady=(5,20), sticky="w")
        #Button for Plot Static
        self.showbutton = customtkinter.CTkButton(self, text= "Plot static", command=lambda: self.show_button_callback())
        self.showbutton.grid(row=1, column=0, padx=20, pady=1, sticky="w", columnspan=2)
        #Button for Grid
        self.button = customtkinter.CTkButton(self, text= "Grid on", command=lambda: self.grid_button_callback())
        self.button.grid(row=2, column=0, padx=20, pady=10, sticky="w", columnspan=2)

        
        

        #X zoom parameters
        self.center_var = customtkinter.DoubleVar(value=self.usr_start_time)
        self.text_label_zoom_cen = customtkinter.CTkLabel(self, text=f"Zoom Center in X: {self.center_var.get():.2f}", font=("Arial", 15))
        self.text_label_zoom_cen.grid(row=3, column=0, padx=1, pady=(5,1), sticky="we", columnspan=2)

        
        self.zoom_center_slider = customtkinter.CTkSlider(self,width=150,height=20,from_=self.usr_start_time,to=self.usr_stop_time,
                                    number_of_steps= 101,variable=self.center_var,command=self.zoom_center_slider_callback)
        
        self.zoom_center_slider.grid(row=4, column=0, padx=20, pady=1, sticky="w", columnspan=2)
        

        self.text_label_zoom = customtkinter.CTkLabel(self, text="Zoom Level in X", font=("Arial", 15))
        self.text_label_zoom.grid(row=5, column=0, padx=1, pady=(5,1), sticky="we", columnspan=2)
        self.zoom_slider = customtkinter.CTkSlider(self,width=150,height=20,from_=1,to=100,
                                    number_of_steps=99,
                                    command=self.zoom_slider_callback)
        self.zoom_slider.grid(row=6, column=0, padx=20, pady=1, sticky="w", columnspan=2)
        self.zoom_slider.set(0)


        
        #Y zoom parameters
        self.center_var_y = customtkinter.DoubleVar(value=0)
        self.text_label_zoom_cen_y = customtkinter.CTkLabel(self, text=f"Zoom Center in Y: {self.center_var_y.get():.2f}", font=("Arial", 15))
        self.text_label_zoom_cen_y.grid(row=7, column=0, padx=1, pady=(5,1), sticky="we", columnspan=2)

        
        self.zoom_center_slider_y = customtkinter.CTkSlider(self,width=150,height=20,from_=0,to=1400,
                                    number_of_steps= 101,variable=self.center_var_y,command=self.zoom_center_slider_callback_y)
        
        self.zoom_center_slider_y.grid(row=8, column=0, padx=20, pady=1, sticky="w", columnspan=2)
        

        self.text_label_zoom_y = customtkinter.CTkLabel(self, text="Zoom Level in Y", font=("Arial", 15))
        self.text_label_zoom_y.grid(row=9, column=0, padx=1, pady=(5,1), sticky="we", columnspan=2)
        self.zoom_slider_y = customtkinter.CTkSlider(self,width=150,height=20,from_=1,to=100,
                                    number_of_steps=99,
                                    command=self.zoom_slider_callback_y)
        self.zoom_slider_y.grid(row=10, column=0, padx=20, pady=1, sticky="w", columnspan=2)
        self.zoom_slider_y.set(0)
        #Button for fitting the plot
        self.fitplotbutton = customtkinter.CTkButton(self, text= "Fit Plot", command=lambda: self.fitbutton_button_callback())
        self.fitplotbutton.grid(row=11, column=0, padx=20, pady=10, sticky="w", columnspan=2)



        self.f_viz_obj_liv = FuncVizLive(self.master.get_usr_func, self)
        #Start button
        self.startlivebutton = customtkinter.CTkButton(self, text= "Start Live", command= self.f_viz_obj_liv.start,width=80, height=30)
        self.startlivebutton.grid(row=12, column=0, padx=1, pady=10, sticky="w", columnspan=1)
        #Stop button
        self.stop_button = customtkinter.CTkButton(self, text="Stop Live", command=self.f_viz_obj_liv.stop,width=80, height=30)
        self.stop_button.grid(row=12, column=1, padx=1, pady=10, sticky="w", columnspan=1)
        self.canvas = FigureCanvasTkAgg(self.f_viz_obj_liv.figure,master=self.master)

        #File name entry from user
        self.filename_entry = customtkinter.CTkEntry(self, placeholder_text="Exp Name", width = 75)
        self.filename_entry.grid(row=13, column = 0, padx = 10, pady = 5, sticky = "w")

        self.export_button = customtkinter.CTkButton(self, text="Export Data", command=self.export_data_callback, width=80, height=30)
        self.export_button.grid(row = 13, column=1, padx=1, pady=10, sticky="w", columnspan=1)
        
    def start_live_button_callback(self):
        pass
        
    def export_data_callback(self):
        filename = self.filename_entry.get()
        if not filename.endswith('.csv'):
            filename += '.csv'
        with open(filename, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Time (s)", "h(t)"])
            writer.writerows(zip(self.f_viz_obj_liv.x_data, self.f_viz_obj_liv.y_data))
        
    def update_canvas_live(self):
        
        self.canvas.draw()
        


    def fitbutton_button_callback(self):
        self.f_viz_obj.gather_data((0,10))
        period_idx = [i for i, h_ in enumerate(self.f_viz_obj.h_arr) if h_ < self.f_viz_obj.h_min+0.000001 and h_ > self.f_viz_obj.h_min - 0.000001]
        t_period = self.f_viz_obj.t_arr[period_idx[1]] - self.f_viz_obj.t_arr[period_idx[0]]
        print("Time Period: "+str(t_period))
        self.f_viz_obj.plot_fun((self.usr_start_time,self.usr_stop_time))
        self.f_viz_obj.ax.set_xlim(self.usr_start_time,self.usr_start_time+t_period)
        self.f_viz_obj.ax.set_ylim(self.f_viz_obj.h_min, self.f_viz_obj.h_max)
        self.x_zoom_start = self.usr_start_time
        self.x_zoom_stop = self.usr_start_time+t_period
        self.y_zoom_start = self.f_viz_obj.h_min
        self.y_zoom_stop = self.f_viz_obj.h_max
        self.reset_all_sliders()
        self.update_canvas()

    def reset_all_sliders(self):
        self.zoom_center_slider.set(self.usr_start_time)
        self.zoom_slider.set(0)
        self.zoom_center_slider_y.set(0)
        self.zoom_slider_y.set(0)
        self.text_label_zoom_cen.configure(text = "Zoom Center in X: 0.00")
        self.text_label_zoom.configure(text = "Zoom Level in X")
        self.text_label_zoom_cen_y.configure(text = "Zoom Center in Y: 0.00")
        self.text_label_zoom_y.configure(text = "Zoom Level in Y")

    def zoom_slider_callback(self,val):
        v = int(val)
        c = self.center_var.get()

        
        a = self.usr_start_time
        b = self.usr_stop_time
        self.f_viz_obj.zoom_xaxis(a+((c-a-0.1)*v)/100, b - (v/100)*(b-c-0.1))
        self.x_zoom_start = a+((c-a-0.1)*v)/100
        self.x_zoom_stop = b - (v/100)*(b-c-0.1)
        self.update_canvas()

    def zoom_center_slider_callback(self, val):
        
        self.text_label_zoom_cen.configure(text=f"Zoom Center in X: {self.center_var.get():.2f}")
        self.zoom_slider_callback(self.zoom_slider.get())




    def zoom_slider_callback_y(self,val):
        v = int(val)
        c = self.center_var_y.get()
        
        
        a = 0
        b = 1400
        
        
        self.f_viz_obj.zoom_yaxis(a+((c-a-10)*v)/100, b - (v/100)*(b-c-10))
        self.y_zoom_start = a+((c-a-10)*v)/100
        self.y_zoom_stop = b - (v/100)*(b-c-10)
        self.update_canvas()

    def zoom_center_slider_callback_y(self, val):
        
        self.text_label_zoom_cen_y.configure(text=f"Zoom Center in Y: {self.center_var_y.get():.2f}")
        self.zoom_slider_callback_y(self.zoom_slider_y.get())



      
    def get_start_entry(self):
        if(self.start_entry.get()==''):
            self.usr_start_time = 0
            
        else:
            self.usr_start_time = float(self.start_entry.get())
            self.x_zoom_start = self.usr_start_time
            self.center_var = customtkinter.DoubleVar(value=self.usr_start_time)
            self.zoom_center_slider = customtkinter.CTkSlider(self,width=150,height=20,from_=self.usr_start_time,to=self.usr_stop_time,
                                   number_of_steps= 101,variable=self.center_var,command=self.zoom_center_slider_callback)
        
            self.zoom_center_slider.grid(row=4, column=0, padx=20, pady=1, sticky="w", columnspan=2)
       

    def get_stop_entry(self):
        if(self.stop_entry.get()==''):
            self.usr_stop_time = 10
        else:
            self.usr_stop_time = float(self.stop_entry.get())
            self.x_zoom_stop = self.usr_stop_time
            self.center_var = customtkinter.DoubleVar(value=self.usr_start_time)
            self.zoom_center_slider = customtkinter.CTkSlider(self,width=150,height=20,from_=self.usr_start_time,to=self.usr_stop_time,
                                    number_of_steps= 101,variable=self.center_var,command=self.zoom_center_slider_callback)
        
            self.zoom_center_slider.grid(row=4, column=0, padx=20, pady=1, sticky="w", columnspan=2)
        


    def grid_button_callback(self):
        self.countPress+=1
        if(self.countPress%2==1):
            self.button.configure(text = "Grid off")
            self.f_viz_obj.grid_on()
            self.update_canvas()
            
        else:
            self.button.configure(text = "Grid on")
            
            self.f_viz_obj.grid_off((self.x_zoom_start, self.x_zoom_stop),(self.y_zoom_start,self.y_zoom_stop))
            self.update_canvas()
            

        
    def update_canvas(self):
        canvas = FigureCanvasTkAgg(self.f_viz_obj.figure,master=self.master)
        global is_canvas_full
        is_canvas_full = 1
        canvas.draw()
        canvas.get_tk_widget().place(relx=0.2, rely=0.025)

    
    def show_button_callback(self):
        self.button.configure(text = "Grid on")
        self.get_start_entry()
        self.get_stop_entry()
        self.f_viz_obj = FuncViz(self.master.get_usr_func)
        self.f_viz_obj.plot_fun((self.usr_start_time,self.usr_stop_time))
        self.reset_all_sliders()
        self.update_canvas()

    
#The class for the GUI
class PlotterApp(customtkinter.CTk):
    def __init__(self):
        super().__init__()

        self.title("Plotter GUI: KTHFSDV")
        self.geometry("1050x500")
        
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)
        self.widgets_frame = WidgetsFrame(self)
        self.widgets_frame.grid(row=0, column=0, padx=10, pady=(10, 0), sticky="nsw")


        
    def get_usr_func(self,t):
        lambda_t = 5 * np.sin(2 * np.pi * 1 * t)
        return 3 * np.pi * np.exp(-lambda_t)

        
    
def on_closing():
    
    print("Cleaning up before exit...")
    app.quit()  
    app.destroy()  


app = PlotterApp()
app.protocol("WM_DELETE_WINDOW", on_closing)
app.mainloop()