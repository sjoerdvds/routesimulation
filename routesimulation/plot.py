from routesimulation.model import Simulation, Point

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
# implement the default mpl key bindings
from matplotlib.backend_bases import key_press_handler
import time

from matplotlib.figure import Figure
import tkinter as tk
import tkinter.ttk as ttk
import threading
import sys

# Test stuff
import random

class Controller:
	def __init__(self, simulation):
		
		self.simulation = simulation
		
		self.playing = False
		
		
		#tk.mainloop()
		self.view = View(self, simulation)
		self.view.start()

	def notify(self):
		# Respond to view state changes
		print("CTRL Notified")
		

	def playpause(self):
		self.playing = not self.playing
		self.view.updateThread.running = self.playing
		print("CTRL playing:" + str(self.playing))

	def quit(self):
		self.view.updateThread.running = False
		self.view.updateThread.stop = True

		self.view.root.quit()
		self.view.root.destroy()

	def updateProgress(self, i):
		self.view.updateProgress(i)

	def progressClicked(self, event):
		print("Clickies", event.x, event.x_root)
		widgetX = self.view.progress.winfo_rootx()
		relx = event.x_root -widgetX
		widgetWidth = self.view.progress.winfo_width()
		print(relx, widgetWidth)
		factor = self.simulation.runTime/widgetWidth
		newI = int(factor * relx)
		print(newI)
		#self.view.updateThread.running = False
		#self.playing = False
		self.view.updateThread.jumpI = newI
		self.view.updateThread.jump = True
		#self.view.updateThread.running = True
		#self.playing = True

	def jumpTo(self):
		self.view.updateThread.jumpI = int(self.view.entry.get())
		self.view.updateThread.jump = True
class View:
	
	def __init__(self, controller, simulation):
		self.controller = controller
		self.simulation = simulation
		self.i = 0
		#Tkinter init
		self.root = tk.Tk()
		self.frame = tk.Frame(self.root)
		self.root.title("Route Simulation")
		self.root.protocol("WM_DELETE_WINDOW", self.controller.quit)

		#Matplotlib init
		self.figure = Figure(figsize = (5,4), dpi=120)
		self.ax = self.figure.add_subplot(111)
		self.ax.set_ylim([-20,20])
		self.ax.set_xlim([-20,20])
		self.canvas = FigureCanvasTkAgg(self.figure, master=self.root)
		self.canvas.show()
		self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

		self.toolbar = NavigationToolbar2TkAgg( self.canvas, self.root)
		self.toolbar.update()
		self.canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=1)

		self.progress = ttk.Progressbar(orient="horizontal", length = 350, mode="determinate", maximum = simulation.runTime)
		self.progress["value"] = 0
		self.progress.bind("<Button-1>", lambda event: self.controller.progressClicked(event))

		
		self.playpause = tk.Button(master = self.root, text = "Play/Pause", command = self.controller.playpause)
		

		self.entry = tk.Entry(master= self.root)
		self.go = tk.Button(master= self.root, text="Jump To", command = self.controller.jumpTo)
		
		self.progress.pack(side = tk.TOP)
		self.entry.pack(side = tk.TOP)
		self.go.pack(side = tk.TOP)
		self.playpause.pack(side = tk.TOP)

		self.vehicles = set()
		for frame in self.simulation.frames:
			# Add all the vehicles to the plot
			for vehicle in frame.keys():
				self.vehicles.add(vehicle)

		self.lineTuples = {}
		for vehicle in self.vehicles:
			line, = self.ax.plot([],[], "o-")
			xdata, ydata = [], []
			# Put the
			tup = (line, [], [])
			self.lineTuples[vehicle] = tup
		
		

	def start(self):
		try:	
			self.updateThread = PlotUpdateThread(self.canvas, self.lineTuples, self.controller, self.simulation.frames)	
			tk.mainloop()
		except:
			print("Whoops")

	def updateProgress(self, i):
		self.progress["value"] = i

class PlotUpdateThread(threading.Thread):
	def __init__(self, canvas, lineTuples, controller, frames):
		self.canvas = canvas
		self.lineTuples = lineTuples
		self.controller = controller
		self.frames = frames
		self.running = False
		self.stop = False
		self.i = 0

		self.jump = False
		self.jumpI = None
		self.canvas.draw()
		threading.Thread.__init__(self)
		self.start()

	def run(self):
		while not self.stop:
			if self.jump:
				# Jump to the new frame
				# Set each vehicle to have all values up to and including this new frame
				i = 0
				lineDataX = {}
				lineDataY = {}
				print("Jumping to", self.jumpI)
				while i <= self.jumpI:
					# This frame should also be included in the xdata, ydata
					frame = self.frames[i]
					for vehicle in self.lineTuples.keys():
						if vehicle not in lineDataX:
							lineDataX[vehicle] = []
							lineDataY[vehicle] = []

						#newX, newY = 0,0

						if vehicle in frame:
							position = frame[vehicle]
							newX = position.lon
							newY = position.lat
							lineDataX[vehicle].append(newX)
							lineDataY[vehicle].append(newY)
					i+=1
				# Now all positions for each vehicle are calculated, we can plot each one:
				for vehicle in lineDataX.keys():
					xdata = lineDataX[vehicle]
					ydata = lineDataY[vehicle]
					tup = self.lineTuples[vehicle]

					line = tup[0]
					line.set_data(xdata, ydata)
					self.lineTuples[vehicle] = (line, xdata, ydata)
				self.i = self.jumpI
				self.jump = False
				self.canvas.draw()
				self.controller.updateProgress(self.i)
				print("Jumped")
			if self.running and self.i < len(self.frames):
				self.update(self.frames[self.i])
				self.i += 1
				self.controller.updateProgress(self.i)
				
				time.sleep(0.5)
				

	def update(self, frame):
		print("VIEW updating")
		for vehicle, position in frame.items():
			tup = self.lineTuples[vehicle]
			if tup is not None:
				line = tup[0]
				xdata = tup[1]
				ydata = tup[2]
				xdata.append(position.lon)
				ydata.append(position.lat)
				line.set_data(xdata, ydata)
				self.lineTuples[vehicle] = (line, xdata, ydata)

		try:
			self.canvas.draw()
			
		except:
			print("Whoops")
		self.controller.notify()

	def jumpTo(self, newI):
		if newI > self.i:
			#plot all intermediate points
			while self.i < newI:
				self.update(self.frames[self.i])
				self.i += 1
				self.controller.updateProgress(self.i)
		self.i = newI	
class TestSimulation():
	def __init__(self, vehicles = 5, runTime = 100):
		self.runTime = runTime
		self.frames = []
		self.vehicles = []
		for v in range(vehicles):
			self.vehicles.append(TestVehicle(v))

		for t in range(runTime):
			frame = {}
			for v in self.vehicles:
				v.update()
				frame[v] = v.position
			self.frames.append(frame)
	

class TestVehicle():
	def __init__(self, id):
		self.position = Point(random.randrange(0,10), random.randrange(0,10))
		self.id = id

	def __eq__(self, other):
		return isinstance(other, TestVehicle) and other.id == self.id

	def __hash__(self):
		return hash(self.id)

	def update(self):
		x = self.position.lon
		y = self.position.lat

		x += random.randrange(-5,5)
		y += random.randrange(-5,5)
		self.position = Point(x,y)