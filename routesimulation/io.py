def exportCSV(fileOut, simulation, delimiter = ";"):
	# Check if the simulation has stored frames
	simulationCheck(simulation)	
	with open(fileOut, "w") as f:
		columns = ["vehicle", "speed", "time", "longitude", "latitude"]
		header = "{d}".join(columns)
		print(header.format(d = delimiter), file = f)

		row = ["{" + x + "}" for x in columns]
		row = "{d}".join(row)
		for time in range(len(simulation.frames)):
			frame = simulation.frames[time]
			for vehicle, position in frame.items():
				print(row.format(vehicle = vehicle.vehicleId, speed = vehicle.speed, time = time, longitude = position.lon, latitude = position.lat, d = delimiter), file = f)

def simulationCheck(simulation):
	if not (simulation.storeFrames and len(simulation.frames) > 0):
		raise Exception("The simulation has no frames")

def exportJSON(fileOut, simulation):
	simulationCheck(simulation)
	with open(fileOut, "w") as f:
		frames = simulation.frames
		newFrames = []
		for frame in frames:
			newFrame = {}
			for vehicle, position in frame.items():
				key = vehicle.vehicleId
				newFrame[key] = position
			newFrames.append(newFrame)
		json.dump(newFrames, file = f)

import psycopg2
from datetime import datetime, timedelta
def exportToDB(dbname, simulation):
	simulationCheck(simulation)
	con = psycopg2.connect("dbname={db} user=postgres host=localhost".format(db=dbname))
	cur = con.cursor()
	today = datetime.now()
	for time in range(len(simulation.frames)):
		frame = simulation.frames[time]
		# Create timestamp from frame
		timeStamp = today + timedelta(seconds = time)
		for vehicle, position in frame.items():
			query = "INSERT INTO observations(id, lon, lat, time) VALUES ({id}, {lon}, {lat}, TIMESTAMP '{time}')".format(id = vehicle.vehicleId, lon = position.lon, lat = position.lat, time = timeStamp)
			cur.execute(query)
	con.commit()
	con.close()