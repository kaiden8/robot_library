from ultrasonicSensor import UltrasonicSensor


if __name__ == '__main__':

	sensor = UltrasonicSensor()
	sensor.importData()
	sensor.graphData()
	sensor.filterData()