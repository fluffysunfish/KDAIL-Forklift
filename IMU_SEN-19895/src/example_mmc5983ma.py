from MMCA5983MA_magnetometer.mmc5983ma import MMC5983MA

sensor = MMC5983MA()
if sensor.begin():
    print("MMC5983MA connected")
    sensor.set_filter_bandwidth(100)  # Set bandwidth to 100Hz
    temp = sensor.get_temperature()
    print(f"Temperature: {temp} C")
    x, y, z = sensor.get_measurement_xyz()
    print(f"X: {x}, Y: {y}, Z: {z}")
else:
    print("Failed to connect to MMC5983MA")