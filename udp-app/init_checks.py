print "Basic pre-arm checks"
# Don't let the user try to arm until autopilot is ready
while not vehicle.is_armable:
    print " Waiting for vehicle to initialise..."
    time.sleep(1)


if v.mode.name == "INITIALISING":
    print "Waiting for vehicle to initialise"
    time.sleep(1)
while vehicle.gps_0.fix_type < 2:
    print "Waiting for GPS...:", vehicle.gps_0.fix_type
    time.sleep(1)

