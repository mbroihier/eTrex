# eTrex 

This repository contains python programs that read and capture eTrex tracks and waypoints.  Eventually it will also download routes.  The most recent change adds the capability to add waypoints to the device.

To run:
```

python3 eTrex.py

```
This will download eTrex tracks and waypoints and store them in a sqlite3 database.

```

python3 eTrex.py waypoint_file.txt

```
This will upload a waypoint to the device.

The contents of the waypoint file is expected to be the following format:

```

waypoint, NAME, 44.478228, -90.589265

```

where waypoint is a keyword identifying it as a waypoint file.
NAME is the name of the waypoint.
Then follows latitude and then longitude.
