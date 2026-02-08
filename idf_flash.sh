bash -lc '
source $IDF_PATH/export.sh

# Kill any process currently using the serial port
fuser -k /dev/ttyACM0 2>/dev/null || true

# Optional short delay so the port is released
sleep 1

idf.py -p /dev/ttyACM0 flash monitor
'
