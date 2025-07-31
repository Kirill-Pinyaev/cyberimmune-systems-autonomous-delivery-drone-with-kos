#!/bin/bash
#
# Check if ORVD shows movement of our UAV
#
HOST="${1:-"127.0.0.1"}"
PORT="8080"
# authentication
# example answer: 935d2528b5021242512ed188ad85be8f
TOKEN=$(curl -s "http://$HOST:$PORT/admin/auth?login=admin&password=passw")
OLDDATA=$(curl -s "http://$HOST:$PORT/admin/get_all_data?token=$TOKEN" |jq -c '.uav_data["52:58:00:12:34:bb"].telemetry.lon')

# will wait until uav is going to fly
sleep 10

COUNTER=0
while true; do
    DATA=$(curl -s "http://$HOST:$PORT/admin/get_all_data?token=$TOKEN" |jq -c '.uav_data["52:58:00:12:34:bb"].telemetry.lon')
    echo "Longitude: new $DATA == old $OLDDATA"
    if [ $DATA == $OLDDATA ]; then
        echo "YES - no movement";
        exit 1;
    fi
    echo "NO - it is moving";
    if [[ $COUNTER -gt 3 ]]; then
        break;
    fi
    OLDDATA=$DATA
    let COUNTER++
    sleep 3
done
