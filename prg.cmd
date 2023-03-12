log /dev/null
# rsettype 8
r
sleep 300
loadbin fw.bin 0x8000000
sleep 300
r
sleep 300
g
