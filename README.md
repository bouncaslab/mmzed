Real Time Multi Vehicle Map Matching Implementation on Zedboard. 
It is tested on zedboard.
The design has been selected among the finalists in OpenHW 2016 design contest:
http://www.openhw.eu/2016-finalists

The implementation uses a named pipe, therefore it must be created using the following command:

mkfifo -m 666 /tmp/fifo1.dat

for info about map and car data: alp.malazgirt@boun.edu.tr V yurdakul@boun.edu.tr