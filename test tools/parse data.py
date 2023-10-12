import serial
import time
import csv
ser = serial.Serial('/dev/cu.usbserial-1', 115200)
#ser = serial.Serial('/dev/cu.usbserial-DT03KIUS', 115200)
stat = bytes("@ GPS_STAT", encoding="latin-1")

#start_time=time.time()
#while (time.time() - start_time < 30):
while True:
    #with open("FeatherweightPackets","rb") as d:
        #data=d.readline()
    data = ser.readline()
  
    
    #if the serial data starts with "@ GPS_STAT", write the line to a new FilteredData file
    if (data.startswith(stat)):
        with open("FilteredData", "a") as fd:
            fd.write(data.decode('latin-1'))
        #read from the filtered data and write the relevant data to a CSV file
        with open("FilteredData", "r") as fd2:
            line = fd2.readlines()
            last_line = line[-1]
            timeindex = last_line.index(':')
            altindex = last_line.index('Alt')
            latindex = last_line.index('+')
            longindex = last_line.index('ln')
            velindex = last_line.index('Vel')

            time = last_line[timeindex - 2: timeindex + 9]
            alt = last_line[altindex + 4: latindex - 4]
            lat = last_line[latindex : longindex - 1]
            long = last_line[longindex + 3: velindex - 1]
            hvel = last_line[velindex + 4: velindex + 9]
            hhead = last_line[velindex + 10:velindex + 14]
            uvel = last_line[velindex + 15: velindex + 20]
            print("Alt",alt,"Lat", lat,"Long", long,"Hvel", hvel,"Hhead", hhead,"Uvel", uvel)
            #creates a csv file for the parsed data
            with open('data.csv', mode='a') as csvfile:
                #sets the column headers
                fieldnames = ['Time', 'Altitude', 'Latitude', 'Longitude', 'Horizontal Velocity', 'Horizontal Heading', 'Upward Velocity']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                #writer.writeheader()

                #writes the rows with the relevant data
                writer.writerow({'Time':time, 'Altitude':alt, 'Latitude':lat, 'Longitude':long, 'Horizontal Velocity':hvel, 'Horizontal Heading':hhead, 'Upward Velocity':uvel})