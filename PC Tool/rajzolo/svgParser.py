# from matplotlib import pyplot
import time
import serial

# x: 050..150
# y: 050..210

enableDrawing = False
enableSerial = True

if enableDrawing:
    from matplotlib import pyplot


if enableSerial:
    ser = serial.Serial(
        port='/dev/ttyACM0',
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )

    ser.close()
    ser.open()
    ser.isOpen()

def sendCommand(command):
    global ser
    ser.write(b"%s\r\n" % command.encode('ascii','ignore'))
    time.sleep(1)



fhSVG = open("drawing.svg")
rgstrSVGContent = fhSVG.readlines()
fhSVG.close()

strCurves = []

bCurveStart = False
for strLine in rgstrSVGContent:
    if "<path" in strLine:
        bCurveStart = True
    if bCurveStart:
        if 'd="' in strLine:
            print(strLine[10:])
            strCurves.append(strLine[10:-2])
            bCurveStart = False
        
print(strCurves)

rgcAllCurves = []
bFirstCoordinate = True
startCoordinate = [0,0]
curveStartCoordinate = []

minX = 0
maxX = 0
minY = 0
maxY = 0

useAbsolutePath = False

for strCurve in strCurves:
    rgcCurveCoordinates = []
    absCoordinate = True
    for strCoord in strCurve.split(" "):
        if "z" in strCoord:
            rgcCurveCoordinates.append(curveStartCoordinate[:])
            continue
            
        if "M" in strCoord:
            useAbsolutePath = True
            continue
            
    
        rgCurveCoordinate = strCoord.split(",")
        rgCurveCoordinate[0] = float(rgCurveCoordinate[0])
        rgCurveCoordinate[1] = float(rgCurveCoordinate[1])
        
        if absCoordinate:
            if not bFirstCoordinate:
                rgCurveCoordinate[1] = startCoordinate[1] - (rgCurveCoordinate[1] - startCoordinate[1])
        else:
            if not useAbsolutePath:
                rgCurveCoordinate[0] = lastCoordinate[0] + rgCurveCoordinate[0]
                rgCurveCoordinate[1] = lastCoordinate[1] - rgCurveCoordinate[1]
        
        if rgCurveCoordinate[0] > maxX: maxX = rgCurveCoordinate[0]
        if rgCurveCoordinate[0] < minX: minX = rgCurveCoordinate[0]
        if rgCurveCoordinate[1] > maxY: maxY = rgCurveCoordinate[1]
        if rgCurveCoordinate[1] < minY: minY = rgCurveCoordinate[1]
        
        if bFirstCoordinate:
            startCoordinate = rgCurveCoordinate
            bFirstCoordinate = False
            minX = startCoordinate[0]
            maxX = startCoordinate[0]
            minY = startCoordinate[1]
            maxY = startCoordinate[1]
            
        if absCoordinate:
            absCoordinate = False
            curveStartCoordinate = rgCurveCoordinate
        rgcCurveCoordinates.append(rgCurveCoordinate)
        lastCoordinate = rgCurveCoordinate
    rgcAllCurves.append(rgcCurveCoordinates)
    
print(rgcAllCurves)
print(startCoordinate)

print(minX)
print(maxX)
print(minY)
print(maxY)

expectedXMin = 70
expectedXMax = 130
expectedYMin = 50
expectedYMax = 190

xExpectedRange = expectedXMax-expectedXMin
yExpectedRange = expectedYMax-expectedYMin

xRange = maxX - minX
yRange = maxY - minY

if xRange >= yRange:
    convertRange = xRange
else:
    convertRange = yRange

if xExpectedRange <= yExpectedRange:
    convertExpRange = xExpectedRange
else:
    convertExpRange = yExpectedRange

newYMax = 0
for rgCurve in rgcAllCurves:
    for coordinates in rgCurve:
        coordinates[0] = ((coordinates[0] - minX) / convertRange * convertExpRange)+expectedXMin
        coordinates[1] = ((coordinates[1] - minY) / convertRange * convertExpRange*2)
        if coordinates[1] > newYMax: newYMax = coordinates[1]

for rgCurve in rgcAllCurves:
    for coordinates in rgCurve:
        coordinates[1] += expectedYMax-newYMax
        
for rgCurve in rgcAllCurves:
    for coordinates in rgCurve:
        print(coordinates)
        if enableDrawing:
            pyplot.plot(coordinates[0], coordinates[1], "rD")

if enableDrawing:      
    pyplot.show()

if enableSerial:
    startCommand = "P%03d;%03d;%03d" % (rgcAllCurves[0][0][0],100,rgcAllCurves[0][0][1])
    sendCommand(startCommand)

    for rgCurve in rgcAllCurves:
        command = "P%03d;%03d;%03d" % (rgCurve[0][0],100,rgCurve[0][1])
        sendCommand(command)
        sendCommand("PEN1")
        
        for coordinates in rgCurve:
            command = "P%03d;%03d;%03d" % (coordinates[0],100,coordinates[1])
            sendCommand(command)
        
        sendCommand("PEN0")
        

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    