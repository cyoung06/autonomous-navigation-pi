import numpy
import numpy as np

wifiLocsAndParams = numpy.empty((1,5))

with open('wifi_locations.csv', 'r') as f:
    lines = f.read().split("\n")
    for line in lines:
        if line == '':
            continue
        x,y,z,m,k = line.split(",")
        #xyzk
        x,y,z,m,k = float(x),float(y),float(z),float(m),float(k)
        wifiLocsAndParams = numpy.append(wifiLocsAndParams, [[x,y,z,m,k]], axis=0)

wifiLocsAndParams = wifiLocsAndParams[1:]


def calculateExpectedRSSIs(x, y, z):
    bruh = numpy.array([x, y, z])
    diff = bruh - wifiLocsAndParams[:, 0:3]
    sq = numpy.square(diff)
    sum = numpy.sum(sq, axis=1)
    distance = numpy.sqrt(sum)

    pre_rssi = numpy.log10(distance)
    pre_rssi /= wifiLocsAndParams[:, 4]  # divide by k
    pre_rssi -= wifiLocsAndParams[:, 3]  # subtract m
    return pre_rssi

width = 2
height = 50
wifiGrid = [[None] * width for i in range(height)]
with open('double_processed_wifi_data.csv', 'r') as f:
    lines = f.read().split("\n")
    for line in lines:
        if line == '':
            continue
        splitttttted = [float(f) for f in line.split(",")]
        x,y,z = splitttttted[:3]
        wifis = splitttttted[3:66]
        stds = splitttttted[66:129]

        wifiGrid[int(y / 1000)][int(x / 1000)] = (np.array(wifis) - calculateExpectedRSSIs(x,y,z), np.array(wifis), np.array(stds))

def getValueAt(pos, access, idx, default):
    x,y = int(pos[0]), int(pos[1])
    if x < 0 or y < 0 or x >= width or y >= height:
        return default
    if access[y][x] is None:
        return default
    return access[y][x][idx]

def interpolateIdxAt(pos, access, idx, mult):
    minY = int(pos[1] // 1000)
    minX = int(pos[0] // 1000)

    maxY = minY + 1
    maxX = minX + 1
    x, y = (pos[0], pos[1])
    # print(f'{minX} {minY} {maxX} {maxY} {minX < 0} {minY < 0} {maxX >= 3} {maxY >= 3}')

    dx1 = x - minX * 1000
    dy1 = y - minY * 1000
    dx2 = maxX * 1000 - x
    dy2 = maxY * 1000 - y
    val = getValueAt((minX, minY), access, idx, mult) * dx2 * dy2 + \
          getValueAt((maxX, minY), access, idx, mult) * dx1 * dy2 + \
          getValueAt((minX, maxY), access, idx, mult) * dx2 * dy1 + \
          getValueAt((maxX, maxY), access, idx, mult) * dx1 * dy1

    val /= 1000 * 1000
    return val

def interpolatedWifiValue(x,y):
    return interpolateIdxAt((x,y), wifiGrid, 1, numpy.ones(63) * 90)
def interpolatedWifiSTD(x, y):
    return interpolateIdxAt((x,y), wifiGrid, 2, numpy.ones(63) * 0.5)


print(interpolatedWifiValue(0, 0))

