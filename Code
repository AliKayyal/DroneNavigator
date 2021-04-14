from numpy import math
import numpy as np
from DroneClient import DroneClient
import time
import airsim.utils

#those values should be tuned,
# maplength is the size of the map expected
#mapx is the beginning of the map (in negative units, i presume its negative)

file = r"C:\Users\Hp\Desktop\map\map.txt"
maplength = 1000
map = np.zeros((maplength,maplength),dtype=int)

mapx=200
mapy=200
height= -70

#filling the obstacles in the map, makes a 7X7 square around the found point of the obstacle
def writeToMap(x,y):
    if(-(mapx + maplength)<x<-mapx and -(mapx + maplength)<y<-mapx):
        xx = round(-x) -mapx
        yy = round(-y)-mapx
        for i in range(7):
            for j in range(7):
                map[xx + i -3][yy+j-3]=1


#print the map to the text file "file", not essential
def printMap():
    strr=""
    for i in range (maplength):
        for j in range(maplength):
            strr+=str(map[i][j])
        strr+="\n"
    fill = open(r"C:\Users\Hp\Desktop\robotics\map\map.txt", "w")
    fill.write(strr)
    fill.close()



def zturn():           #sin and cos of the turning matrix
    pos=client.getPose()
    s = math.sin(pos.orientation.z_rad)
    c = math.cos(pos.orientation.z_rad)
    return [c,s]

def zturnangle(a):           #sin and cos of the turning matrix of angle a
    print("ztunangle")
    s = math.sin(a)
    c = math.cos(a)
    print([c,s])
    return [c,s]

#fill the course of the drone in the map, not essential.
# 5 the course in a free tile, 7 if it has an obstacle on it
def myCourse():
    cur = client.getPose().pos
    xx = round(-cur.x_m) - mapx
    yy = round(-cur.y_m) - mapx
    if (map[xx][yy] != 1):
        map[xx][yy] = 5
    else:
        map[xx][yy] = 7
#fill the point of a chosen point in the map with 4, not essential
def testpoint(cur):
    xx = round(-cur[0]) - mapx
    yy = round(-cur[1]) - mapx
    map[xx][yy] = 4

#same as above, for test purposes, not essential
def myCoursepoint(cur):

    xx = round(-cur[0]) - mapx
    yy = round(-cur[1]) - mapx
    if(map[xx][yy]!=1 and map[xx][yy]!=7 ):
        map[xx][yy] = 5
    else:
        map[xx][yy] = 7
#same as above, for test purposes, not essential
def myCoursepointd(curr):
    xx = round(-curr[0]) - mapx
    yy = round(-curr[1]) - mapx
    if (map[xx][yy] != 1) and (map[xx][yy] != 7):
        map[xx][yy] = 9

#find closest point with the LIDAR, put it as an obstacle in the map,
# if no obstacle found or found too low on the z axis - dont fill the map
def scan():
        pos=client.getPose().pos
        c = zturn()
        lidr = client.getLidarData().points
        if len(lidr) == 3 :
            if(abs(lidr[1])>=0.1 and lidr[0]>=0.1 and abs(lidr[2])<=0.1):
                y = pos.y_m + lidr[0] * c[1] + lidr[1] * c[0]
                x = pos.x_m - lidr[1] * c[1] + lidr[0] * c[0]
                writeToMap(x,y)



#return True if the current point is an obstacle or the end point we search for
#else return False
def curcheck(cur,end):
    # print(str(cur))
    xx = -(round(cur[0]) + mapx)
    yy = -(round(cur[1]) + mapx)

    return (end[0]-1<=cur[0]<=end[0]+1 and end[1]-1<=cur[1]<=end[1]+1) or map[xx][yy]==1 or map[xx][yy]==7

#search the furthest point from point 'curr' to point 'end' that has no obstacles,
# fill the course with points we found that have not obstacles
#return the point of the obstacle in the way to 'end' or 'end'
def furthestPoint(curr,end,course):
    endpoint = [round(end[0]),round(end[1])]
    currpoint = [curr[0],curr[1]]
    xbiger=False
    if endpoint[0]==currpoint[0]:
        m=1000
    else:
        m = abs((endpoint[1] - currpoint[1])/(endpoint[0]-currpoint[0]))
    if 0<m<1:
        m=1/m
        xbiger=True
    counter = 0
    if(endpoint[0]==currpoint[0]):
        x=0
    else:
        x= (endpoint[0]-currpoint[0])/abs(endpoint[0]-currpoint[0])
    if(m!=0):
        y= (endpoint[1]-currpoint[1])/abs(endpoint[1]-currpoint[1])
    else:
        y=0
    while not curcheck(currpoint, endpoint):
        if xbiger:
            while counter < m:
                counter = counter + 1
                currpoint[0] += x

                if curcheck(currpoint, endpoint):

                    return currpoint
                course.append([currpoint[0], currpoint[1]])
            currpoint[1] += y

            if curcheck(currpoint, endpoint):
                return currpoint
            course.append([currpoint[0], currpoint[1]])
            counter = counter - m
        else:  ##didntchange
            while counter < m:
                counter = counter + 1
                currpoint[1] += y

                if curcheck(currpoint, endpoint):
                    return currpoint
                course.append([currpoint[0], currpoint[1]])
            currpoint[0] += x

            if curcheck(currpoint, endpoint):
                return currpoint
            course.append([currpoint[0], currpoint[1]])
            counter = counter - m
    return currpoint

#fill the map with the course in '9's, not essential
def printcourse(course):
    for i in course:
        myCoursepointd(i)


#return True if there is an obstacle in map[x,y]
def isblocked(x,y):

    return map[round(-x-mapx)][round(-y-mapx)]==1 or map[round(-x-mapx)][round(-y-mapx)]==7

#return the closest free point on the right side of the obstacle
#if there is a bend in the obstacle - add the bend to course
def keepleft(point,course):

    #xy corner

    if isblocked(point[0] +1, point[1]) and isblocked(point[0]+1, point[1]+1) and(isblocked(point[0], point[1]+1)
                                                                                   and not isblocked(point[0] - 1, point[1])):
        return  keepleft([point[0]-1, point[1] +1],course)

        # -xy corner

    if isblocked(point[0], point[1] + 1) and isblocked(point[0] - 1, point[1] + 1) and (
            isblocked(point[0], point[1] - 1)
            and not isblocked(point[0], point[1] - 1)):
        return keepleft([point[0] - 1, point[1] - 1], course)


        # x-y corner

    if isblocked(point[0]+1, point[1] ) and isblocked(point[0] + 1, point[1] - 1) and (
            isblocked(point[0], point[1] - 1)
            and not isblocked(point[0] , point[1] + 1)):

        return keepleft([point[0] + 1, point[1] + 1], course)
        # -x-y corner

    if isblocked(point[0]-1, point[1]) and isblocked(point[0] -1, point[1] - 1) and (
            isblocked(point[0], point[1] - 1)
            and not isblocked(point[0] + 1, point[1] )):

        return keepleft([point[0] + 1, point[1] - 1], course)


    #blocked from x
    if (isblocked(point[0]+1,point[1]) and isblocked(point[0]+2,point[1]) and isblocked(point[0]+3,point[1])):
        if isblocked(point[0]+1,point[1]+1):
            if(isblocked(point[0],point[1]+1)):
                if(isblocked(point[0]-1,point[1]+1)):
                    return keepleft([point[0]-2,point[1]+1],course)
                return [point[0]-1, point[1] + 1]
            return [point[0],point[1]+1]
        return [point[0]+1, point[1] + 1]
    #blocked from -x
    if (isblocked(point[0]-1,point[1]) and isblocked(point[0]-2,point[1]) and isblocked(point[0]-3,point[1])):
        if isblocked(point[0]-1,point[1]-1):
            if(isblocked(point[0],point[1]-1)):
                if(isblocked(point[0]+1,point[1]-1)):
                    return keepleft([point[0]+2,point[1]-1],course)
                return [point[0]+1, point[1] - 1]
            return [point[0],point[1]-1]
        return [point[0]-1, point[1] -1]
    #blocked from -y
    if (isblocked(point[0],point[1]-1) and isblocked(point[0],point[1]-2) and isblocked(point[0],point[1]-3)):
        if isblocked(point[0]+1,point[1]-1):
            if(isblocked(point[0]+1,point[1])):
                if(isblocked(point[0]+1,point[1]+1)):
                    return keepleft([point[0]+1,point[1]+2],course)
                return [point[0]+1, point[1] +1]
            return [point[0]+1,point[1]]
        return [point[0]+1, point[1] - 1]
    #blocked from y
    if (isblocked(point[0],point[1]+1) and isblocked(point[0],point[1]+2) and isblocked(point[0],point[1]+3)):
        if isblocked(point[0]-1,point[1]+1):
            if(isblocked(point[0]-1,point[1])):
                if(isblocked(point[0]-1,point[1]-1)):
                    return keepleft([point[0]-1,point[1]-2],course)
                return [point[0]-1, point[1] -1]
            return [point[0]-1,point[1]]
        return [point[0]-1, point[1] + 1]
    #edge xy
    if isblocked(point[0] , point[1]-1) and isblocked(point[0], point[1]-2) and(isblocked(point[0], point[1]-3)
                                                                                   and not isblocked(point[0] + 1, point[1] - 2)):
        course.append([point[0]+1, point[1]])
        return  [point[0]+1, point[1] -1]
    #edge to  -coursexy
    if isblocked(point[0] + 1, point[1]) and isblocked(point[0] + 2, point[1]) and(isblocked(point[0] + 3, point[1])
                                                                                   and not isblocked(point[0] + 2, point[1] + 1)):
        course.append([point[0], point[1] + 1])
        return  [point[0]+1, point[1] + 1]
    #edge x-y
    if isblocked(point[0] - 1, point[1]) and isblocked(point[0] - 2, point[1]) and(isblocked(point[0] - 3, point[1])
                                                                                   and not isblocked(point[0] - 2, point[1] - 1)):
        course.append([point[0], point[1] - 1])
        return  [point[0]-1, point[1] - 1]
    #edge -x-y
    if isblocked(point[0] , point[1]+1) and isblocked(point[0], point[1]+2) and(isblocked(point[0], point[1]+3)
                                                                                   and not isblocked(point[0] - 1, point[1] + 2)):
        course.append([point[0]-1, point[1]])
        return  [point[0]-1, point[1] +1]
    return point

#return True if there is a tile in course that is currently on an obstacle, else return False
def courseblocked(course,blocker):
    if not course:
        return False
    prev=[course[0][0],course[0][1]]
    for currCourse in course:
        if isblocked(currCourse[0], currCourse[1]):
            blocker.clear()
            blocker.append( [prev[0],prev[1]])
            print("blocking:" + str(currCourse))
            return True
        prev = currCourse
    return False

#return True if the drone is currently at a point that is surrounded by obstacle tiles, else False
def isStuck():
    cur = client.getPose().pos
    if isblocked(cur.x_m,cur.y_m+1) and isblocked(cur.x_m,cur.y_m-1) and isblocked(cur.x_m+1,cur.y_m) and isblocked(cur.x_m-1,cur.y_m):
        writeToMap(cur.x_m,cur.y_m)
        return True
    return False

#return True if course is empty or the drone is close to the point 'cur'
def curIsHere(cur,course):
    curr = client.getPose().pos
    if (not course) or  cur[0] - 2 <= curr.x_m <= cur[0] + 2  and cur[1] - 2 <= curr.y_m <= cur[1] + 2:
        while course and cur[0] - 2 <= curr.x_m <= cur[0] + 2  and cur[1] - 2 <= curr.y_m <= cur[1] + 2:
            cur = course.pop(0)
        return True
    return False

#add the obstacles the drone picks with the LIDAR to the map for a full second
def fullscan():
    for i in range(20):
        scan()
        time.sleep(1/20)

#find the angle of 'from' to 'to', not essential
def costurnangle(fromm,to):

    v = fromm[0]-to[0]
    u = fromm[1]-to[1]
    if v == 0:
        return 0
    if u==0:
        return 1.579
    r = math.sqrt(v*v + (u)*(u))
    point = [v,u]
    length = math.sqrt((v-r)*(v-r) + u*u)
    if v/u>0:
        return math.acos( (length*length - 2*r*r)/(2*r*r))
    return -math.acos((length * length - 2 * r * r) / (2 * r * r))

#return True if the point 'stopppoint' is already in course
def segmnentInCourse(stoppoint,course):
    for i in course:
        if i[0] == stoppoint[0] and i[1]==stoppoint[1]:
            return True
    return False
#reurn the closest point to the current tile the drone is on that is not an obstacle
def closestNotStuck(course):
    pos = client.getPose().pos
    cur = [round(pos.x_m),round(pos.y_m)]
    for length in range(10):
        for xdirection in range(length+1):
            for ydirection in range(length+1):
                if not isblocked(cur[0] + length - 2 * xdirection,cur[1] + length - 2 * ydirection ):
                    return [cur[0] + length - 2 * xdirection,cur[1] + length - 2 * ydirection ]
    print("some thing wrong")
    return []

if __name__ == "__main__":
    client = DroneClient()
    client.connect()
    speed = 3
    print(client.isConnected())
    k = 20  # how many different angles there are
    time.sleep(4)
    startpoint = [-1100, -250]
    client.setAtPosition(startpoint[0], startpoint[1], height)
    endPosition = [-250, -1000, height, speed]
    client.flyToPosition(endPosition[0], endPosition[1], height, 1)

    time.sleep(3)


    course=[]
    endpoint = [endPosition[0], endPosition[1]]

    stoppoint =[0,0]

    #realangle= costurnangle(startpoint,endpoint)

    play = False
    m=furthestPoint(startpoint, endpoint, course)
    course.append(m)
    stoppoint = [course[-2][0], course[-2][1]]
    print(stoppoint)
    fullscan()
    flag= False
    curr = course.pop(0)

    while not (endpoint[0]-2<=client.getPose().pos.x_m <= endpoint[0]+2 and endpoint[1]-2<=client.getPose().pos.y_m<=2+ endpoint[1]):
        if isStuck():
            print("stuck")
            closest = closestNotStuck(course)

            client.flyToPosition(closest[0],closest[1],height,2)
            time.sleep(2)

            client.flyToPosition(curr[0], curr[1], height, 0)
            course.clear()
            fullscan()
            course.append(furthestPoint([round(client.getPose().pos.x_m),round(client.getPose().pos.y_m)], endpoint, course))
            curr = course.pop(0)
        if curIsHere(curr,course):
            print("next course")
            if course:
                curr=course.pop(0)
            else:
                course.append(furthestPoint([round(client.getPose().pos.x_m),round(client.getPose().pos.y_m)], endpoint, course))
                curr = course[0]
            print(client.getPose().pos)
            print(curr)
        client.flyToPosition(curr[0],curr[1],height,speed)
        myCourse()
        scan()
        if courseblocked(course,stoppoint):
            stoppoint=stoppoint[0]
            client.flyToPosition(endPosition[0], endPosition[1], height, 0)
            course.clear()
            # fullscan()
            scan()
            print("bro")

            m=furthestPoint([round(client.getPose().pos.x_m),round(client.getPose().pos.y_m)], endpoint, course)
            course.clear()
            flag = True


            # while (realangle-0.1>m or m>realangle+0.1) or flag:
            while ((m[0]>endpoint[0]+1 or m[0]<endpoint[0]-1) or (m[1]>endpoint[1]+1 or m[1]<endpoint[1]-1)) or flag:
                # if endisStuck(endpoint):
                #     break
                print("per")

                flag = False
                stoppoint = keepleft(stoppoint,course)
                if segmnentInCourse(stoppoint,course):
                    course.clear()
                    break
                course.append([stoppoint[0], stoppoint[1]])
                m=furthestPoint(stoppoint, endpoint,[])
            # curr = course.pop()
            furthestPoint(stoppoint, endpoint, course)
            if not course:
                continue
            curr=course.pop(0)
            print("dun")
            flag = False
        # stoppoint = [course[-1][0], course[-1][1]]
    print("pun")
    printMap()







