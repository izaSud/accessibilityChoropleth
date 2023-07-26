import json
import numpy as np
import matplotlib.path as mpltPath
import time

numDataSet = input("How many POI-Data Files? 1 or 2?")
if (numDataSet == "1"):
    file = input("School: School to School \nDoctor: Doctor to Doctor \nFiltered: Filtered to Filtered \nTwoEl \nTwoElDifZIP \nTwoElMultiPol \nTenEl \nHundredEl \nThousandEl \nSelect file: ")
else:
    file = input("School: School to School \nDoctor: Doctor to Doctor \nFiltered: Filtered to Filtered \nTwoEl \nTwoElDifZIP \nTwoElMultiPol \nTenEl \nHundredEl \nThousandEl \nSelect file: ")
    file2 = input("")
mode = input("1: Average Distances \n2: Closest Average Distances \n3: Longest Average Distances \n4: CLosest Node Average Distance \n5: Nodes Average Distance \nSelect mode (1, 2, 3, 4 or 5): ")

startingTime = time.perf_counter()



# import dataPoi if only one file is read in
# dataPoi is of type dict
if file == "Filtered":
    with open('poiFiltered.geojson') as f:
        dataPoi = json.load(f)
elif file == "School":
    with open('poiSchools.geojson') as f:
        dataPoi = json.load(f)
elif file == "Doctor":
    with open('poiDoctors.geojson') as f:
        dataPoi = json.load(f)
elif file == "TwoEl":
    with open('poiEvalTwoElements.geojson') as f:
        dataPoi = json.load(f)
elif file == "TwoElDifZIP":
    with open('poiEvalTwoElementsDifferentZIP.geojson') as f:
        dataPoi = json.load(f)
elif file == "TwoElMultiPol":
    with open('poiEvalTwoElReichenau.geojson') as f:
        dataPoi = json.load(f)
elif file == "TenEl":
    with open('poiEvalTenEl.geojson') as f:
        dataPoi = json.load(f)
elif file == "HundredEl":
    with open('poiEval100El.geojson') as f:
        dataPoi = json.load(f)
elif file == "ThousandEl":
    with open('poiEval1000El.geojson') as f:
        dataPoi = json.load(f)
else:
    raise Exception("File not found.")

# If two files are read in
if numDataSet == "2":
    if file2 == "Filtered":
        with open('poiFiltered.geojson') as f:
            dataPoi2 = json.load(f)
    elif file2 == "School":
        with open('poiSchools.geojson') as f:
            dataPoi2 = json.load(f)
    elif file2 == "Doctor":
        with open('poiDoctors.geojson') as f:
            dataPoi2 = json.load(f)
    elif file2 == "TwoEl":
        with open('poiEvalTwoElements.geojson') as f:
            dataPoi2 = json.load(f)
    elif file2 == "TwoElDifZIP":
        with open('poiEvalTwoElementsDifferentZIP.geojson') as f:
            dataPoi2 = json.load(f)
    elif file2 == "TwoElMultiPol":
        with open('poiEvalTwoElReichenau.geojson') as f:
            dataPoi2 = json.load(f)
    elif file2 == "TenEl":
        with open('poiEvalTenEl.geojson') as f:
            dataPoi2 = json.load(f)
    elif file2 == "HundredEl":
        with open('poiEval100El.geojson') as f:
            dataPoi2 = json.load(f)
    elif file2 == "ThousandEl":
        with open('poiEval1000El.geojson') as f:
            dataPoi2 = json.load(f)
    else:
        raise Exception("File not found.")

# random coordinates for assigning the extrempoints of the grid
rightLong = dataPoi['features'][0]['geometry']['coordinates'][0]
upLat = dataPoi['features'][0]['geometry']['coordinates'][1]
leftLong = dataPoi['features'][0]['geometry']['coordinates'][0]
bottomLat = dataPoi['features'][0]['geometry']['coordinates'][1]

# loop for finding and assigning the extrempoints by comparison
for feature in dataPoi['features']:
    if rightLong < feature['geometry']['coordinates'][0]:
        rightLong = feature['geometry']['coordinates'][0]
    if upLat < feature['geometry']['coordinates'][1]:
        upLat = feature['geometry']['coordinates'][1]
    if leftLong > feature['geometry']['coordinates'][0]:
        leftLong = feature['geometry']['coordinates'][0]
    if bottomLat > feature['geometry']['coordinates'][1]:
        bottomLat = feature['geometry']['coordinates'][1]

# second loop for finding and assigning the extrempoints by comparison, if two files are read in
if numDataSet == "2":
    for feature in dataPoi2['features']:
        if rightLong < feature['geometry']['coordinates'][0]:
            rightLong = feature['geometry']['coordinates'][0]
        if upLat < feature['geometry']['coordinates'][1]:
            upLat = feature['geometry']['coordinates'][1]
        if leftLong > feature['geometry']['coordinates'][0]:
            leftLong = feature['geometry']['coordinates'][0]
        if bottomLat > feature['geometry']['coordinates'][1]:
            bottomLat = feature['geometry']['coordinates'][1]

class Graph:
    # Reading file
    @staticmethod
    def read(fileName):
        #dataGraph = np.array([])
        dataGraph = []
        with open(fileName) as f:
            #lines = f.readlines()
            #dataGraph = np.append(dataGraph,lines)
            dataGraph = f.readlines()
        return dataGraph
    
    # Class for Nodes
    class Node:
        # Int value
        ID = -1
        # Float values
        lat = 0.0
        long = 0.0
        isPoi = False
        insidePolId = -1

        #inedges = np.array([])
        #outedges = np.array([])
        inedges = []
        outedges = []

        def __init__(self, ID,  lat,  long, isPoi, insidePolId):
            self.inedges = []
            self.outedges = []
            self.ID = ID
            self.lat = lat
            self.long = long
            self.isPoi = isPoi
            self.insidePolId = insidePolId

        # Incoming Edge Degree
        def  inDegree(self):
            #return self.inedges.size
            return len(self.inedges)

        # Outgoing Edge Degree
        def  outDegree(self):
            #return self.outedges.size
            return len(self.outedges)

        def  toString(self):
            sNode = "" + str(self.ID) + " " + str(self.lat) + " " + str(self.long) + " " + str(self.isPoi) + " " + str(self.insidePolId)
            return sNode

    # Class for Edges
    class Edge:
        # Int values
        # s Start-ID, t Target-ID, c Cost
        s = 0
        t = 0
        c = 0

        def __init__(self, s,  t,  c):
            self.s = s
            self.t = t
            self.c = c

        # Returns Node ID of target 
        def  getTarget(self):
            return self.t

        # Returns Node ID of start
        def getStart(self):
            return self.s

        def  toString(self):
            sEdge = "" + str(self.s) + " " + str(self.t) + " " + str(self.c)
            return sEdge

    # Class for Graph
    class G:
        # Initialized without any values
        #nodes = np.array(None)
        nodes = None
        # Initialization of dijkstra arrays

        def __init__(self, dataGraph):
            # Number of Nodes (n) and Edges (m)
            n = int(dataGraph[0])
            m = int(dataGraph[1])

            # Initialization of lists without values but with needed length
            self.nodes = [None] * (n)

            #print("read graph with " + str(n) + " nodes and " + str(m) + " edges")
            #startingTimeNodes = time.perf_counter()
            # Adds nodes from file
            i = 2
            isPoi = False
            insidePolId = -1
            while (i < n + 2):
                vpr = dataGraph[i].split()
                lat = vpr[0]
                long = vpr[1]
                node = Graph.Node(i-2, lat, long, isPoi, insidePolId)
                self.nodes[i-2] = node
                i += 1

            #print("Time for creating Nodes: ", time.perf_counter()-startingTimeNodes)
            #startingTimeEdges = time.perf_counter()
            # Adds edges from file
            i = n + 2
            while (i < n + m + 2):
                vpr = dataGraph[i].split()
                s = int(vpr[0])
                t = int(vpr[1])
                c = int(vpr[2])
                edge = Graph.Edge(s, t, c)
                #self.nodes[s].outedges.append(edge)
                self.nodes[s].outedges += edge,
                #self.nodes[t].inedges.append(edge)
                self.nodes[t].inedges += edge,
                i += 1
            
            #print("Time for creating Edges: ", time.perf_counter()-startingTimeEdges)

        def display(self):
            print(str(len(self.nodes)) + " " + str(len(self.outedges)))
            print("NODES")

            i = 0
            while (i < len(self.nodes)):
                print(self.nodes[i].toString())
                i += 1

            print("OUTEDGES")

            i = 0
            while (i < len(self.outedges)):
                print(self.outedges[i].toString())
                i += 1

        def  reversedDijkstra(self, nodesList, polPoints):
            pathlength = 0
            
            # If there are no goals, just one node in nodesList
            if len(nodesList) == 1:
                print("No Dijkstra needed, only one node in List")
                return pathlength
            
            nodesList = sorted(nodesList)
            nodeBefore = -1

            # Initialization of dijkstra array 
            distance = [1000000] * (len(self.nodes))
            pre = [None] * (len(self.nodes))
            distanceIndex = []

            for startNode in nodesList:

                if startNode == nodeBefore:
                    continue

                # Reset all found distances from previous Dijkstra-Run
                for index in distanceIndex:
                    distance[index] = 1000000
                    pre[index] = None
                distanceIndex = []
                start = int(startNode)
                goals = [-1] * len(self.nodes)

                # Distance of start node to itself is 0
                distance[start] = 0
                distanceIndex += start,

                # Get all goals 
                i = 0
                j = 0
                numberDuplicate = 0
                while (i < len(nodesList)):
                    if nodesList[i] != startNode: #nodesList[i] not in goals
                        if nodesList[i] != goals[int(nodesList[i])]:
                            goals[int(nodesList[i])] = int(nodesList[i])
                            j += 1
                    else:
                        numberDuplicate += 1
                    i += 1
                lengthGoals = j
                lengthGoalsBefore = j

                if mode == "2" and numberDuplicate > 1:
                    nodeBefore = startNode
                    continue

                # For duplicate Nodes
                multiplyingFactor = numberDuplicate

                # Initialization of Priority queue, store start node
                sNode = Graph.HeapNode(distance[start], start)
                prio = Graph.MinHeap()
                prio.insert(sNode)

                timeout = time.time() + 3
                a = time.perf_counter()

                # As long as prority queue contains nodes...
                while (not prio.isEmpty()):

                    # If all goals have been found, the loop can be terminated (Mode 1 and 2) else if first goal been found, terminate (Mode 2)
                    if lengthGoals <= 0 and (mode == "1" or mode == "3") :
                        e = time.perf_counter()
                        break
                    elif lengthGoals == (lengthGoalsBefore-1) and mode == "2":
                        e = time.perf_counter()
                        break

                    nodeV = prio.extractMin().getID()
                    v = self.nodes[nodeV]
                    distanceIndex += nodeV,

                    for inE in v.inedges:
                        if float(graph.nodes[inE.getStart()].lat) > (polPoints[0]) or float(graph.nodes[inE.getStart()].long) > (polPoints[1]) or float(graph.nodes[inE.getStart()].long) < (polPoints[3]) or float(graph.nodes[inE.getStart()].lat) < (polPoints[2]):
                            continue
                        wID = inE.getStart()
                        # If distance of reached node is bigger then v + (c)ost...
                        if (distance[wID] > distance[nodeV] + inE.c):
                            # Set distance on v + c
                            distance[wID] = distance[nodeV] + inE.c
                            distanceIndex += wID,
                            # Set current node as pre
                            pre[wID] = v
                            #preIndex += wID,
                            # Set new node into priority queue
                            w = Graph.HeapNode(distance[wID], wID)
                            prio.insert(w)

                    if time.time() > timeout:
                        e = time.perf_counter()
                        print("How many points not found? ", lengthGoals, " Of? ", lengthGoalsBefore)
                        print("TIMEOUT ", (e-a))
                        break

                    if goals[v.ID] != -1 and distance[v.ID] < 1000000:
                        lengthGoals = lengthGoals-1
                
                # Average Distances
                if mode == "1":
                    for goal in nodesList: 
                        if (distance[int(goal)] == 1000000):
                            distance[int(goal)] = -1
                            distanceIndex += int(goal),
                        else:
                            if distance[int(goal)] >= 0 and distance[int(goal)] != 1000000:
                                pathlength = pathlength + (distance[int(goal)] * multiplyingFactor)

                # Average Closest Distances
                elif mode == "2":
                    minimalDistance = 1000000
                    foundStart = False
                    for goal in nodesList: 
                        if (distance[int(goal)] == 1000000):
                            distance[int(goal)] = -1
                            distanceIndex += int(goal),
                        else:
                            if minimalDistance > distance[int(goal)] and distance[int(goal)] >= 0:
                                #if distance[int(goal)] == 0 and foundStart == False:
                                #    foundStart = True
                                #    continue
                                if goal == startNode:
                                    continue
                                minimalDistance = distance[int(goal)]
                    if minimalDistance != 1000000:
                        pathlength = pathlength + minimalDistance 

                # Average longest Distances
                elif mode == "3":
                    maximalDistance = -1
                    for goal in nodesList: 
                        if (distance[int(goal)] == 1000000):
                            distance[int(goal)] = -1
                            distanceIndex += int(goal),
                        else:
                            if maximalDistance < distance[int(goal)] and distance[int(goal)] >= 0:
                                maximalDistance = distance[int(goal)]
                    if maximalDistance != 1000000:
                        pathlength = pathlength + (maximalDistance * multiplyingFactor)

                # If there are no goals, just one node in nodesList
                if len(nodesList) == 1:
                    pathlength = distance[start]

                nodeBefore = startNode

            #print("How many time were Dijkstra done? ", count)
            return pathlength

        # Reversed Dijkstra Algorithm if two datasets of POIs are read in
        def  reversedDijkstraTwoDataSets(self, nodesList, nodesList2, polPoints):
            nodesList = sorted(nodesList)
            nodesList2 = sorted(nodesList2)
            pathlength = 0
            count = 0
            nodeBefore = -1

            # Initialization of dijkstra array 
            distance = [1000000] * (len(self.nodes))
            pre = [None] * (len(self.nodes))
            distanceIndex = []
            preIndex = []

            for startNode in nodesList:

                if startNode == nodeBefore:
                    count = count + 1
                    continue

                # Reset all found distances from previous Dijkstra-Run
                for index in distanceIndex:
                    distance[index] = 1000000
                distanceIndex = []
                start = int(startNode)
                goals = [-1] * len(self.nodes)

                # Distance of start node to itself is 0
                distance[start] = 0
                distanceIndex += start,

                # Get all goals 
                i = 0
                j = 0
                numberDuplicateGoals = 0
                while (i < len(nodesList2)):
                    if nodesList2[i] != goals[int(nodesList2[i])]: #nodesList[i] not in goals
                        goals[int(nodesList2[i])] = int(nodesList2[i])
                        j += 1
                    if startNode == nodesList2[i]:
                        numberDuplicateGoals += 1
                    i += 1
                lengthGoals = j
                lengthGoalsBefore = j

                if mode == "2" and numberDuplicateGoals > 0:
                    nodeBefore = startNode
                    continue

                i = count
                numberDuplicate = 0
                while (i < len(nodesList)):
                    if startNode == nodesList[i]:
                        numberDuplicate += 1
                    i += 1

                multiplyingFactor = numberDuplicate
                count = count + 1

                # Initialization of Priority queue, store start node
                sNode = Graph.HeapNode(distance[start], start)
                prio = Graph.MinHeap()
                prio.insert(sNode)

                timeout = time.time() + 3
                a = time.perf_counter()

                # As long as prority queue contains nodes...
                while (not prio.isEmpty()):

                    # If all goals have been found, the loop can be terminated (Mode 1 and 2) else if first goal been found, terminate (Mode 2)
                    if lengthGoals <= 0 and (mode == "1" or mode == "3") :
                        e = time.perf_counter()
                        break
                    elif lengthGoals == (lengthGoalsBefore-1) and mode == "2":
                        e = time.perf_counter()
                        break

                    nodeV = prio.extractMin().getID()
                    v = self.nodes[nodeV]

                    for inE in v.inedges:
                        if float(graph.nodes[inE.getStart()].lat) > (polPoints[0]) or float(graph.nodes[inE.getStart()].long) > (polPoints[1]) or float(graph.nodes[inE.getStart()].long) < (polPoints[3]) or float(graph.nodes[inE.getStart()].lat) < (polPoints[2]):
                            continue
                        wID = inE.getStart()
                        # If distance of reached node is bigger then v + (c)ost...
                        if (distance[wID] > distance[nodeV] + inE.c):
                            # Set distance on v + c
                            distance[wID] = distance[nodeV] + inE.c
                            distanceIndex += wID,
                            # Set current node as pre
                            pre[wID] = v
                            # Set new node into priority queue
                            w = Graph.HeapNode(distance[wID], wID)
                            prio.insert(w)

                    if time.time() > timeout:
                        e = time.perf_counter()
                        print("How many points not found? ", lengthGoals, " Of? ", lengthGoalsBefore)
                        print("TIMEOUT ", (e-a))
                        break

                    if goals[v.ID] != -1 and distance[v.ID] < 1000000:
                        lengthGoals = lengthGoals-1
                
                # Average Distances
                if mode == "1":
                    for goal in nodesList2: 
                        if (distance[int(goal)] == 1000000):
                            distance[int(goal)] = -1
                            distanceIndex += int(goal),
                        else:
                            if distance[int(goal)] >= 0 and distance[int(goal)] != 1000000:
                                pathlength = pathlength + (distance[int(goal)] * multiplyingFactor)

                # Average Closest Distances
                elif mode == "2":
                    minimalDistance = 1000000
                    foundStart = False
                    for goal in nodesList2: 
                        if (distance[int(goal)] == 1000000):
                            distance[int(goal)] = -1
                            distanceIndex += int(goal),
                        else:
                            if minimalDistance > distance[int(goal)] and distance[int(goal)] >= 0:
                                #if distance[int(goal)] == 0 and foundStart == False:
                                    #foundStart = True
                                    #continue
                                if goal == startNode:
                                    continue
                                minimalDistance = distance[int(goal)]
                    if minimalDistance < 1000000:
                        pathlength = pathlength + (minimalDistance * multiplyingFactor)

                # Average longest Distances
                elif mode == "3":
                    maximalDistance = -1
                    for goal in nodesList2: 
                        if (distance[int(goal)] == 1000000):
                            distance[int(goal)] = -1
                            distanceIndex += int(goal),
                        else:
                            if maximalDistance < distance[int(goal)] and distance[int(goal)] >= 0:
                                maximalDistance = distance[int(goal)]
                    if maximalDistance != 1000000:
                        pathlength = pathlength + (maximalDistance * multiplyingFactor)

                nodeBefore = startNode

            nodeBefore = -1
            count = 0

            for startNode in nodesList2:

                if startNode == nodeBefore:
                    count += 1
                    continue

                # Reset all found distances from previous Dijkstra-Run
                for index in distanceIndex:
                    distance[index] = 1000000
                for index in preIndex:
                    pre[index] = None
                distanceIndex = []
                preIndex = []
                start = int(startNode)
                goals = [-1] * len(self.nodes)

                # Distance of start node to itself is 0
                distance[start] = 0
                distanceIndex += start,

                # Get all goals 
                i = 0
                j = 0
                numberDuplicateGoals = 0
                while (i < len(nodesList)):
                    if nodesList[i] != goals[int(nodesList[i])]: #nodesList[i] not in goals
                        goals[int(nodesList[i])] = int(nodesList[i])
                        j += 1
                    if startNode == nodesList[i]:
                        numberDuplicateGoals += 1
                    i += 1
                lengthGoals = j
                lengthGoalsBefore = j

                if mode == "2" and numberDuplicateGoals > 0:
                    nodeBefore = startNode
                    continue

                i = count
                numberDuplicate = 0
                while (i < len(nodesList2)):
                    if startNode == nodesList2[i]:
                        numberDuplicate += 1
                    i += 1

                multiplyingFactor = numberDuplicate
                count = count + 1

                # Initialization of Priority queue, store start node
                sNode = Graph.HeapNode(distance[start], start)
                prio = Graph.MinHeap()
                prio.insert(sNode)

                timeout = time.time() + 3
                a = time.perf_counter()

                # As long as prority queue contains nodes...
                while (not prio.isEmpty()):

                    # If all goals have been found, the loop can be terminated (Mode 1 and 2) else if first goal been found, terminate (Mode 2)
                    if lengthGoals <= 0 and (mode == "1" or mode == "3") :
                        e = time.perf_counter()
                        break
                    elif lengthGoals == (lengthGoalsBefore-1) and mode == "2":
                        e = time.perf_counter()
                        break

                    nodeV = prio.extractMin().getID()
                    v = self.nodes[nodeV]

                    for inE in v.inedges:
                        if float(graph.nodes[inE.getStart()].lat) > (polPoints[0]) or float(graph.nodes[inE.getStart()].long) > (polPoints[1]) or float(graph.nodes[inE.getStart()].long) < (polPoints[3]) or float(graph.nodes[inE.getStart()].lat) < (polPoints[2]):
                            continue
                        wID = inE.getStart()
                        # If distance of reached node is bigger then v + (c)ost...
                        if (distance[wID] > distance[nodeV] + inE.c):
                            # Set distance on v + c
                            distance[wID] = distance[nodeV] + inE.c
                            distanceIndex += wID,
                            # Set current node as pre
                            pre[wID] = v
                            preIndex += wID,
                            # Set new node into priority queue
                            w = Graph.HeapNode(distance[wID], wID)
                            prio.insert(w)

                    if time.time() > timeout:
                        e = time.perf_counter()
                        print("How many points not found? ", lengthGoals, " Of? ", lengthGoalsBefore)
                        print("TIMEOUT ", (e-a))
                        break

                    if goals[v.ID] != -1 and distance[v.ID] < 1000000:
                        lengthGoals = lengthGoals-1
                
                # Average Distances
                if mode == "1":
                    for goal in nodesList: 
                        if (distance[int(goal)] == 1000000):
                            distance[int(goal)] = -1
                            distanceIndex += int(goal),
                        else:
                            if distance[int(goal)] >= 0 and distance[int(goal)] != 1000000:
                                pathlength = pathlength + (distance[int(goal)] * multiplyingFactor)

                # Average Closest Distances
                elif mode == "2":
                    minimalDistance = 1000000
                    foundStart = False
                    for goal in nodesList: 
                        if (distance[int(goal)] == 1000000):
                            distance[int(goal)] = -1
                            distanceIndex += int(goal),
                        else:
                            if minimalDistance > distance[int(goal)] and distance[int(goal)] >= 0:
                                #if distance[int(goal)] == 0 and foundStart == False:
                                #    foundStart = True
                                #    continue
                                if goal == startNode:
                                    continue
                                minimalDistance = distance[int(goal)]
                    if minimalDistance < 1000000:
                        pathlength = pathlength + (minimalDistance * multiplyingFactor)

                # Average longest Distances
                elif mode == "3":
                    maximalDistance = -1
                    for goal in nodesList: 
                        if (distance[int(goal)] == 1000000):
                            distance[int(goal)] = -1
                            distanceIndex += int(goal),
                        else:
                            if maximalDistance < distance[int(goal)] and distance[int(goal)] >= 0:
                                maximalDistance = distance[int(goal)]
                    if maximalDistance != 1000000:
                        pathlength = pathlength + (maximalDistance * multiplyingFactor)

                nodeBefore = startNode      

            #print("How many time were Dijkstra done? ", count)
            return pathlength

        def  singleReversedDijkstra(self, nodesList, polPoints):
            pathlength = 0

            # If there are no goals, just one node in nodesList
            if len(nodesList) == 1:
                print("No Dijkstra needed, only one node in List")
                return pathlength

            # Initialization of dijkstra array 
            distance = [1000000] * (len(self.nodes))
            pre = [None] * (len(self.nodes))
            distanceIndex = []
            prio = Graph.MinHeap()
            runCount = 0
            numOfNodes = 0

            # Distance of start node to itself is 0
            for node in nodesList:
                start = int(node)
                distance[start] = 0

                # Initialization of Priority queue, store start node
                sNode = Graph.HeapNode(distance[start], start)
                prio.insert(sNode)

            # As long as prority queue contains nodes...
            while (not prio.isEmpty()):

                #if mode == "4" and runCount == len(nodesList):
                #    break

                closestNodeDistance = 1000000
                closestNodeID = 0
                nodeFound = False

                nodeV = prio.extractMin().getID()
                v = self.nodes[nodeV]

                for inE in v.inedges:
                    if float(graph.nodes[inE.getStart()].lat) > (polPoints[0]) or float(graph.nodes[inE.getStart()].long) > (polPoints[1]) or float(graph.nodes[inE.getStart()].long) < (polPoints[3]) or float(graph.nodes[inE.getStart()].lat) < (polPoints[2]):
                        continue
                    wID = inE.getStart()
                    # If distance of reached node is bigger then v + (c)ost...
                    if (distance[wID] > distance[nodeV] + inE.c):
                        # Set distance on v + c
                        distance[wID] = distance[nodeV] + inE.c
                        if mode == "5":
                            distanceIndex += wID,
                            numOfNodes += 1
                        # Set current node as pre
                        pre[wID] = v
                        # Set new node into priority queue
                        w = Graph.HeapNode(distance[wID], wID)
                        if mode == "5":
                            prio.insert(w)
                    if closestNodeDistance > distance[wID] and mode == "4":
                        closestNodeDistance = distance[wID]
                        closestNodeID = wID
                        nodeFound = True

                if mode == "4" and nodeFound == True:
                    distanceIndex += closestNodeID,

                runCount += 1

            for node in distanceIndex:
                if distance[node] != 1000000:
                    pathlength = pathlength + distance[node]

            if mode == "4":
                return pathlength
            else:
                return pathlength/numOfNodes

    # Container for Node Pairs for which the pathlength should be calculated
    class Pair:
        start = 0
        goal = 0

        def __init__(self, start,  goal):
            self.start = start
            self.goal = goal

    class MinHeap:
        heap =  []

        # Constructor
        def  isEmpty(self) :
            if ((len(self.heap) == 0)):
                return True
            else:
                return False

        # Returns Node with minimal distance
        def  extractMin(self):
            min = self.heap[0]
            indexOfLast = len(self.heap) - 1
            last = self.heap[indexOfLast]
            self.heap[0] = last
            del self.heap[indexOfLast]
            self.heapify(1)
            return min

        def insert(self, node):
            #self.heap.append(node)
            self.heap += node,
            child = len(self.heap) - 1 # Position of child
            self.swap(child)

        def swap(self, child):
            if (child == 0): 
                pass
            # Comparing distance of child with distance of parent
            elif(self.heap[child].getDist() < self.getParent(child).getDist()):
                parent = self.getParent(child) # Get parent
                parentPos = int((child - 1) / 2) # Get current position of parent
                childNode = self.heap[child] # Get child
                self.heap[parentPos] = childNode # Set child on previous position of parent
                self.heap[child] = parent # Set parent node on previous child position
                child = parentPos
                self.swap(child)

        def  getParent(self, child):
            parent = int((child - 1) / 2)
            return self.heap[parent]

        # Restores heap properties after removal of minimum
        def heapify(self, pos):
            leftkey = 2 * pos # Left child
            rightkey = (2 * pos) + 1 # Right child
            last = len(self.heap) # Last node on heap

            # If node is already last node in heap
            if (leftkey > last and rightkey > last): 
                pass
            # swap with left child
            elif(rightkey >= last):
                if (self.heap[pos - 1].getDist() > self.heap[leftkey - 1].getDist()):
                    sinkElement = self.heap[pos - 1]
                    riseElement = self.heap[leftkey - 1]
                    self.heap[pos - 1] = riseElement
                    self.heap[leftkey - 1] = sinkElement
                    self.heapify(leftkey)
            else:
                if (self.heap[rightkey - 1].getDist() > self.heap[leftkey - 1].getDist()):
                    sinkElement = self.heap[pos - 1]
                    riseElement = self.heap[leftkey - 1]
                    self.heap[pos - 1] = riseElement
                    self.heap[leftkey - 1] = sinkElement
                    self.heapify(leftkey)
                else:
                    sinkElement = self.heap[pos - 1]
                    riseElement = self.heap[rightkey - 1]
                    self.heap[pos - 1] = riseElement
                    self.heap[rightkey - 1] = sinkElement
                    self.heapify(rightkey)

        def printHeap(self):
            i = 0
            while (i < len(self.heap)):
                print("Node: " + self.heap[i].toString() + ", ", end ="")
                i += 1

    class HeapNode :
        dist = 0
        ID = 0

        def __init__(self, dist,  ID) :
            self.dist = dist
            self.ID = ID

        # Returns the dist
        def  getDist(self):
            return self.dist

        # Returns the ID
        def  getID(self):
            return self.ID

        def  toString(self):
            return "(" + str(self.dist) + ", " + str(self.ID) + ")"

    @staticmethod
    def main(args):
        #print("\nGraph\n")
        if (len('bw.txt') < 4):
            return
    
if __name__=="__main__":
    Graph.main([])

startingTimeBeforeGraph = time.perf_counter()
dataGraph = Graph.read('bw.txt')
graph = Graph.G(dataGraph)
print("Time after graph is read: ", time.perf_counter()-startingTimeBeforeGraph)

# Calculations for distance in grid
x_coord = rightLong - leftLong
y_coord = upLat - bottomLat

gridLength = 16

xDistancePerGrid = x_coord/gridLength
yDistancePerGrid = y_coord/gridLength

startingTimeGrid = time.perf_counter()

# Latitute and Longitute Array. In each field are the values for the lat/long extrempoints per grid
latArray = []
longArray = []

for i in range(gridLength):
    latArray.append([bottomLat + (yDistancePerGrid*i), bottomLat + (yDistancePerGrid*(i+1))])
    longArray.append([leftLong + (xDistancePerGrid*i), leftLong + (xDistancePerGrid*(i+1))])

gridArray = np.zeros((gridLength,gridLength), dict)

if numDataSet == "1":
    for i in range(gridLength):
        for j in range(gridLength):
            points = {}
            points["poi"] = []
            points["nodes"] = []
            gridArray[i,j] = points
elif numDataSet == "2":
    for i in range(gridLength):
        for j in range(gridLength):
            points = {}
            points["poi"] = []
            points["poi2"] = []
            points["nodes"] = []
            gridArray[i,j] = points

# Add POIs and nodes into 10x10 gridArray (gridArray contains dict)
for feature in dataPoi['features']:
    if feature['geometry']['coordinates'][0] > rightLong or feature['geometry']['coordinates'][0] < leftLong or feature['geometry']['coordinates'][1] > upLat or feature['geometry']['coordinates'][1] < bottomLat:
        continue
    foundPOIpos = False
    for i in range(gridLength):
        for j in range(gridLength):
            if feature['geometry']['coordinates'][0] >= longArray[i][0] and feature['geometry']['coordinates'][0] <= longArray[i][1]:
                if feature['geometry']['coordinates'][1] >= latArray[j][0] and feature['geometry']['coordinates'][1] <= latArray[j][1]:
                    gridArray[i,j]["poi"] += feature['geometry']['coordinates'],
                    foundPOIpos = True
            if foundPOIpos:
                break
        if foundPOIpos:
            break

# Add second set of POIs into 10x10 gridArray (gridArray contains dict), when two files are read in
if numDataSet == "2":
    for feature in dataPoi2['features']:
        if feature['geometry']['coordinates'][0] > rightLong or feature['geometry']['coordinates'][0] < leftLong or feature['geometry']['coordinates'][1] > upLat or feature['geometry']['coordinates'][1] < bottomLat:
            continue
        foundPOIpos = False
        for i in range(gridLength):
            for j in range(gridLength):
                if feature['geometry']['coordinates'][0] >= longArray[i][0] and feature['geometry']['coordinates'][0] <= longArray[i][1]:
                    if feature['geometry']['coordinates'][1] >= latArray[j][0] and feature['geometry']['coordinates'][1] <= latArray[j][1]:
                        gridArray[i,j]["poi2"] += feature['geometry']['coordinates'],
                        foundPOIpos = True
                if foundPOIpos:
                    break
            if foundPOIpos:
                break

for point in range(int(dataGraph[0])):
    if float(graph.nodes[point].long) > rightLong or float(graph.nodes[point].long) < leftLong or float(graph.nodes[point].lat) > upLat or float(graph.nodes[point].lat) < bottomLat:
        continue
    foundNodePos = False
    for i in range(gridLength):
        for j in range(gridLength):
            if float(graph.nodes[point].long) >= longArray[i][0] and float(graph.nodes[point].long) < longArray[i][1]:
                if float(graph.nodes[point].lat) >= latArray[j][0] and float(graph.nodes[point].lat) < latArray[j][1]:
                    gridArray[i,j]["nodes"] += graph.nodes[point],
                    foundNodePos = True
            if foundNodePos:
                break
        if foundNodePos:
            break

print("Time after pois and nodes are added to gridArray: ", time.perf_counter()-startingTimeGrid)

startingTimeNearestNode = time.perf_counter()

# Assign node to POI, checking the grid cell the POI is in and all surrounding grid cells
isPoiListID = []
isPoiListID2 = []
for i in range(gridLength):
    for j in range(gridLength):
        aroundPoiList = []
        if i > 0 and j > 0 and i < gridLength-1 and j < gridLength-1:
            aroundPoiList += gridArray[i,j]["nodes"],
            aroundPoiList += gridArray[i+1,j]["nodes"],
            aroundPoiList += gridArray[i,j+1]["nodes"],
            aroundPoiList += gridArray[i+1,j+1]["nodes"],
            aroundPoiList += gridArray[i-1,j-1]["nodes"],
            aroundPoiList += gridArray[i-1,j]["nodes"],
            aroundPoiList += gridArray[i,j-1]["nodes"],
            aroundPoiList += gridArray[i+1,j-1]["nodes"],
            aroundPoiList += gridArray[i-1,j+1]["nodes"],
        elif i == 0 and j == 0:
            aroundPoiList += gridArray[i,j]["nodes"],
            aroundPoiList += gridArray[i+1,j]["nodes"],
            aroundPoiList += gridArray[i,j+1]["nodes"],
            aroundPoiList += gridArray[i+1,j+1]["nodes"],
        elif i == gridLength-1 and j == gridLength-1:
            aroundPoiList += gridArray[i,j]["nodes"],
            aroundPoiList += gridArray[i-1,j]["nodes"],
            aroundPoiList += gridArray[i,j-1]["nodes"],
            aroundPoiList += gridArray[i-1,j-1]["nodes"],
        elif i == 0 and j == gridLength-1:
            aroundPoiList += gridArray[i,j]["nodes"],
            aroundPoiList += gridArray[i,j-1]["nodes"],
            aroundPoiList += gridArray[i+1,j]["nodes"],
            aroundPoiList += gridArray[i+1,j-1]["nodes"],
        elif i == gridLength-1 and j == 0:
            aroundPoiList += gridArray[i,j]["nodes"],
            aroundPoiList += gridArray[i-1,j]["nodes"],
            aroundPoiList += gridArray[i,j+1]["nodes"],
            aroundPoiList += gridArray[i-1,j+1]["nodes"],
        elif i > 0 and i < gridLength-1 and j == 0:
            aroundPoiList += gridArray[i,j]["nodes"],
            aroundPoiList += gridArray[i-1,j]["nodes"],
            aroundPoiList += gridArray[i-1,j+1]["nodes"],
            aroundPoiList += gridArray[i,j+1]["nodes"],
            aroundPoiList += gridArray[i+1,j]["nodes"],
            aroundPoiList += gridArray[i+1,j+1]["nodes"],
        elif i == gridLength-1 and j > 0 and j < gridLength-1:
            aroundPoiList += gridArray[i,j]["nodes"],
            aroundPoiList += gridArray[i-1,j-1]["nodes"],
            aroundPoiList += gridArray[i,j-1]["nodes"],
            aroundPoiList += gridArray[i-1,j]["nodes"],
            aroundPoiList += gridArray[i-1,j+1]["nodes"],
            aroundPoiList += gridArray[i,j+1]["nodes"],
        elif i > 0 and i < gridLength-1 and j == gridLength-1:
            aroundPoiList += gridArray[i,j]["nodes"],
            aroundPoiList += gridArray[i-1,j-1]["nodes"],
            aroundPoiList += gridArray[i-1,j]["nodes"],
            aroundPoiList += gridArray[i,j-1]["nodes"],
            aroundPoiList += gridArray[i+1,j-1]["nodes"],
            aroundPoiList += gridArray[i+1,j]["nodes"],
        elif i == 0 and j > 0 and j < gridLength-1:
            aroundPoiList += gridArray[i,j]["nodes"],
            aroundPoiList += gridArray[i,j-1]["nodes"],
            aroundPoiList += gridArray[i,j+1]["nodes"],
            aroundPoiList += gridArray[i+1,j-1]["nodes"],
            aroundPoiList += gridArray[i+1,j]["nodes"],
            aroundPoiList += gridArray[i+1,j+1]["nodes"],
        for poi in gridArray[i,j]["poi"]:
            minimalDistanceX = xDistancePerGrid
            minimalDistanceY = yDistancePerGrid
            nearestNodeID = None
            k = 0
            while k < len(aroundPoiList):
                for node in aroundPoiList[k]:
                    if  float(minimalDistanceX) > abs(float(poi[0])-float(node.long)) and float(minimalDistanceY) > abs(float(poi[1])-float(node.lat)) and (len(node.inedges) > 0 or len(node.outedges) > 0): #(node.ID in isPoiListID) == False and
                        minimalDistanceX = abs(float(poi[0])-float(node.long))
                        minimalDistanceY = abs(float(poi[1])-float(node.lat))
                        nearestNodeID = node.ID
                k += 1
            isPoiListID += nearestNodeID,
        # If a second set of POIs are read in, check for them the nearest node as well
        if numDataSet == "2":
            for poi in gridArray[i,j]["poi2"]:
                minimalDistanceX = xDistancePerGrid
                minimalDistanceY = yDistancePerGrid
                nearestNodeID = None
                k = 0
                while k < len(aroundPoiList):
                    for node in aroundPoiList[k]:
                        if  float(minimalDistanceX) > abs(float(poi[0])-float(node.long)) and float(minimalDistanceY) > abs(float(poi[1])-float(node.lat)) and (len(node.inedges) > 0 or len(node.outedges) > 0): #(node.ID in isPoiListID) == False and
                            minimalDistanceX = abs(float(poi[0])-float(node.long))
                            minimalDistanceY = abs(float(poi[1])-float(node.lat))
                            nearestNodeID = node.ID
                    k += 1
                isPoiListID2 += nearestNodeID,


print("Time after finding out which node is a poi: ", time.perf_counter()-startingTimeNearestNode)

# Sets every nearest node to POI to isPoi = True
poiNodesList = []
for nodeID in isPoiListID:
    if nodeID is not None:
        graph.nodes[nodeID].isPoi = True
        poiNodesList += graph.nodes[nodeID],

# Sets every nearest node to the second set of POIs to isPoi = True
if numDataSet == "2":
    poiNodesList2 = []
    for nodeID in isPoiListID2:
        if nodeID is not None:
            graph.nodes[nodeID].isPoi = True
            poiNodesList2 += graph.nodes[nodeID],

with open('plz_area.geojson') as f:
#with open('plz_evaluation.geojson') as f:
    dataPLZ = json.load(f)

#fig, axs = plt.subplots()
#axs.set_aspect('equal', 'datalim')

#for geom in dataPLZ['features'][25]['geometry']['coordinates']:
#    shape = geometry.Polygon(geom[0])
#    xs, ys = shape.exterior.xy    
#    axs.fill(xs, ys, alpha=0.5, fc='r', ec='none')

#plt.show()

dist = np.zeros((len(dataPLZ['features'])), dict)

# Adds all polygon IDs to a array with key plz and distance
i = 0
for polygon in dataPLZ['features']:
    distanceArray = {}
    distanceArray["plz"] = []
    distanceArray["extremePoints"] = []
    distanceArray["distance"] = []
    distanceArray["nodes"] = []
    distanceArray["nodes2"] = []
    distanceArray["plz"].append(polygon['properties']['id'])
    dist[i] = distanceArray
    i += 1

# Getting extremepoints for every polygon and adding to array 'dist' in order: Upper border (maxLat), right border (maxLong), bottom border (minLat) and left border (minLong)
for polygon in dataPLZ['features']:
    maxLatVal = -90
    minLatVal = 90
    maxLongVal = -180
    minLongVal = 180
    if polygon['geometry']['type'] == 'Polygon':
        for coord in polygon['geometry']['coordinates'][0]:
            if maxLongVal < coord[0]:
                maxLongVal = coord[0]
            elif minLongVal > coord[0]:
                minLongVal = coord[0]

            if maxLatVal < coord[1]:
                maxLatVal = coord[1]
            elif minLatVal > coord[1]:
                minLatVal = coord[1]
    if polygon['geometry']['type'] == 'MultiPolygon':
        for multiPol in polygon['geometry']['coordinates']:
            for coord in multiPol[0]:
                if maxLongVal < coord[0]:
                    maxLongVal = coord[0]
                elif minLongVal > coord[0]:
                    minLongVal = coord[0]

                if maxLatVal < coord[1]:
                    maxLatVal = coord[1]
                elif minLatVal > coord[1]:
                    minLatVal = coord[1]
    extremePoints = [maxLatVal,maxLongVal,minLatVal,minLongVal]
    for plz in dist:
        if plz["plz"][0] == polygon['properties']['id']:
            plz["extremePoints"] = np.append(plz["extremePoints"], extremePoints)

startingTimeNodeInsidePolygon = time.perf_counter()

# Determines if node is inside polygon
checkPol = 0
checkMultiPol = 0
checkInMult = 0
for node in poiNodesList:
    if float(node.long) > rightLong or float(node.long) < leftLong or float(node.lat) > upLat or float(node.lat) < bottomLat:
        continue
    for polygon in dataPLZ['features']:
        if isinstance(node, Graph.Node):
            if polygon['geometry']['type'] == 'Polygon':
                path = mpltPath.Path(polygon['geometry']['coordinates'][0])
                inside = path.contains_points([[float(node.long), float(node.lat)]], radius=1e-9)
                if inside:
                    node.insidePolId = polygon['properties']['id']
                    checkPol += 1
                    break
            if polygon['geometry']['type'] == 'MultiPolygon':
                for multiPol in polygon['geometry']['coordinates']:
                    path = mpltPath.Path(multiPol[0])
                    inside = path.contains_points([[float(node.long), float(node.lat)]], radius=1e-9)
                    if inside:
                        node.insidePolId = polygon['properties']['id']
                        checkMultiPol += 1
                        break

# Determines if the nodes of the second set of POIs are inside polygon
if numDataSet == "2":
    for node in poiNodesList2:
        if float(node.long) > rightLong or float(node.long) < leftLong or float(node.lat) > upLat or float(node.lat) < bottomLat:
            continue
        for polygon in dataPLZ['features']:
            if isinstance(node, Graph.Node):
                if polygon['geometry']['type'] == 'Polygon':
                    path = mpltPath.Path(polygon['geometry']['coordinates'][0])
                    inside = path.contains_points([[float(node.long), float(node.lat)]], radius=1e-9)
                    if inside:
                        node.insidePolId = polygon['properties']['id']
                        checkPol += 1
                        break
                if polygon['geometry']['type'] == 'MultiPolygon':
                    for multiPol in polygon['geometry']['coordinates']:
                        path = mpltPath.Path(multiPol[0])
                        inside = path.contains_points([[float(node.long), float(node.lat)]], radius=1e-9)
                        if inside:
                            node.insidePolId = polygon['properties']['id']
                            checkMultiPol += 1
                            break


print("Finding out which node is in which polygon: ", time.perf_counter()-startingTimeNodeInsidePolygon)

countActual = 0
countTheoratical = 0
# Place POI in their corresponding plz
for node in poiNodesList:
    countTheoratical += 1
    for plz in dist:
        if plz["plz"][0] == node.insidePolId:
            countActual += 1
            plz["nodes"] = np.append(plz["nodes"], int(node.ID))
            break

# Place POI of second set in their corresponding plz
if numDataSet == "2":
    for node in poiNodesList2:
        countTheoratical += 1
        for plz in dist:
            if plz["plz"][0] == node.insidePolId:
                countActual += 1
                plz["nodes2"] = np.append(plz["nodes2"], int(node.ID))
                break

#print("How many POIs should? ", countTheoratical)
#print("How many POIs now? ", countActual)

startingTimeDijkstra = time.perf_counter()

# Reversed Dijkstra by PLZ-Area
if numDataSet == "1":
    for plz in dist:
        if len(plz["nodes"]) != 0:
            startDijkstraRun = time.perf_counter()
            if mode == "4" or mode == "5":
                path = graph.singleReversedDijkstra(plz["nodes"],plz["extremePoints"])
            else:
                path = graph.reversedDijkstra(plz["nodes"],plz["extremePoints"])
            iPath = int(path)
            plz["distance"].append(iPath)
            print("Dijkstra Run with ", len(plz["nodes"]), " pois, time: ", time.perf_counter()-startDijkstraRun)
# Reversed Dijkstra by PLZ-Area for two set of POIs
elif numDataSet == "2":
    for plz in dist:
        if len(plz["nodes"]) != 0 and len(plz["nodes2"]) != 0:
            startDijkstraRun = time.perf_counter()
            if mode == "4" or mode == "5":
                allNodes = [*plz["nodes"], *plz["nodes2"]]
                path = graph.singleReversedDijkstra(allNodes,plz["extremePoints"])
            else:
                path = graph.reversedDijkstraTwoDataSets(plz["nodes"],plz["nodes2"],plz["extremePoints"])
            iPath = int(path)
            plz["distance"].append(iPath)
            print("Dijkstra Run with ", len(plz["nodes"])+len(plz["nodes2"]), " pois, time: ", time.perf_counter()-startDijkstraRun)
        elif (len(plz["nodes"]) == 0 and len(plz["nodes2"]) != 0) or (len(plz["nodes"]) != 0 and len(plz["nodes2"]) == 0):
            plz["distance"].append(0)

print("Time for Dijkstra: ", time.perf_counter() - startingTimeDijkstra)

numOfPol = len(dist)
pos = 0
f = open("finalClosestNodeDistancesTest.js", "w")
f.write("var distanceClosestNodeTest = { \n"
        "\"type\": \"FeatureCollection\", \n"
        "\"features\": [\n")
for numDist in dist:
    if numDataSet == "1":
        length = len(numDist["nodes"])
    elif numDataSet == "2":
        length = len(numDist["nodes"]) + len(numDist["nodes2"])
    pos += 1
    if pos < numOfPol:
        if numDataSet == "1":
            if length != 0:
                sumDist = int(numDist["distance"][0])
                if length > 1:
                    if mode == "1":
                        avgDist = sumDist/(length*(length-1))
                    elif mode == "2" or mode == "3" or mode == "4":
                        avgDist = sumDist/length
                    elif mode == "5":
                        avgDist = sumDist
                else:
                    avgDist = sumDist
                numDist["distance"] = avgDist
                f.write("{ \"type\": \"Feature\",\"properties\": {\"zip\": " + numDist["plz"][0] + ", \"distance\": " + str(avgDist) + "}}, \n")
            else:
                numDist["distance"] = -1
                f.write("{ \"type\": \"Feature\",\"properties\": {\"zip\": " + numDist["plz"][0] + ", \"distance\": " + str(-1) + "}}, \n")
        elif numDataSet == "2":
            if len(numDist["nodes"]) > 0 and len(numDist["nodes2"]) > 0:
                sumDist = int(numDist["distance"][0])
                if length > 1:
                    if mode == "1":
                        avgDist = sumDist/len(numDist["nodes"]) * len(numDist["nodes2"]) * 2
                    elif mode == "2" or mode == "3" or mode == "4":
                        avgDist = sumDist/length
                    elif mode == "5":
                        avgDist = sumDist
                else:
                    avgDist = sumDist
                numDist["distance"] = avgDist
                f.write("{ \"type\": \"Feature\",\"properties\": {\"zip\": " + numDist["plz"][0] + ", \"distance\": " + str(avgDist) + "}}, \n")
            else:
                numDist["distance"] = -1
                f.write("{ \"type\": \"Feature\",\"properties\": {\"zip\": " + numDist["plz"][0] + ", \"distance\": " + str(-1) + "}}, \n")
    else: 
        if numDataSet == "1":
            if length != 0:
                sumDist = int(numDist["distance"][0])
                if length > 1:
                    if mode == "1": # Avg Distance of POIs
                        avgDist = sumDist/(length*(length-1))
                    elif mode == "2" or mode == "3" or mode == "4": # Avg Distance of Closest, Longest and Closest Node
                        avgDist = sumDist/length
                    elif mode == "5": # Avg Distances of all Nodes used in one Dijkstra
                        avgDist = sumDist
                else:
                    avgDist = sumDist
                numDist["distance"] = avgDist
                f.write("{ \"type\": \"Feature\",\"properties\": {\"zip\": " + numDist["plz"][0] + ", \"distance\": " + str(avgDist) + "}} \n")
            else:
                numDist["distance"] = -1
                f.write("{ \"type\": \"Feature\",\"properties\": {\"zip\": " + numDist["plz"][0] + ", \"distance\": " + str(-1) + "}} \n")
        elif numDataSet == "2":
            if len(numDist["nodes"]) > 0 and len(numDist["nodes2"]) > 0:
                sumDist = int(numDist["distance"][0])
                if length > 1:
                    if mode == "1": # Avg Distance of POIs
                        avgDist = sumDist/len(numDist["nodes"]) * len(numDist["nodes2"]) * 2
                    elif mode == "2" or mode == "3" or mode == "4": # Avg Distance of Closest, Longest and Closest Node
                        avgDist = sumDist/length
                    elif mode == "5": # Avg Distances of all Nodes used in one Dijkstra
                        avgDist = sumDist
                else:
                    avgDist = sumDist
                numDist["distance"] = avgDist
                f.write("{ \"type\": \"Feature\",\"properties\": {\"zip\": " + numDist["plz"][0] + ", \"distance\": " + str(avgDist) + "}} \n")
            else:
                numDist["distance"] = -1
                f.write("{ \"type\": \"Feature\",\"properties\": {\"zip\": " + numDist["plz"][0] + ", \"distance\": " + str(-1) + "}} \n")
            
f.write("]};")
f.close()

endingTime = time.perf_counter()
print("Time until End: ", (endingTime-startingTime))
print("END")