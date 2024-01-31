# IMPORTING LIBRARIES

from time import sleep
import os
from collections import defaultdict
from collections import deque
from colorama import Fore
import heapq
import math

# CREATING MAZE

maze4 = [
    [" ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " "],
    [" ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " "],
    [" ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " "],
    [" ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " "],
    [" ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " "],
    [" ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " "],
    [" ", " ", " ", " ", " ", "|", "|", "|", "|", "|", "|", "|", " ", " ", " ", " "],
    [" ", " ", "|", " ", " ", " ", " ", " ", " ", " ", " ", " ", "|", " ", " ", " "],
    [" ", "|", " ", "|", " ", "|", " ", " ", " ", " ", " ", " ", " ", "|", " ", " "],
    ["|", " ", " ", " ", "|", " ", " ", " ", " ", " ", " ", " ", " ", "G", "|", " "],
    [" ", " ", " ", "|", "S", "|", " ", " ", " ", " ", " ", " ", " ", "|", " ", " "],
    [" ", " ", "|", " ", " ", " ", "|", " ", " ", " ", " ", " ", "|", " ", " ", " "],
    [" ", "|", " ", " ", " ", " ", " ", "|", " ", " ", " ", "|", " ", " ", " ", " "],
    ["|", " ", " ", " ", " ", " ", " ", " ", "|", " ", "|", " ", " ", " ", " ", " "],
    [" ", " ", " ", " ", " ", " ", " ", " ", " ", "|", " ", " ", " ", " ", " ", " "],
    [" ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " "],
]
maze3 = [
    ["|", "|", "|", "|", "|", "|", "|", "|", "|", "|", "|", "|", "|", "|", "|", "|"],
    ["S", " ", " ", " ", "|", " ", "|", "|", "|", "|", "|", "|", "|", "|", "|", "|"],
    ["|", " ", "|", " ", "|", " ", "|", " ", " ", " ", " ", "|", "|", "|", " ", "|"],
    ["|", " ", "|", " ", " ", " ", " ", " ", "|", "|", "|", "|", "|", "|", " ", "|"],
    ["|", " ", "|", "|", "|", "|", "|", "|", "|", "|", " ", "|", "|", "|", " ", "|"],
    ["|", " ", " ", " ", " ", " ", " ", " ", "|", " ", " ", " ", " ", "|", "|", "|"],
    ["|", "|", "|", "|", "|", "|", "|", " ", " ", "|", " ", "|", " ", "|", " ", "|"],
    ["|", " ", " ", " ", " ", " ", " ", "|", " ", "|", " ", "|", " ", "|", " ", "|"],
    ["|", "|", "|", "|", "|", "|", " ", "|", " ", "|", " ", "|", " ", "|", " ", "|"],
    ["|", " ", " ", " ", " ", " ", " ", "|", " ", "|", " ", " ", " ", "|", " ", "|"],
    ["|", "|", "|", "|", "|", "|", " ", "|", " ", "|", "|", "|", " ", "|", " ", "|"],
    ["|", " ", " ", " ", " ", " ", " ", " ", " ", "|", " ", "|", " ", "|", " ", "|"],
    ["|", "|", "|", "|", " ", "|", " ", "|", "|", "|", "|", "|", " ", "|", " ", "|"],
    ["|", " ", " ", "|", " ", "|", " ", " ", " ", " ", " ", " ", " ", "|", " ", "|"],
    ["|", "|", " ", " ", " ", "|", "|", "|", "|", "|", "|", "|", "|", "|", " ", "|"],
    ["|", " ", " ", "|", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", "G"],
]
maze1 = [
    [" " , "|", " ", " ", " ", "G"],
    [" " , "|", " ", " ", "|", " "],
    [" " , "|", " ", " ", " ", " "],
    [" " , " ", "|", " ", " ", "|"],
    ["|" , " ", "|", "|", " ", "|"],
    [" " , " ", " ", " ", " ", "|"],
]
wmaze1 = {
	(0,0):[(2,1,0)],
	(1,0):[(2,2,0)],
	(2,0):[(1,3,0)],
	(3,0):[(1,3,1)],
	(3,1):[(1,4,1)],
	(4,1):[(1,5,1)],
	(5,1):[(3,5,0) , (1,5,2)],
	(5,2):[(1,5,3)],
	(5,3):[(1,5,4)],
	(5,4):[(1,4,4)],
	(4,4):[(1,3,4)],
	(3,4):[(1,2,4) , (20,3,3)],
	(2,4):[(1,2,5) , (5,2,3)],
	(2,3):[(1,2,2),(100,3,3),(1,1,3),(1,2,4)],
	(2,2):[(1,3,2) , (1,2,3) , (1,1,2)],
	(2,5):[(1,1,5)],
	(3,3):[(1,3,4) , (1,2,3)],
	(1,3):[(1,0,3), (1,2,3) , (1,1,2)],
	(1,2):[(1,0,2), (1,2,2) , (1,1,3)],
	(1,5):[(1,0,5)],
	(0,4):[(1,0,5)],
	(0,3):[(1,0,4)],
	(0,2):[(1,0,2)]
	

}
maze2 = [
    [" " , "|", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " "],
    [" " , "|", " ", " ", "|", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " "],
    [" " , "|", " ", " ", "|", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " "],
    [" " , " ", "|", " ", " ", "|", " ", " ", " ", " ", " ", " ", " ", " ", " ", " "],
    ["|" , " ", "|", "|", " ", "|", " ", " ", " ", " ", " ", " ", " ", " ", " ", " "],
    [" " , " ", " ", " ", " ", "|", " ", " ", " ", " ", " ", " ", " ", " ", " ", " "],
    [" " , " ", " ", " ", " ", "|", " ", " ", " ", " ", " ", " ", " ", " ", " ", " "],
    [" " , " ", " ", " ", " ", "|", " ", " ", " ", " ", " ", " ", " ", " ", " ", " "],
    [" " , " ", " ", " ", " ", "|", " ", " ", " ", " ", " ", " ", " ", " ", " ", " "],
    [" " , " ", " ", " ", " ", "|", " ", " ", " ", " ", " ", " ", " ", " ", " ", " "],
    [" " , " ", " ", " ", " ", "|", " ", " ", " ", " ", " ", " ", " ", " ", " ", " "],
    [" " , " ", " ", " ", " ", "|", " ", " ", " ", " ", " ", " ", " ", " ", " ", " "],
    [" " , " ", " ", " ", " ", "|", " ", " ", " ", " ", " ", " ", " ", " ", " ", " "],
    [" " , " ", " ", " ", " ", "|", " ", " ", " ", " ", " ", " ", " ", " ", " ", " "],
    [" " , " ", " ", " ", " ", "|", " ", " ", " ", " ", " ", " ", " ", " ", " ", " "],
    [" " , " ", " ", " ", " ", "|", " ", " ", " ", " ", " ", " ", " ", " ", " ", "G"],
]



class Maze:

	# Constructor
	def __init__(self , maze):
		# Default dictionary to store graph
		self.nodes = 0
		self.dist = []
		self.maze = maze 
		self.height = len(maze) # HEIGHT OF THE MAZE
		self.width = len(maze[0]) # WIDTH OF THE MAZE
	
	# METHOD TO CHECK WHETHER THE NEIGBOUR IS A VALID NEIGHBOUR
	# A NEIGHBOUR IS VALID WHEN 1. NOT VISITED 2. NOT A WALL 3. WITHIN BOUNDARY OF MAZE
	def isValidNeighbour(self,neighbour):
		return neighbour[0]>=0 and neighbour[0]<self.height and neighbour[1]>=0 and neighbour[1] < self.width and self.maze[neighbour[0]][neighbour[1]] != "V" and self.maze[neighbour[0]][neighbour[1]] != "|" and self.maze[neighbour[0]][neighbour[1]] != "S"
	def calculateManhattanDistance(self , point1 , point2):
		return abs((point2[0] - point1[0])) + abs((point2[1] - point1[1]))
	#FUNCTION TO GET NEIGBOURS OF A POSITION
	def getNeighbours(self, currentPosition):
		neighbours = []
		if self.isValidNeighbour((currentPosition[0]-1 , currentPosition[1])):
			neighbours.append((currentPosition[0]-1 , currentPosition[1]))
		if self.isValidNeighbour((currentPosition[0] , currentPosition[1]+1)):
			neighbours.append((currentPosition[0], currentPosition[1]+1))
		if self.isValidNeighbour((currentPosition[0]+1 , currentPosition[1])):
			neighbours.append((currentPosition[0]+1 , currentPosition[1]))
		if self.isValidNeighbour((currentPosition[0] , currentPosition[1]-1)):
			neighbours.append((currentPosition[0], currentPosition[1]-1)) 
		return neighbours

	# METHOD TO PRINT THE MAZE
	def printMaze(self):
		print("_ "*(len(self.maze[0])+2))
		for row in self.maze:
			print("[" , end=" ")
			for col in row:
				if col == "S" or col == "G":
					print (Fore.GREEN+ col , end=" ")
				else:
					print (Fore.BLUE+ col , end=" ")
			print("]")
		print("_ "*(len(self.maze[0])+2))
		print("Nodes visited: ", self.nodes)
	
	def printDist(self):
		print("Distance Matrix")
		for row in self.dist:
			print("[" , end=" ")
			for col in row:
				print (col , end=" ")
			print("]")
	
	# A function used by DFS
	def DFSUtil(self, start):
		if self.maze[start[0]][start[1]] == "V":
			return
		if self.maze[start[0]][start[1]] == "G":
			os.system("cls")
			self.printMaze()
			print("FOUND")
			return True
		self.maze[start[0]][start[1]] = "V" if self.maze[start[0]][start[1]] != "S" else "S"
		self.nodes+=1
		os.system("cls")
		self.printMaze()
		sleep(0.05)
		neighbours = self.getNeighbours(start) 
		for neighbour in neighbours:
			if self.DFSUtil((neighbour[0],neighbour[1])):
				return True

	
	# The function to do DFS traversal. It uses
	# recursive DFSUtil()
	def DFS(self, start):
		self.maze[start[0]][start[1]] = "S"
		self.DFSUtil(start)

	# The function to do BFS traversal. It uses
	# recursive DFSUtil()
	def BFS(self,  start):
		self.BFSUtil(start)

    # A function used by BFS
	def BFSUtil(self,  start):
		self.maze[start[0]][start[1]] = "S"
		que = deque()
		que.appendleft(start)
		visited = set()
		while que:
			v = que.pop()
			self.nodes+=1
			if self.maze[v[0]][v[1]] == "G":
				os.system("cls")
				self.printMaze()
				print("FOUND")
				return True
			self.maze[v[0]][v[1]] = "V" if self.maze[v[0]][v[1]] != "S" else "S"
			visited.add(v)
			os.system("cls")
			self.printMaze()
			sleep(0.03)
			
			for neighbour in self.getNeighbours(v):
				if neighbour not in visited:
					visited.add(neighbour)
					que.appendleft(neighbour)
	def Dijkstra_with_Manhattan_Dist(self, start , end):
		pq = []
	
		heapq.heappush(pq, (0,start[0],start[1]))
 
        # Create a vector for distances and initialize all
        # distances as infinite (INF)
		self.dist = [[float("inf") for j in range(self.width) ]  for r in range(self.height)]
		self.dist[start[0]][start[1]] = 0
		while pq:
            # The first vertex in pair is the minimum distance
            # vertex, extract it from priority queue.
            # vertex label is stored in second of pair
			d, r , c = heapq.heappop(pq)
			self.nodes+=1		
			if self.maze[r][c] == "G":
				os.system("cls")
				self.printMaze()
				self.printDist()
				print("FOUND")
				print("Cost: ",d)
				break
			self.maze[r][c] = d
			os.system("cls")
			self.printMaze()
			# self.printDist()
			print("Nodes: ",self.nodes)
			sleep(0.3)
            # 'i' is used to get all adjacent vertices of a
            # vertex
			for neighbour in self.getNeighbours((r,c)):
				w = self.calculateManhattanDistance(neighbour ,end)
				w = int(w)
				if self.dist[neighbour[0]][neighbour[1]] > w:
                    # Updating distance of v
					self.dist[neighbour[0]][neighbour[1]] = w
					heapq.heappush(pq, (self.dist[neighbour[0]][neighbour[1]], neighbour[0], neighbour[1]))
	def Dijkstra(self, start , weight):
		pq = []
		heapq.heappush(pq, (0,start[0],start[1]))
        # Create a vector for distances and initialize all
        # distances as infinite (INF)
		self.dist = [[float("inf") for j in range(self.width) ]  for r in range(self.height)]
		self.dist[start[0]][start[1]] = 0
		while pq:
            # The first vertex in pair is the minimum distance
            # vertex, extract it from priority queue.
            # vertex label is stored in second of pair
			d, r , c = heapq.heappop(pq)		
			if self.maze[r][c] == "G":
				os.system("cls")
				self.printMaze()
				self.printDist()
				print("FOUND")
				print("Cost: ",d)
				break
			self.maze[r][c] = d
			os.system("cls")
			self.printMaze()
			self.printDist()
			print("Cost: ",d)
			sleep(0.3)
            # 'i' is used to get all adjacent vertices of a
            # vertex
			for neighbour in self.getNeighbours((r,c)):
                # If there is shorted path to v through u
				try:
					get_w = weight[(r,c)]
				except:
					get_w = []
				w = float('inf')
				for tup in get_w:
					if tup[1] == neighbour[0] and tup[2] == neighbour[1]:
						w = tup[0]
						break
				if self.dist[neighbour[0]][neighbour[1]] > d + w:
                    # Updating distance of v
					self.dist[neighbour[0]][neighbour[1]] = d + w
					heapq.heappush(pq, (self.dist[neighbour[0]][neighbour[1]], neighbour[0], neighbour[1]))
	def Astar(self , start , weight):
		pq = []
		heapq.heappush(pq, (0,start[0],start[1]))
        # Create a vector for distances and initialize all
        # distances as infinite (INF)
		self.dist = [[float("inf") for j in range(self.width) ]  for r in range(self.height)]
		self.dist[start[0]][start[1]] = 0
		while pq:
            # The first vertex in pair is the minimum distance
            # vertex, extract it from priority queue.
            # vertex label is stored in second of pair
			d, r , c = heapq.heappop(pq)		
			if self.maze[r][c] == "G":
				os.system("cls")
				self.printMaze()
				self.printDist()
				print("FOUND")
				print("Cost: ",d)
				break
			self.maze[r][c] = d
			os.system("cls")
			self.printMaze()
			self.printDist()
			print("Cost: ",d)
			sleep(0.3)
            # 'i' is used to get all adjacent vertices of a
            # vertex
			for neighbour in self.getNeighbours((r,c)):
                # If there is shorted path to v through u
				try:
					get_w = weight[(r,c)]
				except:
					get_w = []
				w = float('inf')
				for tup in get_w:
					if tup[1] == neighbour[0] and tup[2] == neighbour[1]:
						w = tup[0]
						break
				if self.dist[neighbour[0]][neighbour[1]] >  w+ self.dist[neighbour[0]][neighbour[1]]:
                    # Updating distance of v
					self.dist[neighbour[0]][neighbour[1]] =  w+ self.dist[neighbour[0]][neighbour[1]]
					self.maze[r][c] = d
					heapq.heappush(pq, (self.dist[neighbour[0]][neighbour[1]], neighbour[0], neighbour[1]))		
if __name__ == "__main__":
	os.system('cls')
	g = Maze(maze=maze4)
	
	# Function call
	
	option = input("Select searching algorithm \n 1. DFS \n 2. BFS \n 3. Dijkstra \n")
	startingNode = eval(input("Select starting node (row, col): "))
	if option == "1":
		g.DFS(start=startingNode)
	elif option == "2":
		g.BFS(start=startingNode)
	elif option == "3":
		g = Maze(maze1)
		g.Dijkstra((0,0) , wmaze1)
	elif option == "4":
		g = Maze(maze1)
		g.Astar((0,0) , wmaze1)
	else:
		g.Dijkstra_with_Manhattan_Dist((10,4),(9,13))

# This code is contributed by Neelam Yadav
