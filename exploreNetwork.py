import random
import math
import sys
import multiprocessing
import cPickle

def f(p, q, x, y):
  return -30.0*math.log10(math.hypot(x-p, y-q))

def dist(x1, y1, x2, y2):
  return math.hypot(x1-x2, y1-y2)

class Node:
  def __init__(self, x, y):
    self.x = x
    self.y = y

def generateNodes(minX, maxX, minY, maxY, numNodes, d):
  nodes = []
  while len(nodes) < 0.1*numNodes+1:
    firstNode = Node(random.uniform(minX+(maxX-minX)/5.0, maxX-(maxX-minX)/5.0), random.uniform(minY+(maxY-minY)/5.0, maxY-(maxY-minY)/5.0))
    tooClose = False
    for node in nodes:
      if node != firstNode:
        neighborDist = dist(firstNode.x, firstNode.y, node.x, node.y)
        if neighborDist < d:
          tooClose = True
          break
    if not tooClose:
      nodes.append(firstNode)
  while len(nodes) < numNodes:
    refNode = random.choice(nodes)
    for i in range(3):
      direction = random.uniform(0.0, 2.0*math.pi)
      x = refNode.x + d*math.cos(direction)
      y = refNode.y + d*math.sin(direction)
      if x > minX and x < maxX:
        if y > minY and y < maxY:
          tooClose = False
          for node in nodes:
            if node != refNode:
              neighborDist = dist(node.x, node.y, x, y)
              if neighborDist < d:
                tooClose = True
                break
          if tooClose:
            continue
          else:
            nodes.append(Node(x, y))
            break
  return nodes

def explore(args):
  vx = args[0]
  vy = args[1]
  x = [args[2]]
  y = [args[3]]
  
  nodes = args[4]
  fNoise = args[5]
  dNoise = args[6]
  frac = args[7]

  # real:
  visitedNodes = []
  n = 0
  while True:
    visibleNodes = []
    RSSI = []
    for node in nodes:
      if dist(node.x, node.y, x[-1], y[-1] < 200):
        signal = f(node.x, node.y, x[-1], y[-1])
        if signal > -80:
          visibleNodes.append(node)
          RSSI.append(signal + random.gauss(0.0, fNoise))
    visitedNodes = list(set(visitedNodes + visibleNodes))
    if len(visibleNodes) > 0:
      maxRSSI = max(RSSI)
      maxRSSINode = visibleNodes[RSSI.index(maxRSSI)]
      dx = (maxRSSINode.x-x[-1])
      dy = (maxRSSINode.y-y[-1])
      theta = math.atan2(dy, dx) + random.gauss(0.0, dNoise)
      dx = math.cos(theta)
      dy = math.sin(theta)
      D = math.hypot(dx, dy)
      dx = dx/D
      dy = dy/D
      if maxRSSI < -65:
        vx = (1.0-frac)*vx + frac*dx
        vy = (1.0-frac)*vy + frac*dy
      elif maxRSSI > -60:
        vx = (1.0-frac)*vx - frac*dx
        vy = (1.0-frac)*vy - frac*dy
      else:
        vx = (1.0-frac)*vx - frac*dy
        vy = (1.0-frac)*vy + frac*dx
    x.append(x[-1]+vx)
    y.append(y[-1]+vy)
    n = n+1
    if n % 500 == 0 and n > 15000:
      if min([dist(x[-1], y[-1], pos[0], pos[1]) for pos in zip(x[:4500], y[:4500])]) < 50.0:
        break
    if n > 150000:
      break
  nvis = len(visitedNodes)
  duration = len(x)
  #ideal: 
  vx = args[0]
  vy = args[1]
  x = [args[2]]
  y = [args[3]]
  visitedNodes = []
  n = 0
  while True:
    visibleNodes = []
    RSSI = []
    for node in nodes:
      if dist(node.x, node.y, x[-1], y[-1] < 200):
        signal = f(node.x, node.y, x[-1], y[-1])
        if signal > -80:
          visibleNodes.append(node)
          RSSI.append(signal)
    visitedNodes = list(set(visitedNodes + visibleNodes))
    if len(visibleNodes) > 0:
      maxRSSI = max(RSSI)
      maxRSSINode = visibleNodes[RSSI.index(maxRSSI)]
      dx = (maxRSSINode.x-x[-1])
      dy = (maxRSSINode.y-y[-1])
      D = math.hypot(dx, dy)
      dx = dx/D
      dy = dy/D
      if maxRSSI < -65:
        vx = (1.0-frac)*vx + frac*dx
        vy = (1.0-frac)*vy + frac*dy
      elif maxRSSI > -60:
        vx = (1.0-frac)*vx - frac*dx
        vy = (1.0-frac)*vy - frac*dy
      else:
        vx = (1.0-frac)*vx - frac*dy
        vy = (1.0-frac)*vy + frac*dx
    x.append(x[-1]+vx)
    y.append(y[-1]+vy)
    n = n+1
    if n % 500 == 0 and n > 15000:
      if min([dist(x[-1], y[-1], pos[0], pos[1]) for pos in zip(x[:4500], y[:4500])]) < 50.0:
        break
    if n > 150000:
      break
  nvisIdeal = len(visitedNodes)
  durationIdeal = len(x)
  
  vis = float(nvis)/float(nvisIdeal)
  speed = float(duration)/float(durationIdeal)
  return [vis, speed]


if __name__ == "__main__":
  # deal with arguments and parameters
  if len(sys.argv) != 3:
    print "USAGE: "+sys.argv[0]+" #OfWorkerInstances outfile"
    sys.exit()
  nWorkers = int(sys.argv[1])
  pool = multiprocessing.Pool(processes=nWorkers)
  
  minX = 200
  maxX = 3700
  minY = 200
  maxY = 3700
  
  frac = 0.1

  nSamples = 900
  
  data = []
  for dNoise in [0.025*2.0*math.pi, 0.05*2.0*math.pi, 0.075*2.0*math.pi, 0.1*2.0*math.pi, 0.125*2.0*math.pi, 0.15*2.0*math.pi, 0.175*2.0*math.pi, 0.2*2.0*math.pi]:
    for fNoise in [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0]:
      tasks =[[1.0/math.sqrt(2), 1.0/math.sqrt(2.0), 0.0, 0.0, generateNodes(minX, maxX, minY, maxY, 200, 150.0), fNoise, dNoise, frac] for i in range(nSamples)]
      results = pool.map(explore, tasks)
      data.append([fNoise, dNoise, [result[0] for result in results], [result[1] for result in results]])
      cPickle.dump(data, open(sys.argv[2], "wb"))

  cPickle.dump(data, open(sys.argv[2], "wb"))
