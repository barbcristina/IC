import numpy as py
import gurobipy as gp
from gurobipy import GRB
import math
import heapq
from numpy import ubyte
import time

inicio = time.time()
for i in range(0, 1):
  mapas = '120_pontos/mapas15x8.txt'

  # Abrir o arquivo para leitura
  with open(mapas, 'r') as arquivo:
    dados = arquivo.read()

  # Separa a string em linhas
  linhas = dados.strip().split('\n')

  # Converte as linhas em uma lista de tuplas
  coord = [tuple(linha.split()) for linha in linhas[:-1]]  # Exclui a última linha

  n = len(coord)  # número de células (pontos)

  coord = [(float(x), float(y), float(z)) for cidade, x, y, z in coord]

  obstaculos_indices = [int(celula) for celula in linhas[-1].split()]  # Lista de células a serem evitadas
  print("Obstáculos: ", obstaculos_indices)
  obstaculos = [coord[i] for i in obstaculos_indices]
  validos = [coord.index(i) for i in coord if coord.index(i) not in obstaculos_indices]

  ini = 0
  fin = 1

  # Cria a matriz de custos
  c = [[float('inf')] * n for _ in range(n)]
  for i in validos:
      for j in validos:
        if i != j:
          dist = int(math.sqrt((coord[i][0] - coord[j][0]) ** 2 + (coord[i][1] - coord[j][1]) ** 2 + (coord[i][2] - coord[j][2]) ** 2))
          if dist > 28: # Distancia maxima entre vértices adjacentes
            continue  # Então se i e j nào forem adjacentes, c[i][j] = inf
          else:
            c[i][j] = dist + 0.5

  # Ângulo do caminho entre pontos
  def calcular_a(i, j, k):

    # 500 seria P0, um ponto virtual que garante que "a" sempre será igual a 1
    if i == j:
      return 0

    if j == k:
      return 0
    else:
      coord_cidade_1 = coord[i]  # Coordenadas da cidade 1
      coord_cidade_2 = coord[j]  # Coordenadas da cidade 2
      coord_cidade_3 = coord[k]  # Coordenadas da cidade 3

      #print("Coordenadas da Cidade 1:", coord_cidade_1)
      #print("Coordenadas da Cidade 2:", coord_cidade_2)
      #print("Coordenadas da Cidade 3:", coord_cidade_3)

      # Fórmula para encontrar ângulo dado 3 pontos e suas coordenadas
      r = pow((coord_cidade_1[0] - coord_cidade_2[0]), 2) + pow((coord_cidade_1[1] - coord_cidade_2[1]), 2)
      s = pow((coord_cidade_2[0] - coord_cidade_3[0]), 2) + pow((coord_cidade_2[1] - coord_cidade_3[1]), 2)
      t = pow((coord_cidade_3[0] - coord_cidade_1[0]), 2) + pow((coord_cidade_3[1] - coord_cidade_1[1]), 2)

      # Lei dos cossenos
      div = (r + s - t)/math.sqrt(4*r*s)
      ang = math.pi - math.acos(div)

      angulo = (0.2 * (180/math.pi) * ang)
      #print("angulo entre: ", i, " ", j, " ",k, " = ", angulo)

      return angulo
    
  def calcular_h(i, j, dist):
      coord_1 = coord[i]
      coord_2 = coord[j]

      # Verifica se a cidade de partida é diferente da de chegada
      if i==j:
          return 0
      else:
          # k = 1 enquanto não for decidido a penalidade
          h = (2 * ((coord_2[2] - coord_1[2])) / dist)
          h = math.sqrt(h ** 2)

          return h

  # Matrizes que guardam a penalidade de altitude e ângulo
  altitudes = [[0] * n for _ in range(n)]
  q = [[[0] * n for _ in range(n)] for _ in range(n)]
  distancias = [[float('inf')] * n for _ in range(n)]

  # Função para imprimir o caminho mínimo
  def print_path(parent, v):
      if parent[v] == None:
          print(v, end=' ')
          return
      print_path(parent, parent[v])
      print(v, end=' ')

  # Função para calcular o caminho mínimo usando o algoritmo de Dijkstra com heap
  def dijkstra(c, i, j):
      dist = [float('inf')] * n
      parent = [None] * n
      dist[i] = 0
      alt = [0] * n
      spt_set = [False] * n

      heap = [(0, i)]

      while heap:
          _, u = heapq.heappop(heap)
          if spt_set[u]:
              continue
          spt_set[u] = True

          for v in validos:
              if u != v and not spt_set[v] and c[u][v] > 0 and dist[u] + c[u][v] < dist[v]:
                  altitude = calcular_h(u, v, (dist[u] + c[u][v]))
                  angulos = calcular_a(parent[u] if parent[u] is not None else u, u, v)
                  alt[v] = alt[u] + altitude
                  dist[v] = dist[u] + (c[u][v] + angulos)
                  parent[v] = u
                  heapq.heappush(heap, (dist[v], v))
      if c[i][j] != float('inf'):
        distancias[i][j] = c[i][j]
      else:
        distancias[i][j] = dist[j]
      altitudes[i][j] = alt[j]

  #Caminho com o modelo já atualizado
  route = [0]

  import matplotlib.pyplot as plt

  xx, yy, zz = zip(*coord)

  plt.figure(figsize=(8,15))

  plt.scatter(xx, yy, color="black") #plota vertices

  max_x = max(xx)
  max_y = max(yy)
  plt.axis([0, max_x+10, 0, max_y+10])

  plt.xticks(range(0, int(max_x) + 1, 20))
  plt.yticks(range(0, int(max_y) + 1, 20))

  plt.grid(True, color = "blue", linestyle = "-", linewidth = 1.0, alpha = 0.25)
  #plt.grid(True)

  for i in obstaculos:
      # Defina os vértices do polígono para a célula
      vertices = [((i[0]-10), (i[1]-10)), ((i[0]-10) + 20, (i[1]-10)), ((i[0]-10) + 20, (i[1]-10) + 20), ((i[0]-10), (i[1]-10) + 20)]
      # Crie um polígono a partir dos vértices e pinte-o de vermelho
      poly = plt.Polygon(vertices, facecolor='red')
      plt.gca().add_patch(poly)


  for i, coordxy in enumerate(zip(xx, yy)):
      plt.annotate(str(i+1), xy=coordxy, xytext=(2, 2), textcoords='offset points')

  x_ordered = [xx[i] for i in route]
  y_ordered = [yy[i] for i in route]

  # x_ordered.append(xx[route[0]-1])
  # y_ordered.append(yy[route[0]-1])

  plt.plot(x_ordered, y_ordered, color="green") #plota arestas

  #plt.title(f"Cost = {modelo.objVal:.2f}")

  plt.savefig('100_pontos/gurobi100.png')

  #829.24