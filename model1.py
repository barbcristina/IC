import numpy as py
import gurobipy as gp
from gurobipy import GRB
import math
import heapq
from numpy import ubyte
import time

mapaqtd = 16 #5

for i in range(0, 1):
  mapaqtd += 1
  mapas = f'{mapaqtd*mapaqtd}_pontos/mapas{mapaqtd}.txt'

  # Abrir o arquivo para leitura
  with open(mapas, 'r') as arquivo:
    dados = arquivo.read()

  # Separa a string em linhas
  linhas = dados.strip().split('\n')
  qtdobs = len(linhas) - (mapaqtd*mapaqtd) # Quantidade de obstáculos

  # Converte as linhas em uma lista de tuplas
  coord = [tuple(linha.split()) for linha in linhas[:-qtdobs]]  # Exclui as linhas de obstáculos

  n = len(coord)  # número de células (pontos)

  coord = [(float(x), float(y), float(z)) for cidade, x, y, z in coord]

  versao = 0 # Versão do arquivo

  for k in range(0, 1):
    inicio = time.time()
    obstaculos_indices = [int(celula) for celula in linhas[-qtdobs+0].split()]  # Lista de células a serem evitadas
    print("Obstáculos: ", obstaculos_indices)

    obstaculos = [coord[i] for i in obstaculos_indices] # Coordenadas dos obstáculos
    validos = [coord.index(i) for i in coord if coord.index(i) not in obstaculos_indices] # Lista de vértices válidos

    ini = 0 # Ponto inicial
    fin = 1 # Ponto final

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
    
    maiorx = (coord[-1][0] + 10) // 20
    maiory = (coord[-1][1] + 10) // 20
    print(maiorx, maiory)

    for i in validos:
      for j in validos:
        if i-1 in obstaculos_indices and j+1 in obstaculos_indices and (j == (i+maiorx+1) or i == (j-maiorx-1)) and (i-1)//maiorx == i//maiorx and (j+1)//maiorx == j//maiorx:
          c[i][j] = float('inf')
          c[j][i] = float('inf')
        elif i+1 in obstaculos_indices and j-1 in obstaculos_indices and (j == (i+maiorx+1) or i == (j-maiorx-1)) and (i+1)//maiorx == i//maiorx and (j-1)//maiorx == j//maiorx:
          c[i][j] = float('inf')
          c[j][i] = float('inf')

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
    # Matrizes que guardam a penalidade de altitude e ângulo
    altitudes = [[0] * n for _ in range(n)]
    distancias = [[float('inf')] * n for _ in range(n)]
    all_angles = [[0] * n for _ in range(n)]
    matrix = [[0] * n for _ in range(n)]

    # Função para imprimir o caminho mínimo
    def print_and_return_path(parent, v):
        path = []
        if parent[v] is None:
            return [v]
        path.extend(print_and_return_path(parent, parent[v]))
        path.append(v)
        return path

    # Função para calcular o caminho mínimo usando o algoritmo de Dijkstra com heap
    def dijkstra(c, i, j):
        dist = [float('inf')] * n
        parent = [None] * n
        dist[i] = 0
        alt = [0] * n
        angles = [0] * n
        spt_set = [False] * n

        heap = [(0, i)]

        while heap:
            _, u = heapq.heappop(heap)
            if spt_set[u]:
                continue
            spt_set[u] = True

            for v in validos:
                if u != v and not spt_set[v] and c[u][v] > 0 and dist[u] + c[u][v] < dist[v]: #
                    altitude = calcular_h(u, v, (dist[u] + c[u][v]))
                    angulos = calcular_a(parent[u] if parent[u] is not None else u, u, v)
                    alt[v] = alt[u] + altitude
                    angles[v] = angles[u] + angulos
                    dist[v] = dist[u] + c[u][v]
                    parent[v] = u
                    heapq.heappush(heap, (dist[v], v))
        if c[i][j] != float('inf'):
          distancias[i][j] = c[i][j]
        else:
          distancias[i][j] = dist[j]
        altitudes[i][j] = alt[j]
        all_angles[i][j] = angles[j]
        matrix[i][j] = print_and_return_path(parent, j)

    # Chamar o algoritmo de Dijkstra para variar nos vértices válidos
    for i in validos:
        for j in validos:
            dijkstra(c, i, j)

    q = [[[0] * n for _ in range(n)] for _ in range(n)]

    for i in validos:
        for j in validos:
          for k in validos:
            if c[i][j] != float('inf') and c[j][k] != float('inf'):
              q[i][j][k] = calcular_a(i, j, k)

    for i in validos:
      for j in validos:
        for k in validos:
          if (c[i][j] == float('inf') and c[j][k] == float('inf')) and i!=j and j!=k:
            q[i][j][k] = (all_angles[i][j] + all_angles[j][k]) + (q[matrix[i][j][len(matrix[i][j])-2]][j][matrix[j][k][1]])
          elif (c[i][j] == float('inf') and c[j][k] != float('inf')) and i!=j and j!=k:
            q[i][j][k] = (all_angles[i][j] + all_angles[j][k]) + q[matrix[j][k][1]][j][k]
          elif (c[i][j] != float('inf') and c[j][k] == float('inf')) and i!=j and j!=k:
            q[i][j][k] = (all_angles[i][j] + all_angles[j][k]) + q[i][j][matrix[i][j][len(matrix[i][j])-2]]
          

    route = [0, 1, 288, 138, 139, 137, 142, 136, 143, 135, 144, 134, 146, 133, 147, 132, 148, 130, 149, 129, 150, 127, 152, 126, 156, 125, 157, 124, 159, 123, 160, 122, 161, 121, 163, 120, 164, 119, 166, 118, 168, 117, 169, 116, 172, 115, 173, 114, 174, 113, 176, 112, 177, 111, 178, 110, 180, 109, 181, 108, 182, 107, 183, 106, 184, 105, 186, 104, 187, 103, 189, 99, 190, 98, 192, 97, 193, 96, 194, 95, 195, 94, 196, 92, 197, 91, 200, 90, 202, 89, 203, 88, 204, 87, 205, 86, 206, 85, 207, 83, 208, 82, 209, 81, 210, 78, 211, 77, 213, 76, 214, 75, 215, 74, 218, 73, 219, 72, 220, 71, 221, 69, 222, 68, 224, 67, 226, 66, 227, 64, 228, 63, 229, 62, 230, 61, 231, 60, 232, 59, 233, 56, 234, 55, 236, 53, 237, 51, 238, 50, 239, 49, 241, 48, 242, 46, 243, 45, 244, 44, 245, 42, 246, 40, 247, 39, 248, 38, 249, 36, 250, 35, 251, 34, 252, 33, 253, 32, 254, 31, 255, 30, 256, 29, 257, 26, 258, 24, 259, 23, 260, 22, 262, 21, 263, 19, 264, 17, 266, 16, 267, 15, 268, 14, 269, 13, 270, 12, 271, 11, 272, 10, 273, 9, 274, 8, 275, 7, 276, 6, 283, 5, 284, 4, 285, 3, 286, 2, 287, 0]
    total = 0
    for i in range(0, len(route)-1):
      total += distancias[route[i]][route[i+1]] + altitudes[route[i]][route[i+1]] + q[route[i-1 if i>0 else 0]][route[i+1]][route[i+1]]

    print("Custo total: ", total)
    

    """
    import matplotlib.pyplot as plt

    xx, yy, zz = zip(*coord)

    plt.figure(figsize=(12,12))

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

    x_ordered = [xx[i-1] for i in route]
    y_ordered = [yy[i-1] for i in route]

    # x_ordered.append(xx[route[0]-1])
    # y_ordered.append(yy[route[0]-1])

    plt.plot(x_ordered, y_ordered, color="green") #plota arestas

    plt.title(f"Cost = {modelo.objVal:.2f}")
    #versao += 1 # Incrementa a versão
    plt.savefig(f'{mapaqtd*mapaqtd}_pontos/gurobi{mapaqtd*mapaqtd}_{versao}.png') # Salva a imagem do grafo
    print(f'resolvida a versao {mapaqtd*mapaqtd}_{versao}') 
    """
    versao += 1 # Incrementa a versão