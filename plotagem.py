import matplotlib.pyplot as plt
# Leitura dos resultados do arquivo de texto
with open('resultados.txt', 'r') as arquivo:
    linhas = arquivo.readlines()

coord = []
caminhoHamiltoniano = []
obstaculos = []  # Lista de obstáculos

lendoCoordenadas = False
lendoCaminhoHamiltoniano = False
lendoObstaculos = False

for linha in linhas:
    if linha.startswith("Coordenadas:"):
        lendoCoordenadas = True
    elif linha.startswith("Ciclo Hamiltoniano:"):
        lendoCaminhoHamiltoniano = True
        lendoCoordenadas = False
    elif linha.startswith("Obstaculos:"):
        lendoObstaculos = True
        lendoCaminhoHamiltoniano = False
        lendoCoordenadas = False
    elif linha.startswith("Valor Total do Ciclo:"):
        lendoObstaculos = False
        custo_total = float(linha.split(": ")[1])
    elif lendoCoordenadas:
        x, y, z = map(float, linha.split())
        coord.append((x, y, z))
    elif lendoCaminhoHamiltoniano:
        caminhoHamiltoniano = list(map(int, linha.split()))
    elif lendoObstaculos:
        x, y = map(float, linha.split())
        obstaculos.append((x, y))

# Leitura do arquivo contendo todos os caminhos
with open('path.txt', 'r') as allPathsFile:
    all_paths = allPathsFile.readlines()

# Encontrar caminhos usados no ciclo Hamiltoniano
caminhos_usados = []

for i in range(len(caminhoHamiltoniano) - 1):
    vertex_start = caminhoHamiltoniano[i]
    vertex_end = caminhoHamiltoniano[i + 1]
    path_found = False

    for caminho in all_paths:
        partes = caminho.split(": ")
        if len(partes) == 2:
            vertices = list(map(int, partes[1].split()))
            if vertices[0] == vertex_start and vertices[-1] == vertex_end:
                caminhos_usados.append(caminho)
                path_found = True
                break

    if not path_found:
        print(f"Caminho de {vertex_start} para {vertex_end} não encontrado.")

# Cria um novo arquivo para os caminhos usados no ciclo Hamiltoniano
with open('used_paths.txt', 'w') as usedPathsFile:
    usedPathsFile.writelines(caminhos_usados)

xx, yy, zz = zip(*coord)

plt.figure(figsize=(15, 15))

plt.scatter(xx, yy, color="black")  # Plota vértices

max_x = max(xx)
max_y = max(yy)
plt.axis([0, max_x + 10, 0, max_y + 10])

plt.xticks(range(0, int(max_x) + 1, 20))
plt.yticks(range(0, int(max_y) + 1, 20))

plt.grid(True, color="blue", linestyle="-", linewidth=1.0, alpha=0.25)

for i in obstaculos:
    # Define os vértices do polígono para a célula
    vertices = [((i[0] - 10), (i[1] - 10)), ((i[0] - 10) + 20, (i[1] - 10)),
                ((i[0] - 10) + 20, (i[1] - 10) + 20), ((i[0] - 10), (i[1] - 10) + 20)]
    # Cria um polígono a partir dos vértices e pinte-o de vermelho
    poly = plt.Polygon(vertices, facecolor='red')
    plt.gca().add_patch(poly)

for i, coordxy in enumerate(zip(xx, yy)):
    plt.annotate(str(i + 1), xy=coordxy, xytext=(2, 2), textcoords='offset points')

# Leitura dos caminhos usados no ciclo Hamiltoniano a partir do arquivo
used_paths = []
with open('used_paths.txt', 'r') as usedPathsFile:
    used_paths = usedPathsFile.readlines()

# Plota as arestas dos caminhos usados no ciclo Hamiltoniano
for path in used_paths:
    if path.startswith("Caminho de"):
        vertices = list(map(int, path.split(": ")[1].split()))
        x_ordered = [xx[i] for i in vertices]
        y_ordered = [yy[i] for i in vertices]
        plt.plot(x_ordered, y_ordered, color="green")

# Conecta os vértices do caminho Hamiltoniano
for i in range(len(x_ordered) - 1):
    plt.plot(x_ordered[i:i + 2], y_ordered[i:i + 2], color="green")

plt.title(f"Custo Total do Ciclo: {custo_total:.2f}")

plt.savefig("grasp_IMB_121_10.png")