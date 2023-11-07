#include <iostream>
#include <fstream>
#include <vector>
#include <tuple>
#include <cmath>
#include <limits>
#include <sstream>
#include <queue>
#include <algorithm>
#include <numeric>
#include <random>

double valorTotalAcumulado = 0.0; // Inicialize com zero

// Função para calcular h
double calcular_h(int i, int j, double dist, const std::vector<std::tuple<double, double, double>>& coord) {
    std::tuple<double, double, double> coord_1 = coord[i];
    std::tuple<double, double, double> coord_2 = coord[j];

    // Verifica se a cidade de partida é diferente da de chegada
    if (i == j) {
        return 0;
    } else {
        double h = pow(1.0 + (1.0 * ((std::get<2>(coord_2) - std::get<2>(coord_1)) / dist)), 2);
        return h;
    }
}

// Função para calcular a
double calcular_a(int i, int j, int k, const std::vector<std::tuple<double, double, double>>& coord) {
    // 500 seria P0, um ponto virtual que garante que "a" sempre será igual a 1
    if (i == j) {
        return 1;
    }

    if (j == k) {
        return 0;
    } else {
        std::tuple<double, double, double> coord_cidade_1 = coord[i];  // Coordenadas da cidade 1
        std::tuple<double, double, double> coord_cidade_2 = coord[j];  // Coordenadas da cidade 2
        std::tuple<double, double, double> coord_cidade_3 = coord[k];  // Coordenadas da cidade 3

        double r = pow(std::get<0>(coord_cidade_1) - std::get<0>(coord_cidade_2), 2) + pow(std::get<1>(coord_cidade_1) - std::get<1>(coord_cidade_2), 2);
        double s = pow(std::get<0>(coord_cidade_2) - std::get<0>(coord_cidade_3), 2) + pow(std::get<1>(coord_cidade_2) - std::get<1>(coord_cidade_3), 2);
        double t = pow(std::get<0>(coord_cidade_3) - std::get<0>(coord_cidade_1), 2) + pow(std::get<1>(coord_cidade_3) - std::get<1>(coord_cidade_1), 2);

        // Lei dos cossenos
        double div = (r + s - t) / sqrt(4 * r * s);
        double ang = M_PI - acos(div);

        double angulo = (180 / M_PI) * ang;

        // Penalização em relação ao ângulo
        double a = pow(1.0 + ((1.0 * angulo) / 180.0), 2);

        return a;
    }
}

// Função para imprimir o caminho mínimo
void print_path(const std::vector<int>& parent, int v, std::ofstream& output) {
    if (parent[v] == -1) {
        output << v << " ";
        return;
    }
    print_path(parent, parent[v], output);
    output << v << " ";
}

// Função para encontrar o próximo ponto mais próximo que não tenha sido visitado
int encontrarProximoPonto(int atual, const std::vector<std::vector<double>>& distancias, std::vector<bool>& visitado, int pontoInicial, std::vector<int> ciclo) {
    int n = distancias.size();
    int proximoPonto = -1;
    double menorDistancia = std::numeric_limits<double>::max();

    for (int i = 0; i < n; i++) {
        if (i != atual && !visitado[i] && distancias[atual][i] < menorDistancia) {
            proximoPonto = i;
            menorDistancia = distancias[atual][i];
        }
    }

    return proximoPonto;
}

// Função para encontrar o ciclo hamiltoniano usando KNN
std::vector<int> encontrarCicloHamiltoniano(const std::vector<std::vector<double>>& distancias) {
    int n = distancias.size();
    std::vector<bool> visitado(n, false);
    std::vector<int> cicloHamiltoniano;

    int pontoAtual = 0;
    int pontoat = 168;
    int pontoini = pontoat;
    int pontoInicial = pontoAtual;

    cicloHamiltoniano.push_back(pontoAtual);
    visitado[pontoAtual] = true;
    // tentar incluir a penalizacao por angulo aqui
    for (int i = 1; i < n/2; i++) {
        int proximoPonto = encontrarProximoPonto(pontoAtual, distancias, visitado, pontoInicial, cicloHamiltoniano);

        if (proximoPonto == -1) {
            break;
        }

        valorTotalAcumulado += distancias[pontoAtual][proximoPonto];
        cicloHamiltoniano.push_back(proximoPonto);
        visitado[proximoPonto] = true;
        pontoAtual = proximoPonto;
    }


    cicloHamiltoniano.push_back(pontoat);
    visitado[pontoat] = true;
    for (int i = n/2; i < n; i++) {
        int proximoPonto = encontrarProximoPonto(pontoat, distancias, visitado, pontoini, cicloHamiltoniano);

        if (proximoPonto == -1) {
            break;
        }

        valorTotalAcumulado += distancias[pontoat][proximoPonto];
        cicloHamiltoniano.push_back(proximoPonto);
        visitado[proximoPonto] = true;
        pontoat = proximoPonto;
    }
    std::reverse(cicloHamiltoniano.begin() + n/2, cicloHamiltoniano.end());
    //cicloHamiltoniano.push_back(pontoInicial);

    return cicloHamiltoniano;
}

std::pair<std::vector<int>, double> OPT(std::vector<int> rota, std::vector<std::vector<double>>& distancias, double dist){
    double ganhoMax = 0;
    std::pair<std::vector<int>, double> melhor;
    melhor.first = rota;
    melhor.second = dist;
    std::vector<int> bestV(4);
    int n = rota.size()-1;
    int limc;

    for(int a = 0; a < n-3; a++){
        int b = a+1;
        limc = n-1;
        for(int c = a+2; c < limc; c++){
            int d=c+1;
            //if(distancias[rota[a]][rota[b]] < 30 && distancias[rota[c]][rota[d]] < 30 && distancias[rota[a]][rota[c]] < 30 && distancias[rota[b]][rota[d]] < 30);
            double ganho = (distancias[rota[a]][rota[b]] + distancias[rota[c]][rota[d]]) - (distancias[rota[a]][rota[c]] + distancias[rota[b]][rota[d]]);
            if(ganho > ganhoMax){
                ganhoMax = ganho;
                bestV = {a, b, c, d};
                std::cout << rota[a] << "  " << rota[b]<< "  " << rota[c] << "  " << rota[d] << "  " << ganho << std::endl;
            }
        }
    }
    if(ganhoMax > 0){
        melhor.second = dist - ganhoMax;
        melhor.first[bestV[1]] = rota[bestV[2]];
        melhor.first[bestV[2]] = rota[bestV[1]];
    }
    
    return melhor;
}

std::pair<std::vector<int>, double> Local_Search(std::vector<int> rota,std::vector<std::vector<double>>& distancias, double dist){
    bool melhora = true;
    std::pair<std::vector<int>, double> melhorRota;
    while(melhora){
        melhora = false;
        melhorRota = OPT(rota, distancias, dist);
        std::cout << melhorRota.second << "  " << dist << std::endl;
        if(melhorRota.second < dist){
            rota = melhorRota.first;
            dist = melhorRota.second;
            melhora = true;
        }
    }
    return melhorRota;
}

int main() {
    std::string mapas = "mapas.txt";

    // Abrir o arquivo para leitura
    std::ifstream arquivo(mapas);
    if (!arquivo.is_open()) {
        std::cerr << "Erro ao abrir o arquivo." << std::endl;
        return 1;
    }

    std::string dados;
    std::string linha;
    while (std::getline(arquivo, linha)) {
        dados += linha + "\n";
    }

    arquivo.close();

    // Separa a string em linhas
    size_t pos = 0;
    std::vector<std::string> linhas;
    while ((pos = dados.find("\n")) != std::string::npos) {
        linhas.push_back(dados.substr(0, pos));
        dados.erase(0, pos + 1);
    }

    // Converte as linhas em uma lista de tuplas
    std::vector<std::tuple<double, double, double>> coord;
    for (const std::string& linha : linhas) {
        std::istringstream iss(linha);
        std::string cidade;
        double x, y, z;
        iss >> cidade >> x >> y >> z;
        coord.push_back(std::make_tuple(x, y, z));
    }

    int n = coord.size();  // número de células (pontos)

    std::vector<std::tuple<double, double, double>> obstaculos;
    std::vector<int> obstaculos_indices = {33, 34, 153, 154, 67, 80, 82, 83, 84, 85, 132, 133, 134, 25, 97, 27, 28, 63, 64, 113, 114, 115, 116, 81, 54, 55, 56, 57, 23, 24};
    for (int i : obstaculos_indices) {
        obstaculos.push_back(coord[i]);
    }

    std::vector<int> validos;
    for (int i = 0; i < n; i++) {
        if (std::find(obstaculos_indices.begin(), obstaculos_indices.end(), i) == obstaculos_indices.end()) {
            validos.push_back(i);
        }
    }

    int ini = 0;
    int fin = 168;

    // Cria a matriz de custos
    std::vector<std::vector<double>> c(n, std::vector<double>(n, std::numeric_limits<double>::infinity()));
    for (int i : validos) {
        for (int j : validos) {
            if (i != j) {
                int dist = std::sqrt(std::pow(std::get<0>(coord[i]) - std::get<0>(coord[j]), 2) + std::pow(std::get<1>(coord[i]) - std::get<1>(coord[j]), 2));
                if (dist > 29) { // Distancia máxima entre vértices adjacentes
                    //printf("Distancia entre %d e %d: %f\n", i, j, c[i][j]);
                    continue;  // Então se i e j não forem adjacentes, c[i][j] = inf
                } else {
                    c[i][j] = dist;
                }    //printf("Distancia entre %d e %d: %f\n", i, j, c[i][j]);
            }
        }
    }

    // Matrizes que guardam a penalidade de altitude e ângulo
    std::vector<std::vector<double>> altitudes(n, std::vector<double>(n, 0.0));
    std::vector<std::vector<std::vector<double>>> q(n, std::vector<std::vector<double>>(n, std::vector<double>(n, 0.0)));
    std::vector<std::vector<double>> distancias(n, std::vector<double>(n, std::numeric_limits<double>::infinity()));

    std::ofstream pathFile("path.txt");

    // Função para calcular o caminho mínimo usando o algoritmo de Dijkstra com heap
    auto dijkstra = [&](const std::vector<std::vector<double>>& c, int i, int j) {
        std::vector<double> dist(n, std::numeric_limits<double>::infinity());
        std::vector<int> parent(n, -1);
        dist[i] = 0.0;
        std::vector<double> alt(n, 0.0);
        std::vector<bool> spt_set(n, false);

        // Usar um heap para manter os vértices não processados com base em suas distâncias
        std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> heap;
        heap.push(std::make_pair(0.0, i));

        while (!heap.empty()) {
            double dist_u = heap.top().first;
            int u = heap.top().second;
            heap.pop();

            if (spt_set[u]) {
                continue;
            }
            spt_set[u] = true;

            for (int v : validos) {
                if (u != v && !spt_set[v] && c[u][v] > 0 && dist[u] + c[u][v] < dist[v]) {
                    double altitude = calcular_h(u, v, (dist[u] + c[u][v]), coord);
                    double angulos = calcular_a((parent[u] == -1) ? u : parent[u], u, v, coord);
                    alt[v] = alt[u] + angulos;
                    dist[v] = dist[u] + (c[u][v] * angulos * altitude);
                    if (parent[u] != -1) {
                        q[parent[u]][u][v] = angulos;
                    } else {
                        q[u][u][v] = angulos;
                    }
                    parent[v] = u;
                    heap.push(std::make_pair(dist[v], v));
                }
            }
        }

        //if (c[i][j] != std::numeric_limits<double>::infinity()) {
        //    distancias[i][j] = c[i][j];
        //} else {
        distancias[i][j] = dist[j];
        //}
        altitudes[i][j] = alt[j];
        pathFile << "Caminho de " << i << " para " << j << ": ";
        print_path(parent, j, pathFile);
        pathFile << "\n";
        std::cout << std::endl;
    };

    for (int i : validos) {
        for (int j : validos) {
            dijkstra(c, i, j);
        }
    }
    
    std::vector<int> cicloHamiltoniano = encontrarCicloHamiltoniano(distancias);
    //std::pair<std::vector<int>, double> melhorRota = Local_Search(cicloHamiltoniano, distancias, valorTotalAcumulado);
    //cicloHamiltoniano = melhorRota.first;

    std::random_device rd;
    std::mt19937 g(rd());

    //std::shuffle(cicloHamiltoniano.begin(), cicloHamiltoniano.end(), g);
    //cicloHamiltoniano = {17, 165, 144, 52, 126, 16, 6, 71, 72, 149, 131, 164, 18, 20, 98, 14, 8, 1, 92, 39, 112, 152, 78, 147, 155, 91, 127, 122, 100, 117, 105, 49, 37, 79, 129, 111, 76, 13, 51, 140, 35, 88, 59, 41, 77, 66, 120, 93, 101, 162, 150, 50, 65, 10, 108, 69, 136, 96, 19, 158, 31, 45, 48, 32, 58, 168, 60, 22, 26, 40, 95, 90, 53, 143, 166, 142, 47, 128, 135, 3, 125, 94, 160, 148, 2, 106, 139, 42, 118, 87, 103, 86, 38, 12, 70, 110, 46, 4, 43, 15, 167, 29, 7, 75, 130, 109, 163, 99, 89, 124, 161, 137, 151, 159, 0, 9, 5, 157, 61, 119, 141, 138, 146, 102, 73, 123, 21, 68, 74, 104, 36, 156, 145, 44, 11, 121, 30, 62, 107};

    //std::pair<std::vector<int>, double> melhorRota = Local_Search(cicloHamiltoniano, distancias, valorTotalAcumulado);
    //cicloHamiltoniano = melhorRota.first;

    //caminho encontrado pelo modelo
    //[1, 2, 3, 16, 15, 14, 27, 40, 53, 66, 79, 92, 93, 80, 67, 54, 41, 42, 43, 30, 17, 4, 5, 6, 7, 8, 9, 10, 12, 13, 11, 23, 22, 21, 20, 19, 18, 31, 44, 45, 32, 33, 46, 47, 60, 59, 71, 69, 70, 72, 73, 74, 75, 62, 61, 48, 49, 36, 37, 38, 39, 52, 51, 50, 63, 76, 89, 90, 77, 78, 91, 104, 103, 102, 101, 88, 87, 100, 99, 112, 125, 124, 111, 110, 97, 96, 95, 94, 107, 106, 105, 118, 131, 144, 157, 158, 159, 160, 147, 146, 145, 132, 119, 120, 121, 108, 109, 122, 123, 136, 149, 148, 161, 162, 163, 164, 151, 150, 137, 138, 139, 126, 113, 127, 128, 129, 130, 143, 142, 141, 140, 153, 152, 165, 166, 167, 168, 169, 156]
    //std::vector<int> fo(validos.size());
    //fo = {1, 2, 3, 16, 15, 14, 27, 40, 53, 66, 79, 92, 93, 80, 67, 54, 41, 42, 43, 30, 17, 4, 5, 6, 7, 8, 9, 10, 12, 13, 11, 23, 22, 21, 20, 19, 18, 31, 44, 45, 32, 33, 46, 47, 60, 59, 71, 69, 70, 72, 73, 74, 75, 62, 61, 48, 49, 36, 37, 38, 39, 52, 51, 50, 63, 76, 89, 90, 77, 78, 91, 104, 103, 102, 101, 88, 87, 100, 99, 112, 125, 124, 111, 110, 97, 96, 95, 94, 107, 106, 105, 118, 131, 144, 157, 158, 159, 160, 147, 146, 145, 132, 119, 120, 121, 108, 109, 122, 123, 136, 149, 148, 161, 162, 163, 164, 151, 150, 137, 138, 139, 126, 113, 127, 128, 129, 130, 143, 142, 141, 140, 153, 152, 165, 166, 167, 168, 169, 156};
    //fo = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 13, 12, 11, 23, 22, 21, 20, 33, 32, 19, 18, 17, 16, 15, 14, 27, 40, 53, 66, 67, 54, 41, 42, 43, 30, 31, 44, 45, 46, 47, 48, 49, 36, 37, 38, 39, 52, 51, 50, 63, 62, 61, 60, 59, 71, 70, 69, 72, 73, 74, 87, 88, 75, 76, 77, 78, 91, 104, 103, 90, 89, 102, 101, 100, 99, 112, 113, 126, 125, 124, 111, 110, 97, 96, 109, 108, 95, 94, 93, 80, 79, 92, 105, 106, 107, 120, 121, 122, 123, 136, 137, 138, 139, 140, 127, 128, 129, 130, 143, 142, 141, 153, 152, 151, 150, 149, 148, 147, 146, 145, 132, 119, 118, 131, 144, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 156};
    //double totalfo = 0;
    //for(int i = 0; i < fo.size() - 1; i++)
    //    totalfo += distancias[fo[i]-1][fo[i+1]-1];

    //std::cout << "Total Gurobi: " << totalfo << std::endl;

    double total = 0;
    std::cout << valorTotalAcumulado << std::endl;
    for(int i = 0; i < cicloHamiltoniano.size() - 1; i++)
        total += distancias[cicloHamiltoniano[i]][cicloHamiltoniano[i+1]];

    valorTotalAcumulado = total;

    pathFile.close();

    int l =1;
    // Imprima o caminho hamiltoniano
    std::cout << "Ciclo Hamiltoniano: " << std::endl;
    for (int vertice : cicloHamiltoniano) {
        std::cout << vertice << ", ";
        if(l%13 == 0){
        std::cout << std::endl;
        l++;
        }
    }
    std::cout << std::endl;

    std::ofstream resultadosFile("resultados.txt");

    if (resultadosFile.is_open()) {
        resultadosFile << "Coordenadas:" << std::endl;
        for (const auto& tuple : coord) {
        resultadosFile << std::get<0>(tuple) << " " << std::get<1>(tuple) << " " << std::get<2>(tuple) << std::endl;
        }

        resultadosFile << "Ciclo Hamiltoniano:" << std::endl;
        for (int vertex : cicloHamiltoniano) {
            resultadosFile << vertex << " ";
        }
        resultadosFile << std::endl;

        resultadosFile << "Obstaculos:" << std::endl;
        for (const auto& obstacle : obstaculos) {
            resultadosFile << std::get<0>(obstacle) << " " << std::get<1>(obstacle) << std::endl;
        }

        resultadosFile << "Valor Total do Ciclo: " << valorTotalAcumulado << std::endl;

        resultadosFile.close();
    } else {
        std::cerr << "Erro ao abrir o arquivo resultados.txt para escrita." << std::endl;
        return 1;
    }

    return 0;
}
