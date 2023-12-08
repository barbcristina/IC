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
#include <chrono>

auto start_time = std::chrono::steady_clock::now();

double valorTotalAcumulado = 0.0; // Inicialize com zero

// Função para calcular h
double calcular_h(int i, int j, double dist, const std::vector<std::tuple<double, double, double>>& coord) {
    std::tuple<double, double, double> coord_1 = coord[i];
    std::tuple<double, double, double> coord_2 = coord[j];

    // Verifica se a cidade de partida é diferente da de chegada
    if (i == j) {
        return 0;
    } else {
        double h = 2 * ((std::get<2>(coord_2) - std::get<2>(coord_1)) / dist);
        h = std::pow(h,2);
        h = std::sqrt(h);

        return h;
    }
}

// Função para calcular a
double calcular_a(int i, int j, int k, const std::vector<std::tuple<double, double, double>>& coord) {
    // 500 seria P0, um ponto virtual que garante que "a" sempre será igual a 1

    if (j == k or i == j) {
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

        double angulo = 3 * (180 / M_PI) * ang;

        return angulo;
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

std::pair<std::vector<int>, double> OPT2(std::vector<int> rota, std::vector<std::vector<double>>& distancias, double dist, std::vector<std::vector<std::vector<double>>> q,  std::vector<std::vector<double>> altitudes){
    double ganhoMax = 0;
    std::pair<std::vector<int>, double> melhor;
    melhor.first = rota;
    melhor.second = dist;
    std::vector<int> bestV(4);
    int n = rota.size()-1;
    int limc;

    for(int a = 0; a < n-2; a++){
        int b = a+1;
        if(a!=0)
        limc = n;
        else
        limc = n-1;
        for(int c = a+2; c < limc; c++){
            int d;
            if(c!=n)
            d=c+1;
            else
            d=0;
            //+ q[rota[b]][rota[d]][(d == n) ? rota[d] : rota[d+1]]
            //if(distancias[rota[a]][rota[b]] < 30 && distancias[rota[c]][rota[d]] < 30 && distancias[rota[a]][rota[c]] < 30 && distancias[rota[b]][rota[d]] < 30);
            double ganho = (distancias[rota[a]][rota[b]] + altitudes[rota[a]][rota[b]] + distancias[rota[c]][rota[d]] + altitudes[rota[c]][rota[d]]
            + q[rota[(a != 0) ? a-1 : d]][rota[a]][rota[b]] + q[rota[a]][rota[b]][rota[b+1]] + q[rota[c]][rota[d]][rota[(c!=n) ? d+1 : 0]] + q[rota[c-1]][rota[c]][rota[d]]) 
            - (distancias[rota[a]][rota[c]] + altitudes[rota[a]][rota[c]] + distancias[rota[b]][rota[d]] + altitudes[rota[b]][rota[d]] 
            + q[rota[(a != 0) ? a-1 : d]][rota[a]][rota[c]] + q[rota[d]][rota[b]][rota[b+1]] + q[rota[b]][rota[d]][rota[(c!=n) ? d+1 : 0]] + q[rota[c-1]][rota[c]][rota[a]]);
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

std::pair<std::vector<int>, double> OPT(std::vector<int> rota, std::vector<std::vector<double>>& distancias, double dist, std::vector<std::vector<std::vector<double>>> q,  std::vector<std::vector<double>> altitudes){
    double ganhoMax = 0;
    std::pair<std::vector<int>, double> melhor;
    melhor.first = rota;
    melhor.second = dist;
    std::vector<int> bestV(4);
    int n = rota.size()-1;
    int limc;

    for(int a = 0; a < n-2; a++){
        int b = a+1;
        if(a!=0)
        limc = n;
        else
        limc = n-1;
        for(int c = a+2; c < limc; c++){
            int d;
            if(c!=n)
            d=c+1;
            else
            d=0;
            //+ q[rota[b]][rota[d]][(d == n) ? rota[d] : rota[d+1]]
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

std::pair<std::vector<int>, double> Local_Search(std::vector<int> rota,std::vector<std::vector<double>>& distancias, double dist, std::vector<std::vector<std::vector<double>>> q,  std::vector<std::vector<double>> altitudes){
    bool melhora = true;
    std::pair<std::vector<int>, double> melhorRota;
    rota.pop_back();
    while(melhora){
        double total = 0;
        for(int i = 0; i < rota.size() - 1; i++){
            total += (distancias[rota[i]][rota[i+1]]);
        }
        melhora = false;
        melhorRota = OPT(rota, distancias, dist, q, altitudes);
        std::cout << melhorRota.second << "  " << dist << std::endl;
        if(melhorRota.second < dist){
            rota = melhorRota.first;
            dist = total;
            melhora = true;
        }
    }

    melhora = true;
    while(melhora){
        double total = 0;
        for(int i = 0; i < rota.size() - 1; i++){
            total += (distancias[rota[i]][rota[i+1]] + altitudes[rota[i]][rota[i+1]] + q[rota[(i > 0) ? i-1 : i]][rota[i]][rota[i+1]]);
        }
        melhora = false;
        melhorRota = OPT2(rota, distancias, dist, q, altitudes);
        std::cout << melhorRota.second << "  " << dist << std::endl;
        if(melhorRota.second < dist){
            rota = melhorRota.first;
            dist = total;
            melhora = true;
        }
    }
    return melhorRota;
}

std::vector<int> construirFronteira(std::vector<int> obstaculos_indices, int largura, int altura) {
    std::vector<int> fronteira;
    int x = 0;
    int y = 0;

    // Lado esquerdo
    for (int i = 0; i < altura; i++) {
        auto it = std::find(obstaculos_indices.begin(), obstaculos_indices.end(), x);
        if (it != obstaculos_indices.end()) {
            x -= altura;
            y = x + altura + altura;
            while (std::find(obstaculos_indices.begin(), obstaculos_indices.end(), x+altura) != obstaculos_indices.end()) {
                x++;
                fronteira.push_back(x);
            }
            fronteira.pop_back();
            x += altura;
            fronteira.push_back(x);
            x += altura - 1;
            while (x != y) {
                fronteira.push_back(x);
                x--;
            }
            continue;
        }
        fronteira.push_back(x);
        x += altura;
    }

    // Lado superior
    x = altura * (largura - 1);
    for (int i = 0; i < largura - 1; i++) {
        auto it = std::find(obstaculos_indices.begin(), obstaculos_indices.end(), x);
        if (it != obstaculos_indices.end()) {
            y = x + 2;
            while (std::find(obstaculos_indices.begin(), obstaculos_indices.end(), x) != obstaculos_indices.end()) {
                x++;
                fronteira.push_back(x);
            }
            fronteira.pop_back();
            x += 1;
            fronteira.push_back(x);
            x += 2;
            while (x != y) {
                fronteira.push_back(x);
                x--;
            }
            continue;
        }
        x++;
        fronteira.push_back(x);
    }

    // Lado direito
    x = altura * largura - 1;
    for (int i = 0; i < altura; i++) {
        auto it = std::find(obstaculos_indices.begin(), obstaculos_indices.end(), x);
        if (it != obstaculos_indices.end()) {
            x += altura;
            y = x - altura - altura;
            while (std::find(obstaculos_indices.begin(), obstaculos_indices.end(), x-altura) != obstaculos_indices.end()) {
                x--;
                fronteira.push_back(x);
            }
            fronteira.pop_back();
            x -= altura;
            fronteira.push_back(x);
            if(x > 10)
            x -= altura - 1;
            while (x != y) {
                fronteira.push_back(x);
                x++;
            }
            continue;
        }
        fronteira.push_back(x);
        x -= altura;
    }

    // Lado inferior
    x = altura - 1;
    for (int i = 0; i < largura - 4; i++) {
        auto it = std::find(obstaculos_indices.begin(), obstaculos_indices.end(), x);
        if (it != obstaculos_indices.end()) {
            y = x - 2;
            while (std::find(obstaculos_indices.begin(), obstaculos_indices.end(), x) != obstaculos_indices.end()) {
                x--;
                fronteira.push_back(x);
            }
            fronteira.pop_back();
            x -= 1;
            fronteira.push_back(x);
            x -= 2;
            while (x != y) {
                fronteira.push_back(x);
                x++;
            }
            continue;
        }
        if(std::find(fronteira.begin(), fronteira.end(), x) != fronteira.end()){
            i--;
            x-=1;
            continue;
        }
        fronteira.push_back(x);
        x -= 1;
    }

    fronteira.push_back(0);

    for (int vertice : fronteira) {
        std::cout << vertice + 1 << ", ";
    }
    std::cout << std::endl;

    return fronteira;
}

int encontrarProximoPontoNaoVisitado(const std::vector<int>& caminho, const std::vector<std::vector<double>>& distancias, std::vector<bool>& visitado, std::vector<std::vector<std::vector<double>>> q,  std::vector<std::vector<double>> altitudes) {
    int n = distancias.size();
    int proximoPonto = -1;
    double menorCustoInsercao = std::numeric_limits<double>::max();

    for (int i = 0; i < n; i++) {
        if (!visitado[i]) {
            for (size_t j = 0; j < caminho.size() - 1; j++) {
                int pontoA = caminho[j];
                int pontoB = caminho[j + 1];
                int pontoD;
                if(j+1 < caminho.size() - 1)
                pontoD = caminho[j + 2];
                else
                pontoD = pontoA;
                int pontoC;
                if(j!=0)
                pontoC = caminho[j - 1];
                else
                pontoC = pontoA;

                // Verifica se os pontos e índices estão dentro dos limites
                if (pontoA >= 0 && pontoA < n && pontoB >= 0 && pontoB < n && i >= 0 && i < n) {
                    // Verifica se as distâncias são finitas antes de calcular o custo de inserção
                    if (std::isfinite(distancias[pontoA][i]) && std::isfinite(distancias[i][pontoB])) {
                        double custoInsercao = std::numeric_limits<double>::infinity();

                        // Verifica se pontoA para pontoB é finito, evitando problemas com obstáculos
                        if (std::isfinite(distancias[pontoA][pontoB])) {
                            custoInsercao = (distancias[pontoA][i] + altitudes[pontoA][i] + q[pontoC][pontoA][i]) + (distancias[i][pontoB] + altitudes[i][pontoB] + q[pontoA][i][pontoB]) + q[i][pontoB][pontoD] - (distancias[pontoA][pontoB] + altitudes[pontoA][pontoB] + q[pontoC][pontoA][pontoB]);
                        }

                        if (custoInsercao < menorCustoInsercao) {
                            menorCustoInsercao = custoInsercao;
                            proximoPonto = i;
                        }
                    }
                }
            }
        }
    }

    return proximoPonto;
}


// Função para construir um caminho hamiltoniano usando a heurística de inserção mais barata
std::vector<int> construirCaminhoInsercaoMaisBarata(const std::vector<std::vector<double>>& distancias, const std::vector<int>& fronteira, std::vector<std::vector<std::vector<double>>> q,  std::vector<std::vector<double>> altitudes) {
    int n = distancias.size();
    std::vector<bool> visitado(n, false);
    std::vector<int> caminho; // Caminho inicial com a fronteira
    for(int i = 0; i < fronteira.size(); i++){
        caminho.push_back(fronteira[i]);
        visitado[fronteira[i]] = true;
    }

    while (caminho.size() < n-20) {
        std::cout << caminho.size() << std::endl;
        int proximoPonto = encontrarProximoPontoNaoVisitado(caminho, distancias, visitado, q, altitudes);
    
        if (proximoPonto != -1) {
            // Encontre a posição de inserção que minimiza o custo
            int melhorPosicaoInsercao = -1;
            double menorCustoInsercao = std::numeric_limits<double>::max();
            //std::cout << caminho.size()+1 << std::endl;
            for (size_t i = 0; i < caminho.size() - 1; i++) {
                int pontoA = caminho[i];
                int pontoB = caminho[i + 1];
                int pontoD;
                if(i+1 < caminho.size() - 1)
                pontoD = caminho[i + 2];
                else
                pontoD = pontoA;
                int pontoC;
                if(i!=0)
                pontoC = caminho[i - 1];
                else
                pontoC = pontoA;
                double custoInsercao = (distancias[pontoA][proximoPonto] + altitudes[pontoA][proximoPonto] + q[pontoC][pontoA][proximoPonto]) + (distancias[proximoPonto][pontoB] + altitudes[proximoPonto][pontoB] + q[pontoA][proximoPonto][pontoB]) + q[proximoPonto][pontoB][pontoD] - (distancias[pontoA][pontoB] + altitudes[pontoA][pontoB] + q[pontoC][pontoA][pontoB]);

                if (custoInsercao < menorCustoInsercao) {
                    menorCustoInsercao = custoInsercao;
                    melhorPosicaoInsercao = i + 1;
                }
            }
            //std::cout << caminho.size()+2 << std::endl;
            caminho.insert(caminho.begin() + melhorPosicaoInsercao, proximoPonto);
            //std::cout << caminho.size()+3 << std::endl;
            visitado[proximoPonto] = true;
        }
    }

    //caminho.push_back(0);

    return caminho;
}


int main() {
    std::string mapas = "mapas12.txt";

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
    //std::vector<int> obstaculos_indices = {19, 20, 21, 22, 50, 51, 52, 53, 54, 76, 77, 78, 205, 206, 207, 79, 102, 103, 104, 104, 124, 125, 126, 126, 160, 161, 162, 163, 167, 168, 169, 170, 231, 232, 233};
    //12x12
    std::vector<int> obstaculos_indices = {13, 14, 15, 22, 23, 40, 41, 42, 43, 67, 68, 69, 84, 85, 104, 105, 106, 107, 122, 123, 124, 125};
    //10x10
    //std::vector<int> obstaculos_indices = {13, 14, 15, 24, 25, 26, 57, 58, 59, 70, 71, 72, 94, 95, 96};
    //8x8
    //std::vector<int> obstaculos_indices = {20, 21, 22, 23, 40, 41, 42, 61, 60, 59};
    //15x15
    //std::vector<int> obstaculos_indices = {102, 11, 12, 13, 14, 19, 20, 21, 22, 36, 33, 34, 35, 50, 51, 52, 53, 54, 78, 79, 80, 81, 103, 104, 124, 125, 126, 160, 161, 162, 163, 192, 193, 191, 167, 168, 169, 170, 198, 199, 200, 201};
    //13x13
    //std::vector<int> obstaculos_indices = {33, 34, 153, 154, 67, 80, 82, 83, 84, 85, 132, 133, 134, 25, 97, 27, 28, 63, 64, 113, 114, 115, 116, 81, 54, 55, 56, 57, 23, 24};
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
    int fin = 1;

    // Cria a matriz de custos
    std::vector<std::vector<double>> c(n, std::vector<double>(n, std::numeric_limits<double>::infinity()));
    for (int i : validos) {
        for (int j : validos) {
            if (i != j) {
                int dist = std::sqrt(std::pow(std::get<0>(coord[i]) - std::get<0>(coord[j]), 2) + std::pow(std::get<1>(coord[i]) - std::get<1>(coord[j]), 2) + std::pow(std::get<2>(coord[i]) - std::get<2>(coord[j]), 2));
                if (dist > 28) { // Distancia máxima entre vértices adjacentes
                    //printf("Distancia entre %d e %d: %f\n", i, j, c[i][j]);
                    continue;  // Então se i e j não forem adjacentes, c[i][j] = inf
                } else {
                    c[i][j] = dist + 0.5;
                }    //printf("Distancia entre %d e %d: %f\n", i, j, c[i][j]);
            }
        }
    }

    // Matrizes que guardam a penalidade de altitude e ângulo
    std::vector<std::vector<double>> altitudes(n, std::vector<double>(n, 0.0));
    std::vector<std::vector<std::vector<double>>> q(n, std::vector<std::vector<double>>(n, std::vector<double>(n, 0.0)));
    std::vector<std::vector<double>> distancias(n, std::vector<double>(n, std::numeric_limits<double>::infinity()));
    std::ofstream pathFile("path12.txt");

    // Função para calcular o caminho mínimo usando o algoritmo de Dijkstra com heap
    auto dijkstra = [&](const std::vector<std::vector<double>>& c, int i, int j) {
        std::vector<double> dist(n, std::numeric_limits<double>::infinity());
        std::vector<double> dist2(n, std::numeric_limits<double>::infinity());
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
                    alt[v] = alt[u] + altitude;
                    dist[v] = dist[u] + (c[u][v] + angulos);
                    parent[v] = u;
                    heap.push(std::make_pair(dist[v], v));
                }
            }
        }

        distancias[i][j] = dist[j];
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


    for (int i : validos) {
        for (int j : validos) {
            for(int k : validos)
                q[i][j][k] = calcular_a(i, j, k, coord);
        }
    }
    
    int maiorx = (std::get<0>(coord[coord.size()-1]) + 10)/20;
    int maiory = (std::get<1>(coord[coord.size()-1]) + 10)/20;
    std::cout << maiorx << " " << maiory << std::endl;
    //15x15
    //std::vector<int> fronteira = {0, 15, 30, 45, 60, 76, 90, 105, 120, 135, 150, 165, 180, 195, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 209, 194, 179, 164, 149, 134, 119, 118, 117, 101, 87, 88, 89, 74, 59, 44, 29, 28, 27, 26, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1};
    //13x13
    //std::vector<int> fronteira = construirFronteira(obstaculos_indices, maiorx, maiory);
    //8x8
    //std::vector<int> fronteira = {0, 8, 16, 24, 32, 33, 34, 43, 50, 49, 48, 56, 57, 58, 51, 52, 53, 62, 63, 55, 47, 39, 31, 30, 29, 28, 19, 12, 13, 14, 15, 7, 6, 5, 4, 3, 2, 1};
    //std::vector<int> fronteira = {0, 10, 20, 30, 40, 50, 60, 61, 62, 73, 82, 81, 80, 90, 91, 92, 93, 84, 85, 87, 97, 98, 99, 89, 79, 69, 68, 67, 56, 47, 48, 49, 39, 29, 19, 9, 8, 7, 6, 5, 4, 3, 2, 1};
    //std::vector<int> cicloHamiltoniano = construirCaminhoInsercaoMaisBarata(distancias, fronteira, q, altitudes);

    double total = 0;
    //for(int i = 0; i < cicloHamiltoniano.size() - 1; i++){
    //    total += distancias[cicloHamiltoniano[i]][cicloHamiltoniano[i+1]];
    //}

    //std::pair<std::vector<int>, double> melhorRota = Local_Search(cicloHamiltoniano, distancias, total, q, altitudes);
    //cicloHamiltoniano = melhorRota.first;
    //cicloHamiltoniano.push_back(0);
    std::vector<int> cicloHamiltoniano = {1, 13, 25, 26, 27, 28, 40, 39, 38, 37, 49, 50, 51, 52, 53, 54, 55, 56, 67, 66, 65, 64, 63, 62, 61, 73, 74, 75, 76, 87, 88, 100, 101, 102, 114, 113, 112, 111, 99, 98, 97, 109, 110, 122, 121, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 132, 120, 119, 131, 130, 118, 117, 129, 128, 127, 115, 116, 104, 103, 91, 90, 89, 77, 78, 79, 80, 92, 93, 81, 82, 94, 95, 96, 84, 83, 71, 72, 60, 59, 58, 57, 45, 46, 47, 48, 36, 35, 34, 33, 32, 31, 30, 29, 17, 18, 19, 20, 21, 22, 11, 12, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1};

    for(int i = 0; i < cicloHamiltoniano.size(); i++){
        cicloHamiltoniano[i] = cicloHamiltoniano[i] - 1;
    }

    total = 0;
    for(int i = 0; i < cicloHamiltoniano.size() - 1; i++){
        total += (distancias[cicloHamiltoniano[i]][cicloHamiltoniano[i+1]] + altitudes[cicloHamiltoniano[i]][cicloHamiltoniano[i+1]] + q[cicloHamiltoniano[(i > 0) ? i-1 : i]][cicloHamiltoniano[i]][cicloHamiltoniano[i+1]]);
    }

    valorTotalAcumulado = total;

    std::cout << "Total Gurobi: " << valorTotalAcumulado << std::endl;

    auto end_time = std::chrono::steady_clock::now();
    std::cout << "Distancia entre 0 e 24: " << distancias[0][24] << std::endl;
    std::cout << "Distancia entre 37 e 64: " << distancias[37][64] << std::endl;
    std::cout << "Distancia entre 120 e 121: " << distancias[120][121] << std::endl; 
    // Calcular la duración del tiempo transcurrido
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);

    // Imprimir el tiempo transcurrido en microsegundos
    std::cout << "Tempo Gasto: " << duration.count() << " segundos" << std::endl;

    pathFile.close();

    std::cout << cicloHamiltoniano.size() << std::endl;
    
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
