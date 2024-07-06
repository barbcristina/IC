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

// Função para calcular h
double calcular_h(int i, int j, double dist, const std::vector<std::tuple<double, double, double>>& coord) {
    std::tuple<double, double, double> coord_1 = coord[i];
    std::tuple<double, double, double> coord_2 = coord[j];

    // Verifica se a cidade de partida é diferente da de chegada
    if (i == j) {
        return 0;
    } else {
        double h = 1.5 * ((std::get<2>(coord_2) - std::get<2>(coord_1)));
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

        double angulo = 0.2 * (180 / M_PI) * ang;

        return angulo;
    }
}

// Função para imprimir o caminho mínimo
std::vector<int> print_path(std::vector<int>& parent, int v, std::ofstream& output) {
    std::vector<int> path;
    if (parent[v] == -1) {
        path.push_back(v);
        output << v << " ";
        return path;
      }
    path = print_path(parent, parent[v], output);
    path.push_back(v);
    output << v << " ";
    return path;
}

std::pair<std::vector<int>, double> OPT2(std::vector<int>& rota, const std::vector<std::vector<double>>& distancias, double dist, std::vector<std::vector<std::vector<double>>>& q){
    double ganhoMax = 0;
    std::pair<std::vector<int>, double> melhor;
    melhor.first = rota;
    melhor.second = dist;
    std::vector<int> bestV(4);
    int n = rota.size()-1;
    int limc;

    for(int a = 0; a < n-2; a++){
        int b = a+1;
        limc = n-1;
        for(int c = a+2; c < limc; c++){
            int d;
            d=c+1;
            double ganho = distancias[rota[a]][rota[b]] + distancias[rota[c]][rota[d]] + q[rota[(a != 0) ? a-1 : 0]][rota[a]][rota[b]] + q[rota[a]][rota[b]][rota[b+1]] + q[rota[c]][rota[d]][rota[(c==limc) ? d : d+1]] + q[rota[c-1]][rota[c]][rota[d]] - distancias[rota[a]][rota[c]] - distancias[rota[b]][rota[d]] - q[rota[(a != 0) ? a-1 : 0]][rota[a]][rota[c]] - q[rota[d]][rota[b]][rota[b+1]] - q[rota[b]][rota[d]][rota[(c==d) ? d : d+1]] - q[rota[c-1]][rota[c]][rota[a]];
            if(ganho > ganhoMax){
                ganhoMax = ganho;
                bestV = {a, b, c, d};
                //std::cout << "a b c d: " << rota[a] << "  " << rota[b]<< "  " << rota[c] << "  " << rota[d] << "  " << ganho << std::endl;
            }
        }
    }
    if(ganhoMax > 0){
        melhor.second = dist - ganhoMax;
   
        std::reverse(melhor.first.begin() + bestV[1], melhor.first.begin() + bestV[2] + 1);
        rota = melhor.first;
        //melhor.first[bestV[1]] = rota[bestV[2]];
        //melhor.first[bestV[2]] = rota[bestV[1]];
    }
    
    return melhor;
}

std::pair<std::vector<int>, double> Local_Search(std::vector<int>& rota, const std::vector<std::vector<double>>& distancias, double dist, std::vector<std::vector<std::vector<double>>>& q){
    bool melhora = true;
    std::pair<std::vector<int>, double> melhorRota;
    double total = 0;
    int iter = 0;
    int maxIter = 500;

    for(int i = 0; i < rota.size() - 1; i++){
        total += (distancias[rota[i]][rota[i+1]] + q[rota[(i > 0) ? i-1 : i]][rota[i]][rota[i+1]]);
    }

    while(melhora && iter < maxIter){
        melhora = false;
        melhorRota = OPT2(rota, distancias, total, q);
        double twopt = 0;
        for(int i = 0; i < rota.size() - 1; i++){
            twopt += (distancias[melhorRota.first[i]][melhorRota.first[i+1]] + q[melhorRota.first[(i > 0) ? i-1 : i]][melhorRota.first[i]][melhorRota.first[i+1]]);
        }
        //twopt += (distancias[melhorRota.first[rota.size()-1]][0] + q[melhorRota.first[rota.size()-2]][melhorRota.first[rota.size()-1]][0]);
        //std::cout << "2opt vs dist anterior: " << twopt << "  " << melhorRota.second << std::endl;
        if(melhorRota.second < total){
            rota = melhorRota.first;
            total = melhorRota.second;
            melhora = true;
            //std::cout << "melhorou na iteracao: " << iter << std::endl;
        }
        iter++;
    }
    return melhorRota;
}


std::vector<int> encontrarProximoPonto(int anterior, int atual, const std::vector<std::vector<double>>& distancias, std::vector<std::vector<std::vector<double>>>& q, std::vector<bool>& visitado, int pontoInicial, std::vector<int> ciclo) {
    int n = distancias.size();
    int proximoPonto = -1;
    double menorDistancia = std::numeric_limits<double>::max();
    std::vector<bool> selecionados(n, false);
    std::vector<int> maisprox;

    std::random_device rd;
    std::mt19937 gen(rd());

    int min = 2;
    int max = 20;

    std::uniform_int_distribution<> dist(min, max);
    int numero_aleatorio = dist(gen);

    for(int j = 0; j < numero_aleatorio; j++){
        double menorDistancia = std::numeric_limits<double>::max();
        for (int i = 0; i < n; i++) {
            if (!selecionados[i] && i != atual && !visitado[i] && distancias[atual][i] + q[anterior][atual][i] < menorDistancia) {
                proximoPonto = i;
                menorDistancia = distancias[atual][i];
                selecionados[proximoPonto] = true;
                maisprox.push_back(proximoPonto);
            }
        }
    }

    return maisprox;
}


// Função para encontrar o ciclo hamiltoniano usando KNN
std::vector<int> KNN2INI(int tam, const std::vector<std::vector<double>>& distancias, std::vector<std::vector<std::vector<double>>>& q, int obsSize) {
    int n = distancias.size() - obsSize;
    std::vector<bool> visitado(n, false);
    std::vector<int> cicloHamiltoniano;

    int pontoAtual = 0;
    int pontoat = tam;
    int pontoant = 0;
    int pontoantant = tam;
    int pontoini = pontoat;
    int pontoInicial = pontoAtual;
    int valorTotalAcumulado = 0;

    cicloHamiltoniano.push_back(pontoAtual);
    visitado[pontoAtual] = true;
    // tentar incluir a penalizacao por angulo aqui
    for (int i = 1; i < n/2; i++) { 
        
        std::vector<int> proxPonto = encontrarProximoPonto(pontoantant, pontoAtual, distancias, q, visitado, pontoInicial, cicloHamiltoniano);

        std::mt19937 rng(static_cast<unsigned>(std::time(0)));

        std::uniform_int_distribution<size_t> dist(0, proxPonto.size() - 1);
    
        // Escolha um índice aleatório
        size_t randomIndex = dist(rng);

        int proximoPonto = proxPonto[randomIndex]; 

        if (proximoPonto == -1) {
            break;
        }

        valorTotalAcumulado += (distancias[pontoAtual][proximoPonto] + q[pontoantant][pontoAtual][proximoPonto]);
        cicloHamiltoniano.push_back(proximoPonto);
        visitado[proximoPonto] = true;
        pontoantant = pontoAtual;
        pontoAtual = proximoPonto;
    }


    cicloHamiltoniano.push_back(pontoat);
    visitado[pontoat] = true;
    for (int i = 1; i < n/2; i++) {
        
        std::vector<int> proxPonto = encontrarProximoPonto(pontoantant, pontoAtual, distancias, q, visitado, pontoInicial, cicloHamiltoniano);

        std::mt19937 rng(static_cast<unsigned>(std::time(0)));

        std::uniform_int_distribution<size_t> dist(0, proxPonto.size() - 1);
    
        // Escolha um índice aleatório
        size_t randomIndex = dist(rng);

        int proximoPonto = proxPonto[randomIndex];

        if (proximoPonto == -1) {
            break;
        }

        valorTotalAcumulado += (distancias[pontoat][proximoPonto] + q[pontoant][pontoat][proximoPonto]);
        cicloHamiltoniano.push_back(proximoPonto);
        visitado[proximoPonto] = true;
        pontoant = pontoat;
        pontoat = proximoPonto;
    }
    std::reverse(cicloHamiltoniano.begin() + n/2, cicloHamiltoniano.end());
    cicloHamiltoniano.push_back(pontoInicial);

    return cicloHamiltoniano;
}

// Função para encontrar o ciclo hamiltoniano usando KNN
std::vector<int> KNN(int tam, const std::vector<std::vector<double>>& distancias, std::vector<std::vector<std::vector<double>>>& q, int obsSize) {
    int n = distancias.size() - obsSize;
    std::vector<bool> visitado(n, false);
    std::vector<int> cicloHamiltoniano;

    int pontoAtual = 0;
    int pontoant = 0;
    int pontoInicial = pontoAtual;
    int valorTotalAcumulado = 0;

    cicloHamiltoniano.push_back(pontoAtual);
    visitado[pontoAtual] = true;

    while (cicloHamiltoniano.size() < n) {

        std::vector<int> proxPonto = encontrarProximoPonto(pontoant, pontoAtual, distancias, q, visitado, pontoInicial, cicloHamiltoniano);

        std::mt19937 rng(static_cast<unsigned>(std::time(0)));

        std::uniform_int_distribution<size_t> dist(0, proxPonto.size() - 1);
    
        // Escolha um índice aleatório
        size_t randomIndex = dist(rng);

        int proximoPonto = proxPonto[randomIndex];

        if (proximoPonto == -1) {
            break;
        }

        valorTotalAcumulado += (distancias[pontoAtual][proximoPonto] + q[pontoant][pontoAtual][proximoPonto]);
        cicloHamiltoniano.push_back(proximoPonto);
        visitado[proximoPonto] = true;
        pontoant = pontoAtual;
        pontoAtual = proximoPonto;
    }
    cicloHamiltoniano.push_back(pontoInicial);

    return cicloHamiltoniano;
}

std::vector<int> grasp(int tam, const std::vector<std::vector<double>>& distancias, std::vector<std::vector<std::vector<double>>>& q, int obsSize){
    float lim = 2000000;
    std::vector<int> S;
    std::vector<int> best;
    std::pair<std::vector<int>, double> par;
    std::vector<int> s;
    bool melhora = true;
    int qtdit = 0;
    int melhorou = 0;

    //std::cout << "comecando o grasp " << std::endl;
    while(melhora){
        S = KNN(tam, distancias, q, obsSize);
        //std::cout << "passou pela insercao mais barata " << std::endl;
        float valor = 0;
        for(int k = 0; k < S.size() - 1; k++)
            valor += (distancias[S[k]][S[k+1]] + q[S[(k > 0) ? k-1 : k]][S[k]][S[k+1]]);
        //std::cout << "construtiva: " << valor << std::endl;
        par = Local_Search(S, distancias, 0, q);
        s = par.first;
        valor = 0;
        for(int k = 0; k < s.size() - 1; k++){
            valor += (distancias[s[k]][s[k+1]] + q[s[(k > 0) ? k-1 : k]][s[k]][s[k+1]]);
        }
        qtdit++;
        if(valor < lim){
            best = s;
            lim = valor;
            melhorou = qtdit;
        }

        if(qtdit - melhorou >= 100){
            melhora = false;
        }
        //std::cout << "2opt: " << lim << std::endl;
    }
    std::cout << "Qtd de iterações: " << qtdit - 1 << std::endl;
    return best;
}

int main() {
    std::string mapas = "100_pontos/mapas10.txt";

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
    std::vector<int> obstaculos_indices;

    for (int i = 0; i < linhas.size() - 10; i++) {
        std::istringstream iss(linhas[i]);
        std::string cidade;
        double x, y, z;
        iss >> cidade >> x >> y >> z;
        coord.push_back(std::make_tuple(x, y, z));
    }

    // para iterar só fazer linhas.size() - i
    int nObs = 10;
    std::istringstream iss(linhas[linhas.size() - nObs]);
    int obstaculo;
    while (iss >> obstaculo) {
        obstaculos_indices.push_back(obstaculo);
        std::cout << obstaculo << std::endl;
    }

    //std::vector<int> cicloHamiltoniano = {0, 117, 101, 100, 115, 114, 98, 97, 113, 130, 131, 132, 133, 149, 148, 147, 146, 129, 112, 128, 144, 145, 162, 163, 164, 165, 166, 167, 151, 135, 136, 120, 105, 90, 75, 76, 77, 93, 110, 127, 111, 95, 79, 63, 62, 61, 60, 59, 58, 57, 41, 42, 43, 44, 45, 46, 47, 30, 29, 28, 27, 26, 25, 9, 8, 7, 6, 22, 38, 53, 69, 54, 39, 23, 24, 40, 56, 72, 73, 74, 91, 108, 109, 126, 142, 158, 159, 175, 191, 190, 189, 173, 157, 140, 124, 107, 106, 122, 139, 156, 172, 188, 204, 205, 221, 236, 235, 218, 202, 186, 170, 169, 168, 184, 183, 182, 181, 180, 179, 195, 210, 227, 212, 213, 214, 215, 232, 233, 216, 199, 198, 197, 196, 211, 226, 225, 224, 240, 241, 242, 243, 244, 228, 229, 230, 231, 248, 249, 250, 251, 252, 253, 255, 254, 237, 220, 219, 203, 187, 171, 154, 138, 121, 104, 103, 118, 102, 86, 85, 84, 83, 82, 66, 65, 1, 5, 4, 20, 3, 18, 2, 19, 64, 49, 48, 32, 16, 0};

    int n = coord.size();  // número de células (pontos)

    std::vector<std::tuple<double, double, double>> obstaculos;

    for (int i : obstaculos_indices) {
        obstaculos.push_back(coord[i]);
    }

    std::vector<int> validos;
    for (int i = 0; i < n; i++) {
        if (std::find(obstaculos_indices.begin(), obstaculos_indices.end(), i) == obstaculos_indices.end()) {
            validos.push_back(i);
        }
    }

    auto start_time = std::chrono::steady_clock::now();

    // Cria a matriz de custos
    std::vector<std::vector<double>> c(n, std::vector<double>(n, std::numeric_limits<double>::infinity()));
    for (int i : validos) {
        for (int j : validos) {
            if (i != j) {
                int dist = std::sqrt(std::pow(std::get<0>(coord[i]) - std::get<0>(coord[j]), 2) + std::pow(std::get<1>(coord[i]) - std::get<1>(coord[j]), 2) + std::pow(std::get<2>(coord[i]) - std::get<2>(coord[j]), 2));
                if (dist > 28) { // Distancia máxima entre vértices adjacentes
                    continue;  // Então se i e j não forem adjacentes, c[i][j] = inf
                } else {
                    c[i][j] = dist + 0.5;
                }
            }
        }
    }

    int maiorx = (std::get<0>(coord[coord.size()-1]) + 10)/20;
    int maiory = (std::get<1>(coord[coord.size()-1]) + 10)/20;
    std::cout << maiorx << " " << maiory << std::endl;

    for (int i = 0; i < validos.size(); i++) {
        for (int j = 0; j < validos.size(); j++) {
            if (std::find(obstaculos_indices.begin(), obstaculos_indices.end(), i-1) != obstaculos_indices.end() && std::find(obstaculos_indices.begin(), obstaculos_indices.end(), j+1) != obstaculos_indices.end() && (j == (i+maiorx+1) || i == (j-maiorx-1) || j == (i+maiorx-1) || i == (j-maiorx+1)) && (i-1)/maiorx == i/maiorx && (j+1)/maiorx == j/maiorx) {
                c[i][j] = std::numeric_limits<double>::infinity();
                c[j][i] = std::numeric_limits<double>::infinity();
            } else if (std::find(obstaculos_indices.begin(), obstaculos_indices.end(), i+1) != obstaculos_indices.end() && std::find(obstaculos_indices.begin(), obstaculos_indices.end(), j-1) != obstaculos_indices.end() && (j == (i+maiorx+1) || i == (j-maiorx-1) || j == (i+maiorx-1) || i == (j-maiorx+1)) && (i+1)/maiorx == i/maiorx && (j-1)/maiorx == j/maiorx) {
                c[i][j] = std::numeric_limits<double>::infinity();
                c[j][i] = std::numeric_limits<double>::infinity();
            }
        }
    }

    // Matrizes que guardam a penalidade de altitude e ângulo
    std::vector<std::vector<double>> altitudes(n, std::vector<double>(n, 0.0));
    std::vector<std::vector<double>> all_angles(n, std::vector<double>(n, 0.0));
    std::vector<std::vector<std::vector<int>>> matrix(n, std::vector<std::vector<int>>(n, std::vector<int>(n, 0)));
    std::vector<std::vector<double>> distancias(n, std::vector<double>(n, std::numeric_limits<double>::infinity()));

    std::ofstream pathFile("path.txt");

    // Função para calcular o caminho mínimo usando o algoritmo de Dijkstra com heap
    auto dijkstra = [&](const std::vector<std::vector<double>>& c, int i, int j) {
        std::vector<double> dist(n, std::numeric_limits<double>::infinity());
        std::vector<int> parent(n, -1);
        dist[i] = 0.0;
        std::vector<double> alt(n, 0.0);
        std::vector<double> angles(n, 0.0);
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
                    angles[v] = angles[u] + angulos;
                    dist[v] = dist[u] + (c[u][v] + altitude);
                    parent[v] = u;
                    heap.push(std::make_pair(dist[v], v));
                }
            }
        }

        if (c[i][j] != std::numeric_limits<float>::infinity()) {
          distancias[i][j] = c[i][j];
        } else {
          distancias[i][j] = dist[j];
        }
        all_angles[i][j] = angles[j];

        pathFile << "Caminho de " << i << " para " << j << ": ";
        matrix[i][j] = print_path(parent, j, pathFile);
        pathFile << "\n";
        std::cout << std::endl;
    };

    for (int i : validos) {
        for (int j : validos) {
            dijkstra(c, i, j);
        }
    }

    std::vector<std::vector<std::vector<double>>> q(n, std::vector<std::vector<double>>(n, std::vector<double>(n, 0.0)));

    // Se i, j e k forem adjacentes calcula a penalidade de angulo normalmente
    for (int i : validos) {
        for (int j : validos) {
            for(int k : validos){
                if (c[i][j] != std::numeric_limits<float>::infinity() && c[j][k] != std::numeric_limits<float>::infinity()) {
                    q[i][j][k] = calcular_a(i, j, k, coord);
                }
            }
        }
    }

    // Se i, j e k não forem adjacentes calcula a penalidade de angulo com base no caminho mínimo
    for (int i : validos) {
        for (int j : validos) {
            for(int k : validos){
                if ((c[i][j] == std::numeric_limits<float>::infinity() || c[j][k] == std::numeric_limits<float>::infinity()) && i != j && j != k) {
                    q[i][j][k] = (all_angles[i][j] + all_angles[j][k]) + (q[matrix[i][j][matrix[i][j].size()-2]][j][matrix[j][k][1]]);
                }
            }
        }
    }
    
    //std::vector<int> cicloHamiltoniano = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 21, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 34, 35, 47, 46, 45, 56, 67, 68, 69, 70, 71, 72, 73, 74, 75, 64, 63, 62, 61, 60, 48, 49, 50, 51, 52, 53, 54, 65, 76, 87, 98, 97, 96, 95, 94, 83, 82, 81, 80, 79, 78, 89, 90, 91, 92, 93, 104, 105, 106, 107, 108, 109, 120, 119, 118, 117, 116, 115, 114, 113, 112, 111, 110, 99, 88, 77, 66, 55, 44, 33, 22, 11, 0};

    //return 0;

    // se o último elemento da fronteira for 0, remova, pois pode atrapalhar na inserção mais barata
    // Recalcula a rota do gurobi
    //std::vector<int> cicloHamiltoniano = {0, 1, 2, 3, 4, 5, 6, 7, 19, 18, 17, 16, 15, 14, 13, 12, 23, 34, 45, 56, 57, 58, 59, 48, 37, 36, 26, 27, 40, 41, 42, 53, 52, 62, 61, 60, 49, 38, 28, 29, 30, 31, 43, 54, 65, 64, 63, 73, 72, 71, 70, 69, 68, 67, 77, 89, 90, 91, 92, 93, 94, 95, 96, 97, 117, 116, 115, 114, 113, 102, 82, 83, 84, 85, 86, 87, 98, 109, 108, 107, 106, 105, 104, 103, 101, 100, 99, 88, 66, 55, 44, 33, 22, 11, 0};

    std::vector<int> melhorRota = grasp(validos.back(), distancias, q, obstaculos_indices.size());
    //std::vector<int> melhorRota = construirCaminhoInsercaoMaisBarata(distancias, fronteira2, q, altitudes, obstaculos_indices.size());
    std::vector<int> cicloHamiltoniano = melhorRota;
    //cicloHamiltoniano = Local_Search(cicloHamiltoniano, distancias, 0, q, altitudes).first;
    //std::cout << "chegou no final" << std::endl;
    // Calcular o valor total do ciclo
    double total = 0;
    for(int i = 0; i < cicloHamiltoniano.size() - 1; i++){
        total += (distancias[cicloHamiltoniano[i]][cicloHamiltoniano[i+1]] + q[cicloHamiltoniano[(i > 0) ? i-1 : i]][cicloHamiltoniano[i]][cicloHamiltoniano[i+1]]);
    }

    std::cout << "Total: " << total << std::endl;

    double totalang = 0;
    int qtd = 0;
    int quarentacinco = 0, noventa = 0, centotrintacino = 0, centooitenta = 0;
    for(int i = 0; i < cicloHamiltoniano.size() - 1; i++){
        if(int(q[cicloHamiltoniano[(i > 0) ? i-1 : i]][cicloHamiltoniano[i]][cicloHamiltoniano[i+1]]) == 9)
            quarentacinco++;
        else if(int(q[cicloHamiltoniano[(i > 0) ? i-1 : i]][cicloHamiltoniano[i]][cicloHamiltoniano[i+1]]) == 18)
            noventa++;
        else if(int(q[cicloHamiltoniano[(i > 0) ? i-1 : i]][cicloHamiltoniano[i]][cicloHamiltoniano[i+1]]) == 27)
            centotrintacino++;
        else if(int(q[cicloHamiltoniano[(i > 0) ? i-1 : i]][cicloHamiltoniano[i]][cicloHamiltoniano[i+1]]) == 36)
            centooitenta++;
        totalang += q[cicloHamiltoniano[(i > 0) ? i-1 : i]][cicloHamiltoniano[i]][cicloHamiltoniano[i+1]];
        //std::cout << "Penalizacao e angulo: " << cicloHamiltoniano[(i > 0) ? i-1 : i] + 1 << " -> " << cicloHamiltoniano[i] + 1 << " -> " << cicloHamiltoniano[i+1] + 1 << " = " << q[cicloHamiltoniano[(i > 0) ? i-1 : i]][cicloHamiltoniano[i]][cicloHamiltoniano[i+1]] << ", " << (q[cicloHamiltoniano[(i > 0) ? i-1 : i]][cicloHamiltoniano[i]][cicloHamiltoniano[i+1]])/0.2 << std::endl;
    }

    std::cout << "Qtd de penalizações de angulo 45: " << quarentacinco << std::endl;
    std::cout << "Qtd de penalizações de angulo 90: " << noventa << std::endl;
    std::cout << "Qtd de penalizações de angulo 135: " << centotrintacino << std::endl;
    std::cout << "Qtd de penalizações de angulo 180: " << centooitenta << std::endl;
    std::cout << "Total: " << totalang << std::endl;

    auto end_time = std::chrono::steady_clock::now();
    // Calcular tempo transcorrido
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    std::cout << "Tempo Gasto: " << duration.count() << " millisegundos" << std::endl;

    pathFile.close();

    std::cout << cicloHamiltoniano.size() << std::endl;
    
    int l = 1;
    // Imprima o caminho hamiltoniano
    std::cout << "Ciclo Hamiltoniano: " << std::endl;
    for (int vertice : cicloHamiltoniano) {
        std::cout << vertice+1 << ", ";
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

        resultadosFile << "Valor Total do Ciclo: " << total << std::endl;

        resultadosFile.close();
    } else {
        std::cerr << "Erro ao abrir o arquivo resultados.txt para escrita." << std::endl;
        return 1;
    }

    return 0;
}
