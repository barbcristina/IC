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

        double angulo = 0.2 * (180 / M_PI) * ang;

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

std::pair<std::vector<int>, double> OPT2(std::vector<int> rota, const std::vector<std::vector<double>>& distancias, double dist, std::vector<std::vector<std::vector<double>>> q,  std::vector<std::vector<double>> altitudes){
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
            //+ q[rota[b]][rota[d]][(d == n) ? rota[d] : rota[d+1]]
            //if(distancias[rota[a]][rota[b]] < 30 && distancias[rota[c]][rota[d]] < 30 && distancias[rota[a]][rota[c]] < 30 && distancias[rota[b]][rota[d]] < 30);
            double ganho = (distancias[rota[a]][rota[b]] + distancias[rota[c]][rota[d]] + q[rota[(a != 0) ? a-1 : 0]][rota[a]][rota[b]] + q[rota[a]][rota[b]][rota[b+1]] + q[rota[c]][rota[d]][rota[(c!=n) ? d+1 : d]] + q[rota[c-1]][rota[c]][rota[d]]) - (distancias[rota[a]][rota[c]] + distancias[rota[b]][rota[d]] + q[rota[(a != 0) ? a-1 : 0]][rota[a]][rota[c]] + q[rota[d]][rota[b]][rota[b+1]] + q[rota[b]][rota[d]][rota[(c!=n) ? d+1 : d]] + q[rota[c-1]][rota[c]][rota[a]]);
            if(ganho > ganhoMax){
                ganhoMax = ganho;
                bestV = {a, b, c, d};
                //std::cout << "a b c d: " << rota[a] << "  " << rota[b]<< "  " << rota[c] << "  " << rota[d] << "  " << ganho << std::endl;
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

std::pair<std::vector<int>, double> Local_Search(std::vector<int> rota, const std::vector<std::vector<double>>& distancias, double dist, std::vector<std::vector<std::vector<double>>> q,  std::vector<std::vector<double>> altitudes){
    bool melhora = true;
    std::pair<std::vector<int>, double> melhorRota;
    rota.pop_back();
    double total = 0;
    int iter = 0;
    int maxIter = 1000;

    for(int i = 0; i < rota.size() - 1; i++){
        total += (distancias[rota[i]][rota[i+1]] + q[rota[(i > 0) ? i-1 : i]][rota[i]][rota[i+1]]);
    }

    while(melhora && iter < maxIter){
        melhora = false;
        melhorRota = OPT2(rota, distancias, total, q, altitudes);
        double twopt = 0;
        for(int i = 0; i < rota.size() - 1; i++){
            twopt += (distancias[melhorRota.first[i]][melhorRota.first[i+1]] + q[melhorRota.first[(i > 0) ? i-1 : i]][melhorRota.first[i]][melhorRota.first[i+1]]);
        }
        //std::cout << "2opt vs dist anterior: " << twopt << "  " << total << std::endl;
        if(melhorRota.second < total){
            rota = melhorRota.first;
            total = melhorRota.second;
            melhora = true;
        }
        iter++;
    }
    return melhorRota;
}

std::vector<int> construirFronteira(std::vector<int> obstaculos, int largura, int altura) {
    std::vector<int> fronteira;
    int x = altura;
    int y = largura;

    // Esquerda
    for (int i = 0; i < x * (y - 1) + 1; i += x) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i) == obstaculos.end()) {
            fronteira.push_back(i);
        }
    }

    // Superior
    for (int i = x * (y - 1) + 1; i < x * x; i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i) == obstaculos.end()) {
            fronteira.push_back(i);
        }
    }

    // Direita
    for (int i = (x * x - x) - 1; i >= x - 1; i -= x) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i) == obstaculos.end()) {
            fronteira.push_back(i);
        }
    }

    // Inferior
    for (int i = x - 2; i > 0; i--) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i) == obstaculos.end()) {
            fronteira.push_back(i);
        }
    }

    // Superior
    for (int i = 0; i < (x*y); i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i + x) != obstaculos.end()) {
            int it = i;
            while (true) {
                it += x;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end() && it < (x*y)) {
                    std::cout << it << std::endl;
                    break;
                } else if (it > (x*y)-1) {
                    fronteira.push_back(i);
                    std::cout << it << std::endl;
                    break;
                }
            }
        }
    }

    // Esquerda subindo
    for (int i = 0; i < (x*y); i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i + x) != obstaculos.end() && std::find(fronteira.begin(), fronteira.end(), i) == fronteira.end()) {
            int it = i + x;
            while (true) {
                it--;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end()) {
                    std::cout << it << std::endl;
                    break;
                } else if (it % 8 == 0) {
                    fronteira.push_back(i);
                    std::cout << it << std::endl;
                    break;
                }
            }
        }
    }

    // Direita subindo
    for (int i = 0; i < (x*y); i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i + x) != obstaculos.end() && std::find(fronteira.begin(), fronteira.end(), i) == fronteira.end()) {
            int it = i + x;
            while (true) {
                it++;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end()) {
                    std::cout << it << std::endl;
                    break;
                } else if ((it % 8 == 7)) {
                    fronteira.push_back(i);
                    std::cout << it << std::endl;
                    break;
                }
            }
        }
    }

    // Inferior
    for (int i = 0; i < (x*y); i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i - x) != obstaculos.end()) {
            int it = i;
            while (true) {
                it -= x;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end() && it > 0) {
                    std::cout << it << std::endl;
                    break;
                } else if ((it < 0) && std::find(fronteira.begin(), fronteira.end(), i) == fronteira.end()) {
                    fronteira.push_back(i);
                    std::cout << it << std::endl;
                    break;
                }
            }
        }
    }

    // Direita descendo
    for (int i = 0; i < (x*y); i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i - x) != obstaculos.end()) {
            int it = i - x;
            while (true) {
                it++;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end()) {
                    break;
                } else if ((it % 8 == 7) && std::find(fronteira.begin(), fronteira.end(), i) == fronteira.end()) {
                    fronteira.push_back(i);
                    break;
                }
            }
        }
    }

    // Esquerda descendo
    for (int i = 0; i < (x*y); i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i - x) != obstaculos.end()) {
            int it = i - x;
            while (true) {
                it--;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end()) {
                    break;
                } else if ((it % 8 == 0) && std::find(fronteira.begin(), fronteira.end(), i) == fronteira.end()) {
                    fronteira.push_back(i);
                    break;
                }
            }
        }
    }

    // Direita
    for (int i = 0; i < (x*y); i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i + 1) != obstaculos.end() && std::find(obstaculos.begin(), obstaculos.end(), i) == obstaculos.end()) {
            int it = i;
            while (true) {
                it++;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end()) {
                    break;
                } else if (it % 8 == 7 && std::find(fronteira.begin(), fronteira.end(), i) == fronteira.end()) {
                    fronteira.push_back(i);
                    break;
                }
            }
        }
    }

    // Esquerda
    for (int i = 0; i < (x*y); i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i - 1) != obstaculos.end() && std::find(obstaculos.begin(), obstaculos.end(), i) == obstaculos.end()) {
            int it = i;
            while (true) {
                it--;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end()) {
                    break;
                } else if ((it % 8) == 0 && std::find(fronteira.begin(), fronteira.end(), i) == fronteira.end()) {
                    fronteira.push_back(i);
                    break;
                }
            }
        }
    }

    // Print fronteira
    std::cout << "Fronteira: ";
    for (int i : fronteira) {
        std::cout << i + 1 << " ";
    }
    std::cout << std::endl;

    return fronteira;
}


std::vector<int> encontrarProximoPontoNaoVisitado(const std::vector<int>& caminho, const std::vector<std::vector<double>>& distancias, std::vector<bool>& visitado, std::vector<std::vector<std::vector<double>>> q,  std::vector<std::vector<double>> altitudes) {
    int n = distancias.size();
    std::vector<bool> selecionados(n, false);
    std::vector<int> maisprox;
    int proximoPonto = -1;
    double menorCustoInsercao;
    
    std::random_device rd;
    std::mt19937 gen(rd());

    int min = 2;
    int max = 20;

    std::uniform_int_distribution<> dist(min, max);
    int numero_aleatorio = dist(gen);

    std::cout << "Número aleatório entre " << min << " e " << max << ": " << numero_aleatorio << std::endl;

    for(int j = 0; j < numero_aleatorio; j++){
        menorCustoInsercao = std::numeric_limits<double>::max();
        for (int i = 0; i < n; i++) {
            if (!visitado[i] and !selecionados[i]) {
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
                                custoInsercao = (distancias[pontoA][i] + q[pontoC][pontoA][i]) + (distancias[i][pontoB] + q[pontoA][i][pontoB]) + q[i][pontoB][pontoD] - (distancias[pontoA][pontoB] + altitudes[pontoA][pontoB] + q[pontoC][pontoA][pontoB]);
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
        selecionados[proximoPonto] = true;
        maisprox.push_back(proximoPonto);
    }

    return maisprox;
}


// Função para construir um caminho hamiltoniano usando a heurística de inserção mais barata
std::vector<int> construirCaminhoInsercaoMaisBarata(const std::vector<std::vector<double>>& distancias, const std::vector<int>& fronteira, std::vector<std::vector<std::vector<double>>> q,  std::vector<std::vector<double>> altitudes, int obsSize) {
    int n = distancias.size();
    std::vector<bool> visitado(n, false);
    std::vector<int> caminho; // Caminho inicial com a fronteira
    for(int i = 0; i < fronteira.size(); i++){
        caminho.push_back(fronteira[i]);
        visitado[fronteira[i]] = true;
    }
    std::cout << "criou a fronteira " << std::endl;
    while (caminho.size() < n-obsSize) {
        std::cout << caminho.size() << std::endl;
        std::vector<int> proxPonto = encontrarProximoPontoNaoVisitado(caminho, distancias, visitado, q, altitudes);
        std::cout << "selecionados: " << std::endl;
        //for(int v: proxPonto){
        //    std::cout << v << ", ";
        //}
        //std::cout << std::endl;
        std::mt19937 rng(static_cast<unsigned>(std::time(0)));

        std::uniform_int_distribution<size_t> dist(0, proxPonto.size() - 1);
    
        // Escolha um índice aleatório
        size_t randomIndex = dist(rng);

        int proximoPonto = proxPonto[randomIndex];

        if (proximoPonto != -1 && !visitado[proximoPonto]) {
            // Encontre a posição de inserção que minimiza o custo
            int melhorPosicaoInsercao = -1;
            double menorCustoInsercao = std::numeric_limits<double>::max();
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
                double custoInsercao = (distancias[pontoA][proximoPonto] + q[pontoC][pontoA][proximoPonto]) + (distancias[proximoPonto][pontoB] + q[pontoA][proximoPonto][pontoB]) + q[proximoPonto][pontoB][pontoD] - (distancias[pontoA][pontoB] + q[pontoC][pontoA][pontoB]);

                if (custoInsercao < menorCustoInsercao) {
                    menorCustoInsercao = custoInsercao;
                    melhorPosicaoInsercao = i + 1;
                }
            }
            
            caminho.insert(caminho.begin() + melhorPosicaoInsercao, proximoPonto);
            visitado[proximoPonto] = true;
        }
    }

    caminho.push_back(0);

    return caminho;
}

std::vector<int> grasp(int t, const std::vector<std::vector<double>>& distancias, const std::vector<int>& fronteira, std::vector<std::vector<std::vector<double>>> q,  std::vector<std::vector<double>> altitudes, int obsSize){
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
        S = construirCaminhoInsercaoMaisBarata(distancias, fronteira, q, altitudes, obsSize);
        //std::cout << "passou pela insercao mais barata " << std::endl;
        float valor = 0;
        for(int k = 0; k < S.size() - 1; k++)
            valor += (distancias[S[k]][S[k+1]] + q[S[(k > 0) ? k-1 : k]][S[k]][S[k+1]]);
        std::cout << "construtiva: " << valor << std::endl;
        par = Local_Search(S, distancias, 0, q, altitudes);
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

        if(qtdit - melhorou >= 50){
            melhora = false;
        }
        std::cout << "2opt: " << lim << std::endl;
    }
    std::cout << "Qtd de iterações: " << qtdit - 1 << std::endl;
    return best;
}

int main() {
    std::string mapas = "mapas8.txt";

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

    for (int i = 0; i < linhas.size() - 2; i++) {
        std::istringstream iss(linhas[i]);
        std::string cidade;
        double x, y, z;
        iss >> cidade >> x >> y >> z;
        coord.push_back(std::make_tuple(x, y, z));
    }

    // para iterar só fazer linhas.back() - i
    std::istringstream iss(linhas.back());
    int obstaculo;
    while (iss >> obstaculo) {
        obstaculos_indices.push_back(obstaculo);
        std::cout << obstaculo << std::endl;
    }

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
    std::ofstream pathFile("path8.txt");

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
                    //double angulos = calcular_a((parent[u] == -1) ? u : parent[u], u, v, coord);
                    //alt[v] = alt[u] + altitude;
                    dist[v] = dist[u] + (c[u][v] + altitude);
                    parent[v] = u;
                    heap.push(std::make_pair(dist[v], v));
                }
            }
        }

        distancias[i][j] = dist[j];
        //altitudes[i][j] = alt[j];
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
    
    // calcula altura e largura do grafo
    int maiorx = (std::get<0>(coord[coord.size()-1]) + 10)/20;
    int maiory = (std::get<1>(coord[coord.size()-1]) + 10)/20;
    std::cout << maiorx << " " << maiory << std::endl;

   
    std::vector<int> fronteira = construirFronteira(obstaculos_indices, maiorx, maiory);
    std::vector<int> fronteira2;
    fronteira2.push_back(0);
    int sz = fronteira.size();
    fronteira.erase(find(fronteira.begin(), fronteira.end(), 0));
    fronteira2.push_back(8);
    fronteira.erase(find(fronteira.begin(), fronteira.end(), 8));

    while(fronteira2.size() < sz-1){
        int menor = 100000;
        int k = 0;
        for(int j = 0; j < fronteira.size(); j++){
            std::cout << "distancia: " << fronteira2.back() << " " << fronteira[j] << ": " << c[fronteira2.back()][fronteira[j]] << std::endl;
            if(distancias[fronteira2.back()][fronteira[j]] < menor){
                menor = distancias[fronteira2.back()][fronteira[j]];
                k = fronteira[j];
            }
        }
        auto it = std::find(fronteira.begin(), fronteira.end(), k);

        // Verifique se o elemento foi encontrado
        if (it != fronteira.end()) {
            // Remova o elemento
            fronteira.erase(it);
        }
        std::cout << "Fronteira: ";
        for (int i : fronteira) {
            std::cout << i + 1 << " ";
        }
        
        std::cout << "menor: " << fronteira2.back() + 1 << " e " << k + 1 << ": " << distancias[fronteira2.back()][k] << std::endl;
        if(fronteira2.back() != k)
            fronteira2.push_back(k);
    }

    //8x8
    //std::vector<int> fronteira = {0, 8, 16, 24, 32, 40, 48, 56, 57, 58, 51, 52, 53, 62, 63, 55, 47, 39, 31, 30, 29, 28, 19, 12, 13, 14, 15, 7, 6, 5, 4, 3, 2, 1};
    //10x10
    //std::vector<int> fronteira = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 91, 92, 93, 94, 95, 96, 89, 79, 69, 59, 58, 57, 46, 37, 38, 39, 29, 19, 9, 8, 7, 6, 5, 4, 3, 2, 1};
    //15x15
    std::cout << "obstaculos: " << std::endl;
    for(int i = 0; i < obstaculos_indices.size(); i++){
        std::cout << obstaculos_indices[i] << std::endl;
    }
    std::cout << "Fronteira: ";
    for (int i : fronteira2) {
        std::cout << i + 1 << " ";
    }

    return 0;
    double total = 0;
    //64
    //std::vector<int> cicloHamiltoniano = {1, 2, 3, 4, 5, 6, 7, 8, 16, 15, 14, 13, 12, 11, 10, 18, 19, 20, 36, 37, 38, 39, 47, 46, 45, 44, 43, 42, 41, 49, 57, 58, 59, 60, 61, 62, 63, 64, 56, 48, 40, 32, 31, 30, 29, 28, 27, 26, 25, 17, 9, 1};
    //121
    //std::vector<int> cicloHamiltoniano = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 22, 33, 32, 31, 30, 29, 28, 27, 26, 25, 13, 24, 35, 36, 37, 38, 39, 40, 41, 21, 20, 19, 18, 17, 61, 62, 63, 64, 65, 66, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 57, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 88, 99, 98, 97, 96, 95, 84, 83, 82, 81, 80, 79, 90, 91, 92, 93, 94, 105, 106, 107, 108, 109, 110, 121, 120, 119, 118, 117, 116, 115, 114, 113, 112, 111, 100, 89, 78, 67, 56, 45, 34, 23, 12, 1};
    //144
    //std::vector<int> cicloHamiltoniano = {1, 10, 11, 12, 14, 15, 16, 42, 43, 44, 58, 59, 60, 75, 76, 77, 102, 103, 104, 105, 124, 125, 126, 2, 3, 4, 5, 6, 7, 8, 9, 24, 36, 48, 47, 46, 45, 57, 56, 55, 54, 41, 40, 39, 17, 18, 19, 20, 21, 22, 23, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 38, 50, 51, 52, 53, 66, 67, 68, 69, 70, 71, 72, 84, 96, 108, 120, 132, 144, 143, 142, 141, 140, 139, 138, 137, 136, 135, 123, 111, 112, 113, 114, 115, 116, 117, 118, 106, 94, 93, 92, 91, 90, 89, 88, 87, 86, 74, 62, 63, 64, 65, 78, 79, 80, 81, 82, 83, 95, 107, 119, 131, 130, 129, 128, 127, 101, 100, 99, 98, 110, 122, 134, 133, 121, 109, 97, 85, 73, 61, 49, 37, 25, 13, 1};
    //169 caminho
    //std::vector<int> cicloHamiltoniano = {1, 9, 17, 25, 26, 27, 28, 29, 30, 31, 32, 40, 48, 56, 64, 64, 63, 62, 61, 60, 59, 58, 57, 49, 41, 42, 43, 44, 45, 46, 47, 39, 38, 37, 36, 20, 19, 18, 10, 11, 12, 13, 14, 15, 16, 8, 7, 6, 5, 4, 3, 2, 1};
    //for(int i = 0; i < cicloHamiltoniano.size(); i++){
    //    cicloHamiltoniano[i] = cicloHamiltoniano[i] - 1;
    //}

    std::vector<int> melhorRota = grasp(4, distancias, fronteira2, q, altitudes, obstaculos_indices.size());
    std::vector<int> cicloHamiltoniano = melhorRota;
    cicloHamiltoniano.push_back(0);

    total = 0;
    for(int i = 0; i < cicloHamiltoniano.size() - 1; i++){
        total += (distancias[cicloHamiltoniano[i]][cicloHamiltoniano[i+1]] + q[cicloHamiltoniano[(i > 0) ? i-1 : i]][cicloHamiltoniano[i]][cicloHamiltoniano[i+1]]);
    }

    valorTotalAcumulado = total;

    std::cout << "Total Gurobi: " << valorTotalAcumulado << std::endl;

    auto end_time = std::chrono::steady_clock::now();
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

        resultadosFile << "Valor Total do Ciclo: " << valorTotalAcumulado << std::endl;

        resultadosFile.close();
    } else {
        std::cerr << "Erro ao abrir o arquivo resultados.txt para escrita." << std::endl;
        return 1;
    }

    return 0;
}