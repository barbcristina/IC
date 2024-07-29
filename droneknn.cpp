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
#include <string>

std::string formatWithComma(double number) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << number;
    std::string result = ss.str();

    // Substitui o ponto por vírgula
    size_t pos = result.find('.');
    if (pos != std::string::npos) {
        result[pos] = ',';
    }

    return result;
}

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

std::pair<std::vector<int>, double> OPT2(std::vector<int>& rota, const std::vector<std::vector<double>>& distancias, double dist, std::vector<std::vector<std::vector<double>>>& q, std::vector<std::vector<double>>& altitudes){
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
            double ganho = distancias[rota[a]][rota[b]] + distancias[rota[c]][rota[d]] + altitudes[rota[a]][rota[b]] + altitudes[rota[c]][rota[d]] + q[rota[(a != 0) ? a-1 : 0]][rota[a]][rota[b]] + q[rota[a]][rota[b]][rota[b+1]] + q[rota[c]][rota[d]][rota[(c==limc) ? d : d+1]] + q[rota[c-1]][rota[c]][rota[d]] - distancias[rota[a]][rota[c]] - distancias[rota[b]][rota[d]] - altitudes[rota[a]][rota[c]] - altitudes[rota[b]][rota[d]] - q[rota[(a != 0) ? a-1 : 0]][rota[a]][rota[c]] - q[rota[d]][rota[b]][rota[b+1]] - q[rota[b]][rota[d]][rota[(c==d) ? d : d+1]] - q[rota[c-1]][rota[c]][rota[a]];
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

std::pair<std::vector<int>, double> Local_Search(std::vector<int>& rota, const std::vector<std::vector<double>>& distancias, double dist, std::vector<std::vector<std::vector<double>>>& q, std::vector<std::vector<double>>& altitudes){
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
        melhorRota = OPT2(rota, distancias, total, q, altitudes);
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


std::vector<int> encontrarProximoPonto(int anterior, int atual, const std::vector<std::vector<double>>& distancias, std::vector<std::vector<std::vector<double>>>& q, std::vector<bool>& visitado, int pontoInicial, std::vector<int> ciclo, int obsSize, std::vector<int> obstaculos, std::vector<std::vector<double>>& altitudes) {
    int n = distancias.size();
    //std::cout << "tamanho validos " << obstaculos.size() << std::endl;  
    int proximoPonto = -1;
    double menorDistancia = std::numeric_limits<double>::max();
    std::vector<bool> selecionados(n, false);
    std::vector<int> maisprox;

    //std::cout << "entrou no encontrar prox ponto" << std::endl;

    std::random_device rd;
    std::mt19937 gen(rd());

    int min = 2;
    int max = 15;

    std::uniform_int_distribution<> dist(min, max);
    int numero_aleatorio = dist(gen);


    for(int j = 0; j < numero_aleatorio; j++){
        menorDistancia = std::numeric_limits<double>::max();
        //std::cout << "entrando no for" << std::endl;
        for (int i : obstaculos) {
            if (!selecionados[i] && i != atual && !visitado[i] && distancias[atual][i] + q[anterior][atual][i] < menorDistancia) {
                //std::cout << "passou do if" << std::endl;
                proximoPonto = i;
                menorDistancia = distancias[atual][i] + q[anterior][atual][i];
                //std::cout << "saindo do for" << std::endl;
            }
        }
        if(proximoPonto != -1){
            selecionados[proximoPonto] = true;
            maisprox.push_back(proximoPonto);
        }
    }

    //std::cout << "tamanho do vetor " << maisprox.size() << std::endl;

    return maisprox;
}

// Função para encontrar o ciclo hamiltoniano usando KNN
std::vector<int> KNN(int tam, const std::vector<std::vector<double>>& distancias, std::vector<std::vector<std::vector<double>>>& q, int obsSize, std::vector<int> obstaculos, std::vector<std::vector<double>>& altitudes) {
    int n = distancias.size() - obsSize;
    std::vector<bool> visitado(n + obsSize, false);
    std::vector<int> cicloHamiltoniano;

    int pontoAtual = 0;
    int pontoant = 0;
    int pontoInicial = pontoAtual;
    int valorTotalAcumulado = 0;

    cicloHamiltoniano.push_back(pontoAtual);
    visitado[pontoAtual] = true;

    while (cicloHamiltoniano.size() <= n) {

        //std::cout << "tam: " << cicloHamiltoniano.size() << std::endl;

        std::vector<int> proxPonto = encontrarProximoPonto(pontoant, pontoAtual, distancias, q, visitado, pontoInicial, cicloHamiltoniano, obsSize, obstaculos, altitudes);

        if(proxPonto.size() == 0){
            break;
        }

        std::mt19937 rng(static_cast<unsigned>(std::time(0)));

        std::uniform_int_distribution<size_t> dist(0, proxPonto.size() - 1);
    
        // Escolha um índice aleatório
        size_t randomIndex = dist(rng);

        //std::cout << "escolheu ponto " << randomIndex << " " << proxPonto[randomIndex] << std::endl;

        int proximoPonto = proxPonto[randomIndex];

        valorTotalAcumulado += (distancias[pontoAtual][proximoPonto] + q[pontoant][pontoAtual][proximoPonto]);
        if(!visitado[proximoPonto]){
            //std::cout << "entrou no if" << std::endl;
            cicloHamiltoniano.push_back(proximoPonto);
            visitado[proximoPonto] = true;
            pontoant = pontoAtual;
            pontoAtual = proximoPonto;
        }
    }
    cicloHamiltoniano.push_back(pontoInicial);

    //std::cout << "saiu do knn" << std::endl;

    return cicloHamiltoniano;
}

std::vector<int> grasp(int tam, const std::vector<std::vector<double>>& distancias, std::vector<std::vector<std::vector<double>>>& q, int obsSize, std::vector<int> obstaculos, std::vector<std::vector<double>>& altitudes){
    float lim = 2000000;
    std::vector<int> S;
    std::vector<int> best;
    std::pair<std::vector<int>, double> par;
    std::vector<int> s;
    bool melhora = true;
    int qtdit = 0;
    int melhorou = 0;

    std::cout << "comecando o grasp " << std::endl;
    while(melhora){
        S = KNN(tam, distancias, q, obsSize, obstaculos, altitudes);
        std::cout << "passou pela insercao mais barata " << std::endl;
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
            best = S;
            lim = valor;
            melhorou = qtdit;
        }

        if(qtdit - melhorou >= 100){
            melhora = false;
        }
       std::cout << "2opt: " << lim << std::endl;
    }
    std::cout << "Qtd de iterações: " << qtdit - 1 << std::endl;
    return best;
}

int main() {
    std::string mapas = "289_pontos/mapas17.txt";
    std::ofstream resultadosFile("resultados.txt");

    for(int nObs = 10; nObs > 0; nObs--){

        std::ofstream resultadosFiles("resultados" + std::to_string(11-nObs) + ".txt");

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

        std::istringstream iss(linhas[linhas.size() - nObs]);
        int obstaculo;
        while (iss >> obstaculo) {
            obstaculos_indices.push_back(obstaculo);
            std::cout << obstaculo << std::endl;
        }

        //std::vector<int> cicloHamiltoniano = {0, 7, 8, 9, 11, 17, 23, 29, 27, 26, 25, 24, 30, 31, 32, 33, 28, 22, 21, 20, 19, 18, 13, 14, 15, 16, 10, 3, 2, 1, 0};

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

        std::cout << std::endl;

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

        //std::cout << "passou da matriz de custos" << std::endl;

        // Matrizes que guardam a penalidade de altitude e ângulo
        std::vector<std::vector<double>> altitudes(n, std::vector<double>(n, 0.0));
        std::vector<std::vector<double>> all_angles(n, std::vector<double>(n, 0.0));
        std::vector<std::vector<std::vector<int>>> matrix(n, std::vector<std::vector<int>>(n, std::vector<int>(n, 0)));
        std::vector<std::vector<double>> distancias(n, std::vector<double>(n, std::numeric_limits<double>::infinity()));

        std::ofstream pathFile("path" + std::to_string(11-nObs) + ".txt");

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
                        //alt[v] = alt[u] + altitude;
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
            //altitudes[i][j] = alt[j];
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

        //std::cout << "passou do dijkstra" << std::endl;

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

        for(int i = 0; i < validos.size(); i++){
            std::cout << validos[i] << " ";
        }

        std::cout << std::endl;

        //std::cout << "passou dos ang1" << std::endl;

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

        //std::cout << "passou dos ang2" << std::endl;
        
        //std::vector<int> cicloHamiltoniano = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 21, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 34, 35, 47, 46, 45, 56, 67, 68, 69, 70, 71, 72, 73, 74, 75, 64, 63, 62, 61, 60, 48, 49, 50, 51, 52, 53, 54, 65, 76, 87, 98, 97, 96, 95, 94, 83, 82, 81, 80, 79, 78, 89, 90, 91, 92, 93, 104, 105, 106, 107, 108, 109, 120, 119, 118, 117, 116, 115, 114, 113, 112, 111, 110, 99, 88, 77, 66, 55, 44, 33, 22, 11, 0};

        //return 0;

        // se o último elemento da fronteira for 0, remova, pois pode atrapalhar na inserção mais barata
        // Recalcula a rota do gurobi
        //std::vector<int> cicloHamiltoniano = {0, 1, 2, 3, 4, 5, 6, 7, 19, 18, 17, 16, 15, 14, 13, 12, 23, 34, 45, 56, 57, 58, 59, 48, 37, 36, 26, 27, 40, 41, 42, 53, 52, 62, 61, 60, 49, 38, 28, 29, 30, 31, 43, 54, 65, 64, 63, 73, 72, 71, 70, 69, 68, 67, 77, 89, 90, 91, 92, 93, 94, 95, 96, 97, 117, 116, 115, 114, 113, 102, 82, 83, 84, 85, 86, 87, 98, 109, 108, 107, 106, 105, 104, 103, 101, 100, 99, 88, 66, 55, 44, 33, 22, 11, 0};

        std::vector<int> cicloHamiltoniano = grasp(validos.back(), distancias, q, obstaculos_indices.size(), validos, altitudes);
        //std::vector<int> melhorRota = construirCaminhoInsercaoMaisBarata(distancias, fronteira2, q, altitudes, obstaculos_indices.size());
        //std::vector<int> cicloHamiltoniano = melhorRota;
        //cicloHamiltoniano = Local_Search(cicloHamiltoniano, distancias, 0, q, altitudes).first;
        //std::cout << "chegou no final" << std::endl;
        // Calcular o valor total do ciclo
        //std::cout << "passou do grasp" << std::endl;

        double total = 0;
        for(int i = 0; i < cicloHamiltoniano.size() - 1; i++){
            total += (distancias[cicloHamiltoniano[i]][cicloHamiltoniano[i+1]] + q[cicloHamiltoniano[(i > 0) ? i-1 : i]][cicloHamiltoniano[i]][cicloHamiltoniano[i+1]]);
            std::cout << "Cidade " << cicloHamiltoniano[i] + 1 << " -> " << cicloHamiltoniano[i+1] + 1 << " com " << distancias[cicloHamiltoniano[i]][cicloHamiltoniano[i+1]] << " de distancia, com penalizacao de: " << q[cicloHamiltoniano[(i > 0) ? i-1 : i]][cicloHamiltoniano[i]][cicloHamiltoniano[i+1]] << std::endl;
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

        if (resultadosFile.is_open()) {

            std::string formattedNumber = formatWithComma(total);

            resultadosFile << formattedNumber << std::endl;

        } else {
            std::cerr << "Erro ao abrir o arquivo resultados.txt para escrita." << std::endl;
            return 1;
        }

        if (resultadosFiles.is_open()) {
        resultadosFiles << "Coordenadas:" << std::endl;
        for (const auto& tuple : coord) {
        resultadosFiles << std::get<0>(tuple) << " " << std::get<1>(tuple) << " " << std::get<2>(tuple) << std::endl;
        }

        resultadosFiles << "Ciclo Hamiltoniano:" << std::endl;
        for (int vertex : cicloHamiltoniano) {
            resultadosFiles << vertex << " ";
        }
        resultadosFiles << std::endl;

        resultadosFiles << "Obstaculos:" << std::endl;
        for (const auto& obstacle : obstaculos) {
            resultadosFiles << std::get<0>(obstacle) << " " << std::get<1>(obstacle) << std::endl;
        }

        resultadosFiles << "Valor Total do Ciclo: " << total << std::endl;

        resultadosFiles.close();
        } else {
            std::cerr << "Erro ao abrir o arquivo resultados.txt para escrita." << std::endl;
            return 1;
        }
    }
    resultadosFile.close();
    return 0;
}
