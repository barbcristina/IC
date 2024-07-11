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

std::pair<std::vector<int>, double> OPT2(std::vector<int>& rota, const std::vector<std::vector<double>>& distancias, double dist, std::vector<std::vector<std::vector<double>>>& q,  std::vector<std::vector<double>>& altitudes){
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

std::pair<std::vector<int>, double> Local_Search(std::vector<int>& rota, const std::vector<std::vector<double>>& distancias, double dist, std::vector<std::vector<std::vector<double>>>& q,  std::vector<std::vector<double>>& altitudes){
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

std::vector<int> construirFronteira(std::vector<int>& obstaculos, int largura, int altura, bool &guloso) {
    std::vector<int> fronteira;
    int x = altura;
    int y = largura;
    int xy = x*y;

    // Inferior
    for (int i = 0; i < y; i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i) == obstaculos.end()) {
            fronteira.push_back(i);
            //std::cout << "inserindo inferior normal: " <<i << std::endl;
        }
    }

    // Direita
    for (int i = (x-1) + x; i < xy; i += x) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i) == obstaculos.end()) {
            fronteira.push_back(i);
            //std::cout << "inserindo direita normal: " <<i << std::endl;
        }
    }

    // Superior
    for (int i = xy - 2; i > x * (y-1) - 1; i--) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i) == obstaculos.end()) {
            fronteira.push_back(i);;
            //std::cout << "inserindo superior normal: " <<i << std::endl;
        }
    }

    // Esquerda
    for (int i = x * (y-2); i > 0; i -= x) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i) == obstaculos.end()) {
            fronteira.push_back(i);
            //std::cout << "inserindo esquerda normal: " <<i << std::endl;
        }
    }

    if(fronteira.size() == (altura*2 + largura*2)-4)
        return fronteira;

    guloso = true;

    // Superior
    for (int i = 0; i < xy; i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i + x) != obstaculos.end() && std::find(fronteira.begin(), fronteira.end(), i) == fronteira.end() && std::find(obstaculos.begin(), obstaculos.end(), i) == obstaculos.end()) {
            int it = i;
            while (true) {
                it += x;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end() && it < (xy)) {
                    break;
                } else if (it > xy-1) {
                    fronteira.push_back(i);
                    //std::cout << "inserindo superior: "<<i << std::endl;
                    break;
                }
            }
        }
    }

    // Esquerda subindo
    for (int i = 0; i < xy; i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i + x) != obstaculos.end() && std::find(fronteira.begin(), fronteira.end(), i) == fronteira.end() && std::find(obstaculos.begin(), obstaculos.end(), i) == obstaculos.end()) {
            int it = i + x;
            while (true) {
                it--;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end()) {
                    break;
                } else if (it % x == 0) {
                    fronteira.push_back(i);
                    //std::cout << "inserindo esquerda subindo: "<< i << std::endl;
                    break;
                }
            }
        }
    }

    // Direita subindo
    for (int i = 0; i < xy; i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i + x) != obstaculos.end() && std::find(fronteira.begin(), fronteira.end(), i) == fronteira.end() && std::find(obstaculos.begin(), obstaculos.end(), i) == obstaculos.end()) {
            int it = i + x;
            while (true) {
                it++;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end()) {
                    break;
                } else if (it % x == x-1) {
                    fronteira.push_back(i);
                    //std::cout << "inserindo direita subindo: "<<i << std::endl;
                    break;
                }
            }
        }
    }

    // Inferior
    for (int i = 0; i < xy; i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i - x) != obstaculos.end() && std::find(obstaculos.begin(), obstaculos.end(), i) == obstaculos.end() && std::find(fronteira.begin(), fronteira.end(), i) == fronteira.end()) {
            int it = i;
            while (true) {
                it -= x;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end() && it > 0) {
                    break;
                } else if (it < 0) {
                    fronteira.push_back(i);
                    //std::cout << "inserindo inferior: "<< i << std::endl;
                    break;
                }
            }
        }
    }

    // Direita descendo
    for (int i = 0; i < xy; i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i - x) != obstaculos.end() && std::find(obstaculos.begin(), obstaculos.end(), i) == obstaculos.end() && std::find(fronteira.begin(), fronteira.end(), i) == fronteira.end()) {
            int it = i - x;
            while (true) {
                it++;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end()) {
                    break;
                } else if (it % x == x-1) {
                    fronteira.push_back(i);
                    //std::cout << "inserindo direita descendo: "<< i << std::endl;
                    break;
                }
            }
        }
    }

    // Esquerda descendo
    for (int i = 0; i < xy; i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i - x) != obstaculos.end() && std::find(obstaculos.begin(), obstaculos.end(), i) == obstaculos.end() && std::find(fronteira.begin(), fronteira.end(), i) == fronteira.end()) {
            int it = i - x;
            while (true) {
                it--;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end()) {
                    break;
                } else if (it % x == 0) {
                    fronteira.push_back(i);
                    //std::cout << "inserindo esquerda descendo: "<< i << std::endl;
                    break;
                }
            }
        }
    }

    // Direita
    for (int i = 0; i < xy; i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i + 1) != obstaculos.end() && std::find(obstaculos.begin(), obstaculos.end(), i) == obstaculos.end() && std::find(fronteira.begin(), fronteira.end(), i) == fronteira.end()) {
            int it = i;
            while (true) {
                it++;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end()) {
                    break;
                } else if (it % x == x-1) {
                    fronteira.push_back(i);
                    //std::cout << "inserindo direita: "<<i << std::endl;
                    break;
                }
            }
        }
    }

    // Esquerda
    for (int i = 0; i < xy; i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i - 1) != obstaculos.end() && std::find(obstaculos.begin(), obstaculos.end(), i) == obstaculos.end() && std::find(fronteira.begin(), fronteira.end(), i) == fronteira.end()) {
            int it = i;
            while (true) {
                it--;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end()) {
                    break;
                } else if ((it % x) == 0) {
                    fronteira.push_back(i);
                    //std::cout << "inserindo esquerda: "<<i << std::endl;
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


std::vector<int> encontrarProximoPontoNaoVisitado(const std::vector<int>& caminho, const std::vector<std::vector<double>>& distancias, std::vector<bool>& visitado, std::vector<std::vector<std::vector<double>>>& q,  std::vector<std::vector<double>>& altitudes) {
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

    //std::cout << "Número aleatório entre " << min << " e " << max << ": " << numero_aleatorio << std::endl;

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
std::vector<int> construirCaminhoInsercaoMaisBarata(const std::vector<std::vector<double>>& distancias, const std::vector<int>& fronteira, std::vector<std::vector<std::vector<double>>>& q,  std::vector<std::vector<double>>& altitudes, int obsSize) {
    int n = distancias.size();
    std::vector<bool> visitado(n, false);
    std::vector<int> caminho; // Caminho inicial com a fronteira
    for(int i = 0; i < fronteira.size(); i++){
        caminho.push_back(fronteira[i]);
        visitado[fronteira[i]] = true;
    }
    //std::cout << "criou a fronteira " << std::endl;
    while (caminho.size() < n-obsSize) {
        //std::cout << caminho.size() << std::endl;
        std::vector<int> proxPonto = encontrarProximoPontoNaoVisitado(caminho, distancias, visitado, q, altitudes);
        
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

std::vector<int> grasp(const std::vector<std::vector<double>>& distancias, const std::vector<int>& fronteira, std::vector<std::vector<std::vector<double>>>& q,  std::vector<std::vector<double>>& altitudes, int obsSize){
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
        //std::cout << "construtiva: " << valor << std::endl;
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

        if(qtdit - melhorou >= 100){
            melhora = false;
        }
        //std::cout << "2opt: " << lim << std::endl;
    }
    std::cout << "Qtd de iterações: " << qtdit - 1 << std::endl;
    return best;
}

int main() {
    std::string mapas = "36_pontos/mapas6.txt";

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
    int nObs = 1;
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

    for (int i : validos) {
    for (int j : validos) {
        for (int k : validos) {
            if (c[i][j] == std::numeric_limits<double>::infinity() && c[j][k] == std::numeric_limits<double>::infinity() && i != j && j != k) {
                q[i][j][k] = (all_angles[i][j] + all_angles[j][k]) + q[matrix[i][j][matrix[i][j].size()-2]][j][matrix[j][k][1]];
            } else if (c[i][j] == std::numeric_limits<double>::infinity() && c[j][k] != std::numeric_limits<double>::infinity() && i != j && j != k) {
                q[i][j][k] = (all_angles[i][j] + all_angles[j][k]) + q[matrix[j][k][1]][j][k];
            } else if (c[i][j] != std::numeric_limits<double>::infinity() && c[j][k] == std::numeric_limits<double>::infinity() && i != j && j != k) {
                q[i][j][k] = (all_angles[i][j] + all_angles[j][k]) + q[i][j][matrix[i][j].size()-2];
            }
        }
    }
}


    //return 0;

    bool guloso = false;
    std::vector<int> fronteira = construirFronteira(obstaculos_indices, maiorx, maiory, guloso);
    std::cout << "Fronteira: ";
        for (int i : fronteira) {
            std::cout << i + 1 << " ";
        }


    // Método guloso para organizar os pontos da fronteira se necessario
    if(guloso){
        std::vector<int> fronteira2;
        fronteira2.push_back(0);
        int sz = fronteira.size();
        fronteira.erase(find(fronteira.begin(), fronteira.end(), 0));

        // Método guloso para organizar os pontos da fronteira
        while(fronteira2.size() < sz){
            int menor = 100000;
            int k = 0;
            // Encontra o ponto mais próximo do último ponto da fronteira
            for(int j = 0; j < fronteira.size(); j++){
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
            // Adiciona o elemento à fronteira
            if(fronteira2.back() != k)
                fronteira2.push_back(k);
        }

        //std::cout << "obstaculos: " << std::endl;
        //for(int i = 0; i < obstaculos_indices.size(); i++){
        //    std::cout << obstaculos_indices[i] << std::endl;
        //}
        std::cout << "Fronteira2: ";
        for (int i : fronteira2) {
            std::cout << i << ", ";
        }

        fronteira = fronteira2;
    }

    //return 0;

    // se o último elemento da fronteira for 0, remova, pois pode atrapalhar na inserção mais barata
    // Recalcula a rota do gurobi
    //std::vector<int> cicloHamiltoniano = {0, 1, 2, 3, 10, 11, 17, 23, 29, 28, 33, 32, 31, 30, 24, 18, 13, 14, 15, 21, 20, 19, 25, 26, 27, 22, 16, 9, 8, 7, 0};

    std::vector<int> cicloHamiltoniano = grasp(distancias, fronteira, q, altitudes, obstaculos_indices.size());
    //std::vector<int> melhorRota = construirCaminhoInsercaoMaisBarata(distancias, fronteira2, q, altitudes, obstaculos_indices.size());
    //std::vector<int> cicloHamiltoniano = melhorRota;
    //cicloHamiltoniano = Local_Search(cicloHamiltoniano, distancias, 0, q, altitudes).first;
    std::cout << "chegou no final" << std::endl;

    double total = 0;
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
        total += q[cicloHamiltoniano[(i > 0) ? i-1 : i]][cicloHamiltoniano[i]][cicloHamiltoniano[i+1]];
        //std::cout << "Penalizacao e angulo: " << cicloHamiltoniano[(i > 0) ? i-1 : i] + 1 << " -> " << cicloHamiltoniano[i] + 1 << " -> " << cicloHamiltoniano[i+1] + 1 << " = " << q[cicloHamiltoniano[(i > 0) ? i-1 : i]][cicloHamiltoniano[i]][cicloHamiltoniano[i+1]] << ", " << (q[cicloHamiltoniano[(i > 0) ? i-1 : i]][cicloHamiltoniano[i]][cicloHamiltoniano[i+1]])/0.2 << std::endl;
    }

    //std::cout << "Qtd de penalizações de angulo 45: " << quarentacinco << std::endl;
    //std::cout << "Qtd de penalizações de angulo 90: " << noventa << std::endl;
    //std::cout << "Qtd de penalizações de angulo 135: " << centotrintacino << std::endl;
    //std::cout << "Qtd de penalizações de angulo 180: " << centooitenta << std::endl;
    //std::cout << "Total de penalização por angulo: " << total << std::endl;

    total = 0;

    // Calcular o valor total do ciclo
    for(int i = 0; i < cicloHamiltoniano.size() - 1; i++){
        total += (distancias[cicloHamiltoniano[i]][cicloHamiltoniano[i+1]] + q[cicloHamiltoniano[(i > 0) ? i-1 : i]][cicloHamiltoniano[i]][cicloHamiltoniano[i+1]]);
        std::cout << "Cidade " << cicloHamiltoniano[i] + 1 << " -> " << cicloHamiltoniano[i+1] + 1 << " com " << distancias[cicloHamiltoniano[i]][cicloHamiltoniano[i+1]] << " de distancia, com penalizacao de: " << q[cicloHamiltoniano[(i > 0) ? i-1 : i]][cicloHamiltoniano[i]][cicloHamiltoniano[i+1]] << std::endl;
    }

    //std::cout << "custo de : " << distancias[9][17] << " e angulo: " << q[5][9][17] << std::endl; 

    std::cout << "Custo total: " << total << std::endl;

    auto end_time = std::chrono::steady_clock::now();
    // Calcular tempo transcorrido
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    std::cout << "Tempo Gasto: " << duration.count() << " millisegundos" << std::endl;

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

        resultadosFile << "Valor Total do Ciclo: " << total << std::endl;

        resultadosFile.close();
    } else {
        std::cerr << "Erro ao abrir o arquivo resultados.txt para escrita." << std::endl;
        return 1;
    }

    return 0;
}