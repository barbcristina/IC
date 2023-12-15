#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <random>

int main() {

    int n = 1000000; // Número de zeros desejados

    std::vector<int> zeros(n, 0);

    std::vector<double> negativos;
    std::vector<double> positivos;

    // Gerando números negativos entre -1000 e 0
    for (int i = 0; i < n; ++i) {
        negativos.push_back(round(((double)rand() / RAND_MAX) * -1000));
    }

    // Gerando números positivos entre 0 e 1000
    for (int i = 0; i < n; ++i) {
        positivos.push_back(round(((double)rand() / RAND_MAX) * 1000));
    }

    // Mesclando os três vetores
    std::vector<double> cores;
    cores.reserve(negativos.size() + zeros.size() + positivos.size());
    cores.insert(cores.end(), negativos.begin(), negativos.end());
    cores.insert(cores.end(), zeros.begin(), zeros.end());
    cores.insert(cores.end(), positivos.begin(), positivos.end());

    // Embaralhando os elementos
    std::shuffle(cores.begin(), cores.end(), std::default_random_engine());
    
    auto inicio = std::chrono::high_resolution_clock::now();

    // Ordenando os elementos
    std::sort(cores.begin(), cores.end());

    auto fim = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> tempo_decorrido = fim - inicio;
    std::cout << "Tempo decorrido: " << tempo_decorrido.count() << " segundos" << std::endl;

    return 0;
}