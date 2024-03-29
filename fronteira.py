obstaculos = [9, 10, 11, 29, 30, 31, 32, 41, 42, 63, 64, 7]
fronteira = []
x = 8
y = 8

#include <iostream>
#include <vector>

int main() {
    std::vector<int> obstaculos = {9, 10, 11, 29, 30, 31, 32, 41, 42, 63, 64, 7};
    std::vector<int> fronteira;
    int x = 8;
    int y = 8;

    // Esquerda
    for (int i = 1; i <= x * (y - 1) + 1; i += x) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i) == obstaculos.end()) {
            fronteira.push_back(i);
        }
    }

    // Superior
    for (int i = x * (y - 1) + 2; i <= x * x; i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i) == obstaculos.end()) {
            fronteira.push_back(i);
        }
    }

    // Direita
    for (int i = x * x - x; i >= x - 1; i -= x) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i) == obstaculos.end()) {
            fronteira.push_back(i);
        }
    }

    // Inferior
    for (int i = x - 1; i >= 1; i--) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i) == obstaculos.end()) {
            fronteira.push_back(i);
        }
    }

    // Print fronteira
    for (int i : fronteira) {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    // Superior
    for (int i = 1; i <= 64; i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i + x) != obstaculos.end()) {
            int it = i;
            while (true) {
                it += x;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end() && it < 65) {
                    std::cout << it << std::endl;
                    break;
                } else if (it > 64) {
                    fronteira.push_back(i);
                    std::cout << it << std::endl;
                    break;
                }
            }
        }
    }

    // Print fronteira
    for (int i : fronteira) {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    // Esquerda subindo
    for (int i = 1; i <= 64; i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i + x) != obstaculos.end() && std::find(fronteira.begin(), fronteira.end(), i) == fronteira.end()) {
            int it = i + x;
            while (true) {
                it--;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end()) {
                    std::cout << it << std::endl;
                    break;
                } else if (it % 8 == 1) {
                    fronteira.push_back(i);
                    std::cout << it << std::endl;
                    break;
                }
            }
        }
    }

    // Direita subindo
    for (int i = 1; i <= 64; i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i + x) != obstaculos.end() && std::find(fronteira.begin(), fronteira.end(), i) == fronteira.end()) {
            int it = i + x;
            while (true) {
                it++;
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

    // Print fronteira
    for (int i : fronteira) {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    // Inferior
    for (int i = 1; i <= 64; i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i - x) != obstaculos.end()) {
            int it = i;
            while (true) {
                it -= x;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end() && it > 0) {
                    std::cout << it << std::endl;
                    break;
                } else if (it < 0) {
                    fronteira.push_back(i);
                    std::cout << it << std::endl;
                    break;
                }
            }
        }
    }

    // Direita descendo
    for (int i = 1; i <= 64; i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i - x) != obstaculos.end()) {
            int it = i - x;
            while (true) {
                it++;
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

    // Esquerda descendo
    for (int i = 1; i <= 64; i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i - x) != obstaculos.end()) {
            int it = i - x;
            while (true) {
                it--;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end()) {
                    std::cout << it << std::endl;
                    break;
                } else if (it % 8 == 1) {
                    fronteira.push_back(i);
                    std::cout << it << std::endl;
                    break;
                }
            }
        }
    }

    // Direita
    for (int i = 1; i <= 64; i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i + 1) != obstaculos.end() && std::find(obstaculos.begin(), obstaculos.end(), i) == obstaculos.end()) {
            int it = i;
            while (true) {
                it++;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end()) {
                    std::cout << it << std::endl;
                    break;
                } else if (it % 8 == 0 && std::find(fronteira.begin(), fronteira.end(), i) == fronteira.end()) {
                    fronteira.push_back(i);
                    std::cout << it << std::endl;
                    break;
                }
            }
        }
    }

    // Esquerda
    for (int i = 1; i <= 64; i++) {
        if (std::find(obstaculos.begin(), obstaculos.end(), i - 1) != obstaculos.end() && std::find(obstaculos.begin(), obstaculos.end(), i) == obstaculos.end()) {
            int it = i;
            while (true) {
                it--;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end()) {
                    std::cout << it << std::endl;
                    break;
                } else if (it % 8 == 1 && std::find(fronteira.begin(), fronteira.end(), i) == fronteira.end()) {
                    fronteira.push_back(i);
                    std::cout << it << std::endl;
                    break;
                }
            }
        }
    }

    // Print fronteira
    for (int i : fronteira) {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    return 0;
}
