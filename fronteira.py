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

    for (int i = 1; i <= 64; i++) {
        if (i + x in obstaculos) {
            int it = i;
            while (true) {
                it += x;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end() && it < 65) {
                    break;
                } else if (it > 64) {
                    fronteira.push_back(i);
                    break;
                }
            }
        }
    }

    for (int i = 1; i <= 64; i++) {
        if (i + x in obstaculos && i not in fronteira) {
            int it = i + x;
            while (true) {
                it--;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end()) {
                    break;
                } else if (it % 8 == 1) {
                    fronteira.push_back(i);
                    break;
                }
            }
        }
    }

    for (int i = 1; i <= 64; i++) {
        if (i + x in obstaculos && i not in fronteira) {
            int it = i + x;
            while (true) {
                it++;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end()) {
                    break;
                } else if (it % 8 == 0) {
                    fronteira.push_back(i);
                    break;
                }
            }
        }
    }

    for (int i = 1; i <= 64; i++) {
        if (i - x in obstaculos) {
            int it = i;
            while (true) {
                it -= x;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end() && it > 0) {
                    break;
                } else if (it < 0) {
                    fronteira.push_back(i);
                    break;
                }
            }
        }
    }

    for (int i = 1; i <= 64; i++) {
        if (i - x in obstaculos) {
            int it = i - x;
            while (true) {
                it++;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end()) {
                    break;
                } else if (it % 8 == 0) {
                    fronteira.push_back(i);
                    break;
                }
            }
        }
    }

    for (int i = 1; i <= 64; i++) {
        if (i - x in obstaculos) {
            int it = i - x;
            while (true) {
                it--;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end()) {
                    break;
                } else if (it % 8 == 1) {
                    fronteira.push_back(i);
                    break;
                }
            }
        }
    }

    for (int i = 1; i <= 64; i++) {
        if (i + 1 in obstaculos && i not in obstaculos) {
            int it = i;
            while (true) {
                it++;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end()) {
                    break;
                } else if (it % 8 == 0 && i not in fronteira) {
                    fronteira.push_back(i);
                    break;
                }
            }
        }
    }

    for (int i = 1; i <= 64; i++) {
        if (i - 1 in obstaculos && i not in obstaculos) {
            int it = i;
            while (true) {
                it--;
                if (std::find(obstaculos.begin(), obstaculos.end(), it) == obstaculos.end()) {
                    break;
                } else if (it % 8 == 1 && i not in fronteira) {
                    fronteira.push_back(i);
                    break;
                }
            }
        }
    }

    for (int i = 0; i < fronteira.size(); i++) {
        std::cout << fronteira[i] << " ";
    }
    std::cout << std::endl;

    return 0;
}
