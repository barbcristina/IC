#include <iostream>
#include <fstream>
#include <iomanip>

int main(){
    
    int x = 10, y = 10, v = 0;
    int w, h;
    float z = 200;
    std::ofstream arquivo("mapas18.txt");

    std::cin >> h >> w;

    if(!arquivo){
        std::cerr << "Não foi possível abrir o arquivo." << std::endl;
        return 1;
    }

    for(int i = 0; i < h; i++){
        for(int j = 0; j < w; j++){
            arquivo << v << " " << x << " " << y << " " << z << std::endl;
            x+=20;
            z += 0.07;
            v++;
        }
        y+=20;
        x = 10;
    }

    arquivo.close();

    return 0;
}