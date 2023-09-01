#include <iostream>
#include <fstream>

int main(){
    
    int x = 10, y = 10, v = 1;
    std::ofstream arquivo("mapas.txt");

    if(!arquivo){
        std::cerr << "Não foi possível abrir o arquivo." << std::endl;
        return 1;
    }

    for(int i = 1; i < 16; i++){
        for(int j = 1; j < 9; j++){
            arquivo << v << " " << x << " " << y << std::endl;
            x+=20;
            v++;
        }
        y+=20;
        x = 10;
    }

    arquivo.close();

    return 0;
}