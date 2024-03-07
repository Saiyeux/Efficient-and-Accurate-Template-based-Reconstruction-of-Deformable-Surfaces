#include "objReader.h"
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

void readObj(std::vector<double> &xyz, std::vector<int> &faces, int &number_vertices, int &number_faces) {
    std::string file_path = "../data/Hamlyn/ReferenceMesh2.obj";//ReferenceMesh2.obj";
    // std::vector<double> points;
    // Dateistream-Objekt zum Lesen der Objektdatei erstellen
    std::ifstream obj_file(file_path);

    // Überprüfen, ob die Datei erfolgreich geöffnet wurde
    if (!obj_file.is_open()) {
        std::cerr << "Incorrect path to the Obj-file." << std::endl;
        return;
    }

    // Lesen und Ausgabe des Inhalts der Objektdatei
    std::string line;
    std::getline(obj_file, line);
    std::getline(obj_file, line);
    std::getline(obj_file, line);
    std::stringstream sstream(line);
    std::istringstream iss;
    std::string word;
    std::string token;
    sstream >> word;sstream >> word;
    sstream >> word;
    sstream >> word;sstream >> word;
    // std::cout << word << std::endl;
    number_vertices = stoi(word);

    std::getline(obj_file, line);
    sstream.str(line);
    sstream.seekg(0);
    sstream.seekp(0);
    // std::stringstream sstream(line);
    sstream >> word;sstream >> word;
    sstream >> word;
    sstream >> word;sstream >> word;
    // std::cout << word << std::endl;
    number_faces = stoi(word);

    // std::cout << number << std::endl;
    
    // double points[number_vertices * 3];
    // double faces[number_faces * 3];
    int idx_points = 0;
    while (std::getline(obj_file, line)) {
        if(line[0] != 'v' && line[0] != 'f')
            continue;
        if(line[1] == 'n')
            continue;
            
        if (line[0] == 'v'){
            // std::stringstream sstream(line);
            sstream.str(line);
            sstream.seekg(0);
            sstream.seekp(0);
            sstream >> word;sstream >> word;
            double x = stod(word);
            sstream >> word;
            double y = stod(word);
            sstream >> word;
            double z = stod(word);
            xyz.push_back(x);
            xyz.push_back(y);
            xyz.push_back(z);
            idx_points++;
        } else if(line[0] == 'f') {
            sstream.str(line);
            sstream.seekg(0);
            sstream.seekp(0);
            sstream >> word;sstream >> word;
            iss.str(word);
            std::getline(iss, token, '/');
            int f1 = stoi(token);
            sstream >> word;
            iss.str(word);
            
            std::getline(iss, token, '/');
            int f2 = stoi(token);
            sstream >> word;
            iss.str(word);
            
            std::getline(iss, token, '/');
            int f3 = stoi(token);
            faces.push_back(f1);
            faces.push_back(f2);
            faces.push_back(f3);
        }
        // std::cout << line[0] == 'v' << std::endl;
        // std::cout << line << std::endl;
        // std::stringstream sstream(line);
  		// std::string word;

        // while(sstream >> word) {
        //     std::cout << word << std::endl;
        // }
        
    }
    
    // double *tmp = points.data();
    // xyz = tmp;
    // Schließen der Datei
    obj_file.close();
    // for (int i = 0; i< 1680; i++) {
    //     std::cout << xyz[i*3 + 0] << " " << xyz[i*3 + 1] << " " << xyz[i*3 + 2] << " " << std::endl;  
    // }
    // exit(1);
}