
#include <boost/python.hpp>


#include <fstream>
#include <iostream>
using namespace std;
 
int main() {
 
    ///Test d'existance du fichier
    ifstream fichier("hello.py");
 
    if (fichier.fail()) {
        cout << "Fichier de script introuvable : " << "hello.py" <<"\n";
        return 0;
    }
 
    ///Lance le script 
    cout << "\n--- Execution du script : " << "hello.py" <<" ---\n";
 
    // Ouvre le script python a executer
    FILE* pyFile = fopen("hello.py", "r");
 
    Py_Initialize();
 
    //Test : ecriture d'une commande Python
    PyRun_SimpleString(    "print \"Bonjour\"");
 
    // Execute le script
    PyRun_AnyFile(pyFile, "hello.py");
 
    Py_Finalize();
 
    return 1;
}