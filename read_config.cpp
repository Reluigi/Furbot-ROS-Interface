#include <fstream>
#include <string>
#include <iostream>
#include <cctype>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <cstdlib>


using namespace std;

int main() {

    //*******************************
    //Global furbot variables:
    int ciao;
    int hi;
    //*******************************

    const int n = 64;
    string lines[n];
    string line;

    string s;

    string var_name;
    string var_value;

    int numbers[n];
    int a = 0;



    ifstream furbot_data;

    furbot_data.open("furbot_config.txt");
    if (!furbot_data) {
        cout << "Unable to open file 'furbot_config.txt'";
        exit(1); // terminate with error
    }

    //load all lines
    for(int i = 0; furbot_data >> line; i++) {
        lines[i] = line;
    }
    //Set variables with values
    for(int i = 0; i<n; i++){
        s=lines[i];
        if(isdigit(s[0])){
            numbers[a] = stoi(s);
            a++;
        }
    }
    furbot_data.close();

    //*******************************
    //Set values to the variables
    ciao = numbers[0];
    hi = numbers[1];
    //*******************************

    return 0;
}

