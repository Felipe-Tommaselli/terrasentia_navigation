/*
Created by Karan on 6/6/18.
Modified by Mateus
*/

#pragma once

#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <string>

using namespace std;

class ParameterReader
{
    public:
        ParameterReader(string filename = "mpc.config")
        {
            cout << "ParameterReader filename " << filename << endl;
            bool verbose = false;

            ifstream fin( filename.c_str() );
            if (!fin)
            {
                cerr << "parameter file does not exist." << endl;
                return;
            }

            if(verbose)
                cout << "\n------ Reading in Parameter File: " << filename << endl;;

            while(!fin.eof())
            {                
                string str;
                getline( fin, str );
                
                if(verbose)
                    cout << "Line Read: " << str << endl;
                
                if (str[0] == '#')
                    continue;

                int pos = str.find("=");
                if (pos == -1)
                {
                    if(verbose)
                        cout << "pos found = -1 ---- Continuing loop...\r\n";
                    continue;
                }
                string key = str.substr( 0, pos );
                string value = str.substr( pos+1, str.length() );

                this->data[key] = value;

                if(verbose)
                    cout << "Key Found with Value: " << key << " -> " << value << endl;

                if ( !fin.good() )
                {
                    cout << "\r\n";
                    break;
                }
            }
        }

        string getData( string key )
        {
            map<string, string>::iterator iter;
            iter = this->data.find(key.c_str());
            std::cout << "Searching for key (" << key.c_str() << ") => " << this->data[key] << '\n';
            if (iter == this->data.end())
            {
                cerr << "Parameter name " << key << " not found!" << endl;
                return string("NOT_FOUND");
            }
            return iter->second;
        }

    public:
        map<string, string> data;
};
