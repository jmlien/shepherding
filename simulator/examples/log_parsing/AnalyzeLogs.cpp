#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cassert>
#include <cstdlib>
#include <map>
#include "log_parser.h"
#include "log_stats.h"
using namespace std;


#ifdef WIN32
const char dir_char = '\\';
#else
const char dir_char = '/';
#endif


// argv[1] = list of input log file names
// argv[2] = output directory for text-files

int main(int argc, char** argv)
{
    assert(argc > 1 && "usage: AnalyzeLogs list_of_logs output_dir");
    
    // read in the list of filenames
    ifstream fin(argv[1]);
    vector<string> filenames;
    string filename;
    while(fin)
    {
        getline(fin, filename);
        filenames.push_back(filename);
    }
    fin.close();
    
    // correct the output path (argv[1]) if necessary
    string path = argv[2];
    if(path[path.size() - 1] != dir_char)
    {
        path += dir_char;
    }
    
    // loop through each log file
    const int size = filenames.size() - 1;
    std::vector<LOG_DATA> logs;
    logs.reserve(size);
    for(int i = 0; i < size; i++)
    {   
        cout << "Parsing log: " << filenames[i] << " ...\n";
        // parse log and store trial results in struct
        logs.push_back(LOG_DATA::Parse(filenames[i]));
    }
    
    // output the trial results to a text file, trials on
    // the same environment get appended to the same file
    //
    // NOTE: this is probably the main part you would want to change
    //
    for(int i = 0; i < size; i++)
    {
        const LOG_DATA& log = logs[i];
        const float mouse = TotalMouseDistance(log);
        const float shepherd = TotalShepherdDistance(log, 0);
        const float separation = AverageSheepSeparation(log);
        const float groups = AverageNumberOfGroups(log);
        
        // remove directory separator from the filename if necessary
        std::string envname = log.environment;
        int found_dir_char = envname.find(dir_char);
        while(found_dir_char != -1)
        {
            envname[found_dir_char] = '-';
            found_dir_char = envname.find(dir_char, found_dir_char);
        }
        
        // append results to output files grouped by environment name
        filename = path + envname + ".txt";
        ofstream fout(filename.c_str(), ios::app);
        fout << log.user_id << "\t"
             << log.total_time << "\t"
             << log.total_steps << "\t"
             << log.success << "\t"
             << mouse << "\t"
             << shepherd << "\t"
             << separation << "\t"
             << groups << "\n";
        fout.close();
    }
    
    return 0;
}
        
        
        
        
