#ifndef DUAL_STREAM_H
#define DUAL_STREAM_H

#include <iostream>
#include <fstream>

class dual_stream {
        private:
            
        public:
            std::ofstream logfile;
            dual_stream(const char* file)
            {
                logfile.open(file,std::ios::out|std::ios::app|std::ios::ate); 
        
                if (!(logfile.is_open())) {
                    std::cerr << "[]: Couldn't open file \"" << file << "\" for logging.\n";
                    this->~dual_stream(); /* Destroy the object */
                }

            }

            ~dual_stream()
            {
                if (logfile.is_open())
                    logfile.close();
            }

            dual_stream& operator<< (std::ostream& (*pfun)(std::ostream&))
            {
                pfun(logfile);
                pfun(std::cout);
                return *this;
            }
};

template <class T>
dual_stream& operator<<(dual_stream& ds, const T& out)
{
    std::cout << out;
    ds.logfile << out;

    return ds;
};

#endif