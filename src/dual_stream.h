#ifndef DUAL_STREAM_H
#define DUAL_STREAM_H

#include <iostream>
#include <fstream>

class dual_stream {
public:
    dual_stream(std::ofstream& os1, std::ostream& os2) : os1(os1), os2(os2) {}

    template<class T>
    dual_stream& operator<<(const T& x) {
        os1 << x;
        os2 << x;
        return *this;
    }
private:
    std::ofstream& os1;
    std::ostream& os2;
};


#endif