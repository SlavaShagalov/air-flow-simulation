#ifndef IMAGE_H
#define IMAGE_H

#include <vector>



class Image
{
private:
    int _width;
    int _height;

    std::vector<unsigned char> _data;

public:
    Image() : _width(0), _height(0) {}
};

#endif // IMAGE_H
