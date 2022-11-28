#ifndef BASECOLOR_H
#define BASECOLOR_H

class Color {
private:
  unsigned char _alpha = 255;
  unsigned char _red = 255;
  unsigned char _green = 255;
  unsigned char _blue = 255;

public:
  enum { RED, GREEN, BLUE };

  Color() = default;
  Color(unsigned char r, unsigned char g, unsigned char b,
        unsigned char a = 255)
      : _alpha(a), _red(r), _green(g), _blue(b) {}

  unsigned char red() const { return _red; }
  unsigned char green() const { return _green; }
  unsigned char blue() const { return _blue; }
  unsigned char alpha() const { return _alpha; }

  void setRed(unsigned char red) { _red = red; }
  void setGreen(unsigned char green) { _green = green; }
  void setBlue(unsigned char blue) { _blue = blue; }
  void setAlpha(unsigned char alpha) { _alpha = alpha; }
};

#endif // BASECOLOR_H
