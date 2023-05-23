#pragma once

#include "common.h"



void save_features(std::string filename, Features const& xfeatures);
//void save_matches(std::string filename, XMatches const& matches);
//void save_matches_bin(std::string filename, XMatches const& matches);
void save_enables(std::string filename, std::vector<bool> const& enable);
void save_xcoordinates(std::string filename, Coordinates& xcoordinates);
void save_xdescriptors(std::string filename, Descriptors const& xdescriptors);
void save_xdescriptors_bin(std::string filename, Descriptors const& xdescriptors);
void save_xrange2(std::string filename, Range2 const& xrange2);
void save_base(std::string filename, FloatImage const& base);
void save_ximage (std::string filename, CImage const& ximage);

void load_features(std::string filename, Features& xfeatures);
//void load_matches(std::string filename, XMatches& matches);
void load_enables(std::string filename, std::vector<bool>& enables);

void load_xcoordinates(std::string filename, Coordinates& xcoordinates);


void load_xrange2(std::string filename, Range2& xrange2);
void load_xframe(std::string filename1, std::string filename2, CFrame& xframe);

void load_xdescriptors(std::string filename, Descriptors& xdescriptors);
void save_xdescriptors(std::string filename, Descriptors const& xdescriptors);
void load_base(std::string filename, FloatImage & base);

CImage create_ximage(int cols, int rows);
void free_ximage(CImage & ximage);

FloatImage create_xfloat_image(int rows, int cols);
void free_xfloat_image(FloatImage & ximage);

void free_xframe(CFrame & xframe);

uchar xat(CImage const& ximage, int x, int y);
uchar& xat(CImage & ximage, int x, int y);

uchar xat(CImage const& ximage, int x);
uchar& xat(CImage & ximage, int x);


s16 xat(FloatImage const& ximage, int x, int y);
s16 & xat(FloatImage & ximage, int x, int y);
s16 xat(FloatImage const& ximage, int x);
s16 & xat(FloatImage & ximage, int x);

// XCoordinate const& xat(XCoordinates const& xcoordinates, int i);
// XCoordinate & xat(XCoordinates & xcoordinates, int i);

// XDescriptor const& xat(XDescriptors const& xdescriptors, int i);
// XDescriptor & xat(XDescriptors & xdescriptors, int i);

s16 xat(FloatImage13x13 const& image, int i);
s16 & xat(FloatImage13x13 & image, int i);


s16 xat(Descriptor const& xdescriptor, int i);
s16 & xat(Descriptor & xdescriptor, int i);


void check_outputs(std::string folder, std::string filename, std::string name);
void check_outputs_bin(std::string folder, std::string filename, std::string name);

std::string add_slash(std::string name);
