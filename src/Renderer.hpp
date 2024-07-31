#pragma once
#include <fstream>
#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

class Renderer
{
    static const int max_elem = 255;
    int _rows;
    int _cols;

public:
    std::string _filename = "../output.ppm";

    Renderer(int rows, int cols, std::string filename = "") : _rows(rows), _cols(cols)
    {
        if (_filename.length() > 0)
            filename = std::move(filename);
    }

    void render(Eigen::Vector3f pixels[]) const
    {
        std::ofstream ofs(_filename);
        assert(ofs.is_open());
        ofs << "P3 " << _cols << " " << _rows << " " << max_elem << '\n';
        for (int i = 0; i < _rows; i++)
        {
            for (int j = 0; j < _cols; j++)
            {
                int r = pixels[_cols * i + j].x() * max_elem;
                int g = pixels[_cols * i + j].y() * max_elem;
                int b = pixels[_cols * i + j].z() * max_elem;
                ofs << r << ' ' << g << ' ' << b << ' ';
            }
            ofs << '\n';
        }
        ofs.close();
    }
};