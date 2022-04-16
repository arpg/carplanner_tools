#ifndef _IO_CONVERSION_TOOLS
#define _IO_CONVERSION_TOOLS

#include <Eigen/Eigen>
#include <string>

template<typename Derived>
inline std::string convertEigenMatrix2String(const Eigen::MatrixBase<Derived>& mat, uint num_decimals=2, std::string row_delimiter=", ",  std::string col_delimiter="\n", std::string mat_prefix="")
{
    std::string str;
    for (uint r=0; r<mat.rows(); r++)
    { 
        if (r>0) { str += col_delimiter; }
        str += mat_prefix;
        for (uint c=0; c<mat.cols(); c++)
        {
            if (c>0) { str += row_delimiter; }
            std::string tempstr = std::to_string(mat(r,c));
            tempstr = tempstr.substr(0,tempstr.find_first_of(".")+num_decimals+1);
            str += tempstr;
        }
    }
    return str;
}

#endif