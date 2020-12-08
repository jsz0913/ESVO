#ifndef ESVO_CORE_TOOLS_CAYLEY_H
#define ESVO_CORE_TOOLS_CAYLEY_H

//嵌套命名空间，提供一个矩阵和向量的转换

#include <Eigen/Eigen>
namespace esvo_core
{
namespace tools
{
Eigen::Matrix3d cayley2rot( const Eigen::Vector3d & cayley);
Eigen::Vector3d rot2cayley( const Eigen::Matrix3d & R );
}// namespace tools
}// namespace esvo_core


#endif //ESVO_CORE_TOOLS_CAYLEY_H
