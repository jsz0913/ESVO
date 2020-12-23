#ifndef ESVO_CORE_CONTAINER_SMARTGRID_H
#define ESVO_CORE_CONTAINER_SMARTGRID_H

//字符串转换、伪随机序列生成、动态内存管理
#include <cstdlib>
#include <memory>
#include <list>
#include <vector>
#include <set>

namespace esvo_core
{
namespace container
{
// 模板h和cpp合为一
// T是一种能const cast的类
template<class T>
class SmartGrid
{
  public:
  typedef std::shared_ptr<SmartGrid> Ptr;
  typedef std::list<T> gridElements;
  //模板类型用typename说明是类型而不是静态变量
  typedef typename gridElements::iterator iterator;

  SmartGrid();
  SmartGrid(size_t rows, size_t cols);
  virtual ~SmartGrid();
  SmartGrid<T> &operator=(const SmartGrid &rhs);

  // utils
  void resize(size_t rows, size_t cols);
  void erode(size_t radius, size_t border = 0, double ratio = 0.5);
  void dilate(size_t radius);
  void clear();
  void clean(double var_threshold, double age_threshold, double range_max, double range_min);
  void reset();

  const T &operator()(size_t row, size_t col) const;
  const T &at(size_t row, size_t col) const;

  size_t rows() const;
  size_t cols() const;

  void set(size_t row, size_t col, const T &value);
  T &get(size_t row, size_t col);
  bool exists(size_t row, size_t col);

  iterator remove(iterator &it);
  iterator begin();
  iterator end();

  size_t size() const;
  void getNeighbourhood(size_t row, size_t col, size_t radius, std::vector<T *> &neighbours);

  private:
  //_grid是个向量，里面存了好多向量的指针，每个向量里存的T的指针
  std::vector<std::vector<T *> *> _grid;
  // 存储T的列表
  gridElements _elements;
  T _invalid; //invalid element that we can return when the element doesn't exist
};
  
// 两种构造时，默认的用default
template<class T>
SmartGrid<T>::SmartGrid() = default;

template<class T>
SmartGrid<T>::SmartGrid(size_t rows, size_t cols)
{
  //向量元素多时用reserve 避免内存空间问题
  _grid.reserve(rows);
  for (size_t r = 0; r < rows; r++)
  {
    _grid.push_back(new std::vector<T *>());
    //刚加入的向量指针取出，resize cols个空指针
    (*_grid.back()).resize(cols, NULL);
  }
}

//new std::vector<T *>() delete new的部分
template<class T>
SmartGrid<T>::~SmartGrid()
{
  for (size_t r = 0; r < rows(); r++)
    delete _grid[r];
}

template<class T>
SmartGrid<T> &
SmartGrid<T>::operator=(const SmartGrid &rhs)
{
  //copy grid elements
  _elements = rhs._elements;

  //remove the old grid
  for (size_t r = 0; r < rows(); r++)
    delete _grid[r];
  _grid.clear();
  //delete后要clear，使得第一维大小变0 因为delete不改变大小
  
  //initialize a new NULL grid with correct size
  for (size_t r = 0; r < rhs.rows(); r++)
  {
    _grid.push_back(new std::vector<T *>());
    (*_grid.back()).resize(rhs.cols(), NULL);
  }

  //go through all the new elements and add the pointers one by one
  typename gridElements::iterator it = _elements.begin();
  while (it != _elements.end())
  {

    (*_grid[it->row()])[it->col()] = &const_cast<T &>(*it);
    it++;
  }

  return *this;
}

template<class T>
void
SmartGrid<T>::resize(size_t rows, size_t cols)
{
  this->reset();
  //列表_element清空，网格全置为空指针
  //多加的行数
  _grid.reserve(rows);
  for (size_t r = 0; r < rows; r++)
  {
    _grid.push_back(new std::vector<T>());
    (*_grid.back()).resize(cols, NULL);
  }
}


template<class T>
void
SmartGrid<T>::erode(size_t radius, size_t border, double ratio)
{
  //剔除周围不存在点太多的点
  // have to save a temporary copy of the depth map
  SmartGrid gridTmp(rows(), cols());
  // first transfer all the existing elements
  typename gridElements::iterator it = _elements.begin();
  while (it != _elements.end())
  {
    // set：如果空指针，先建立新的_elements元素，指向它，直接拷贝（同时修改了_elemnents）
    //（因为有时候存在直接覆盖的情况）
    //_elements中存储实例，smartgrid根据实例中的行列值，存储指向_elements实例的指针
    gridTmp.set(it->row(), it->col(), *it);
    it++;
  }

  size_t empty_pixel_count;
  size_t num_overall_pixel = (2 * radius + 1)*(2 * radius + 1);
//  size_t num_inner_circle_pixel = (2 * (radius - 1) + 1) * (2 * (radius - 1) + 1);

  //erode if not enough neightbours exist(within radius)
  it = gridTmp._elements.begin();
  auto it2 = _elements.begin();
  //再对临时smartgrid的_elements列表遍历
  while (it != gridTmp._elements.end())
  {
    empty_pixel_count = 0;
    //针对_elements中的每个元素，检查其坐标周围是否存在足够多的_elements的元素
    for (int r = it->row() - radius; r < it->row() + radius + 1; r++)
    {
      for (int c = it->col() - radius; c < it->col() + radius + 1; c++)
      {
        //check whether this location is inside the image，那为什么还要个border
        if (r >= border && r < (int) rows() - border && c >= border && c < (int) cols() - border)
        {
          //check whether that element is missing
          if ((*(gridTmp._grid[r]))[c] == NULL)
          {
            empty_pixel_count++;
          }
        }
        else
        {
          empty_pixel_count++;
        }
      }
    }

    // erosion critera，没事的话继续下一个
    if(empty_pixel_count >= (size_t)(num_overall_pixel * ratio))
    {
      typename gridElements::iterator temp = it2;
      it2++;
      remove(temp);
    }
    else
      it2++;
    //这里else只有这一句
    it++;
    }
  //用副本的原因：以最初的深度点分布，来判定哪些需要去掉。
}

template<class T>
void
SmartGrid<T>::dilate(size_t radius)
{
  //在每个生成点的半径内填充满初始化的点
  //have to save a temporary copy of the depth map
  SmartGrid gridTmp(rows(), cols());

  //first transfer all the existing elements
  typename gridElements::iterator it = _elements.begin();
  while (it != _elements.end())
  {
    gridTmp.set(it->row(), it->col(), *it);
    it++;
  }

  //now add all the neightbours (within radius)
  it = gridTmp._elements.begin();
  while (it != gridTmp._elements.end())
  {
    for (int r = it->row() - radius; r < it->row() + radius + 1; r++)
    {
      for (int c = it->col() - radius; c < it->col() + radius + 1; c++)
      {
        //check whether this location is inside the image
        if (r >= 0 && r < (int) rows() &&
            c >= 0 && c < (int) cols())
        {
          //check whether that element is missing
          if ((*(_grid[r]))[c] == NULL)
          {
            _elements.push_back(T(r, c));
            (*(_grid[r]))[c] = &const_cast<T &>(_elements.back());
          }
        }
      }
    }

    it++;
  }
  //用副本的原因：增加最初的分布点周围
}

template<class T>
void
SmartGrid<T>::clean(
  double var_threshold,
  double age_threshold,
  double range_max,
  double range_min)
{
  //first clean the elements and remove any items that are not valid anymore
  //考虑到erase导致迭代器失效，先对其加法。貌似直接it++也可以
  //为什么需要考虑valid
  typename gridElements::iterator it = _elements.begin();
  while (it != _elements.end())
  {
    typename gridElements::iterator temp = it;
    it++;

    if (!temp->valid(var_threshold, age_threshold, range_max, range_min))
    {
      (*_grid[temp->row()])[temp->col()] = NULL;
      _elements.erase(temp);
    }
  }
}

template<class T>
void
SmartGrid<T>::clear()
{
  typename gridElements::iterator it = _elements.begin();
  while (it != _elements.end())
  {
    typename gridElements::iterator temp = it;
    it++;
    if (!temp->valid())
    {
      (*_grid[temp->row()])[temp->col()] = NULL;
      _elements.erase(temp);
    }
  }
}

template<class T>
void
SmartGrid<T>::reset()
{
  for (size_t r = 0; r < rows(); r++)
  {
    int size = _grid[r]->size();
    _grid[r]->assign(size, NULL);
  }
  _elements.clear();
}

template<class T>
const T &
SmartGrid<T>::operator()(size_t row, size_t col) const
{
  if ((*_grid[row])[col] != NULL)
    return *((*_grid[row])[col]);
  return _invalid;
}

template<class T>
const T &
SmartGrid<T>::at(size_t row, size_t col) const
{
  if ((*_grid[row])[col] != NULL)
    return *((*_grid[row])[col]);
  return _invalid;
}

template<class T>
size_t
SmartGrid<T>::rows() const
{
  return _grid.size();
}

 //_grid.front() 取出第一个vector<T*>指针
template<class T>
size_t
SmartGrid<T>::cols() const
{
  return (*_grid.front()).size();
}

  
template<class T>
void
SmartGrid<T>::set(size_t row, size_t col, const T &value)
{
  if ((*_grid[row])[col] == NULL)
  {
    //指针为null，初始化该像素点对应的深度点放入列表，取其地址
    _elements.push_back(T(row, col));
    (*(_grid[row]))[col] = &const_cast<T &>(_elements.back());
  }
  //指针不为空的情况下直接赋值，注意实际是对列表中的元素进行修改
  (*_grid[row])[col]->copy(value);
}

template<class T>
bool
SmartGrid<T>::exists(size_t row, size_t col)
{
  if ((*_grid[row])[col] == NULL)
    return false;
  return true;
}

template<class T>
T &
SmartGrid<T>::get(size_t row, size_t col)
{
  if ((*_grid[row])[col] == NULL)
    return _invalid;
  return *((*_grid[row])[col]);
}

template<class T>
typename SmartGrid<T>::gridElements::iterator
SmartGrid<T>::remove(typename SmartGrid<T>::gridElements::iterator &it)
{
  size_t row = it->row();
  size_t col = it->col();
  (*_grid[row])[col] = NULL;
  return _elements.erase(it);
}

template<class T>
typename SmartGrid<T>::gridElements::iterator
SmartGrid<T>::begin()
{
  return _elements.begin();
}

template<class T>
typename SmartGrid<T>::gridElements::iterator
SmartGrid<T>::end()
{
  return _elements.end();
}

template<class T>
size_t
SmartGrid<T>::size() const
{
  return _elements.size();
}

template<class T>
void
SmartGrid<T>::getNeighbourhood(
  size_t row, size_t col, size_t radius, std::vector<T *> &neighbours)
{
  neighbours.reserve((2 * radius + 1) * (2 * radius + 1));
  for (int r = row - radius; r <= row + radius; r++)
  {
    for (int c = col - radius; c <= col + radius; c++)
    {
      //check whether this location is inside the image
      if (r >= 0 && r < (int) rows() &&
          c >= 0 && c < (int) cols())
    
        //判断指针是否为null，at常量函数常量引用返回值，get引用返回值
        if (exists(r, c) && at(r, c).valid())
          neighbours.push_back(&get(r, c));
      }
    }
  }
}
}
}
#endif //ESVO_CORE_CONTAINER_SMARTGRID_H
