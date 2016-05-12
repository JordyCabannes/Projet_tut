#ifndef VIEWPATHS_H
#define VIEWPATHS_H

#include <iostream>
#include "../Interface/Path.h"
#include <map>

#include <fstream>

#include <src/path.hpp>

class viewpaths
{
 public:
  std::vector<Path> _paths;
  std::vector<unsigned int> _costs;

  viewpaths()
    {
      
    }
  
  unsigned int import_paths(std::string file_path)
  {
    std::fstream stream(file_path);
    
    path_type path;
    
    std::cout << file_path << std::endl;

    int i = 0;

    if(stream.good())
      {
      	while(parse_path(stream, path))
      	  {
            i++;
            std::cout << "i =" << i << std::endl;
            
            std::cout << "cout =" <<path.cost() << std::endl;
      	    _paths.push_back(path);
      	    _costs.push_back(path.cost());

            std::cout << "path =" << _paths.back().edges.front() << std::endl;
            path.clear();
      	  }
      	stream.close();
      }
    
    return _paths.size();
  }

  Path path(unsigned int i)
  {
    return _paths[i];
  }

  unsigned int cost(unsigned int i)
  {
    return _costs[i];
  }
  

};

#endif
